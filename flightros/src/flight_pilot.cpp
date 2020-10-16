#include "flightros/flight_pilot.hpp"
#include <cv_bridge/cv_bridge.h>

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    it_(nh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0),
    use_rgb_(false),
    use_depth_(false),
    use_semantics_(false),
    use_optical_flow_(false)
{
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Matrix<3, 3> R_BC = q_BC_.toRotationMatrix();
  std::cout << "Rotation Camera to Body: \n" << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC_, R_BC);
  rgb_camera_->enableDepth(use_depth_);
  rgb_camera_->enableSegmentation(use_semantics_);
  rgb_camera_->enableOpticalFlow(use_optical_flow_);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // initialize publishers
  if(use_rgb_) {
    rbg_img_pub_ = it_.advertise("rgb_image", 1);
  }
  if(use_depth_) {
    depth_img_pub_ = it_.advertise("depth_image", 1);
  }
  if(use_semantics_) {
    semantic_img_pub_ = it_.advertise("semantic_image", 1);
  }
  if(use_optical_flow_) {
    optical_flow_img_pub_ = it_.advertise("optical_flow_image", 1);
  }

  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    // Output msg
    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = "cam0";
    image_msg.header.stamp = ros::Time::now();
    image_msg.encoding = "bgr8";
  
    // read the image
    if(use_rgb_) {
        cv::Mat rgb_image;
        if(!rgb_camera_->getRGBImage(rgb_image)) {
          ROS_ERROR("[%s] Could not fetch RGB image.", pnh_.getNamespace().c_str());
          return;
        }

        // publish image over ROS
        image_msg.image = rgb_image;
        rbg_img_pub_.publish(image_msg.toImageMsg());
    }

    if(use_depth_) {
        cv::Mat depth_image;
        if(!rgb_camera_->getDepthMap(depth_image)) {
          ROS_ERROR("[%s] Could not fetch depth image.",
                    pnh_.getNamespace().c_str());
          return;
        }

        // publish image over ROS
        image_msg.image = depth_image;
        depth_img_pub_.publish(image_msg.toImageMsg());
    }

    if(use_semantics_) {
        cv::Mat semantic_image;
        if(!rgb_camera_->getSegmentation(semantic_image)) {
          ROS_ERROR("[%s] Could not fetch semantic image.",
                    pnh_.getNamespace().c_str());
          return;
        }

        // publish image over ROS
        image_msg.image = semantic_image;
        semantic_img_pub_.publish(image_msg.toImageMsg());
    }

    if(use_optical_flow_) {
        cv::Mat optical_flow;
        if(!rgb_camera_->getOpticalFlow(optical_flow)) {
          ROS_ERROR("[%s] Could not fetch optical flow.",
                    pnh_.getNamespace().c_str());
          return;
        }

        // publish image over ROS
        image_msg.image = optical_flow;
        optical_flow_img_pub_.publish(image_msg.toImageMsg());
    }
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  int scene_id_tmp;
  quadrotor_common::getParam("scene_id", scene_id_tmp, pnh_);
  scene_id_ = SceneID(scene_id_tmp);

  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  // Camera-body transformation
  quadrotor_common::getParam("t_BC_x", B_r_BC_.x(), pnh_);
  quadrotor_common::getParam("t_BC_y", B_r_BC_.y(), pnh_);
  quadrotor_common::getParam("t_BC_z", B_r_BC_.z(), pnh_);

  quadrotor_common::getParam("q_BC_x", q_BC_.x(), pnh_);
  quadrotor_common::getParam("q_BC_y", q_BC_.y(), pnh_);
  quadrotor_common::getParam("q_BC_z", q_BC_.z(), pnh_);
  quadrotor_common::getParam("q_BC_w", q_BC_.w(), pnh_);
  q_BC_.normalize();

  // Output auxiliaries
  quadrotor_common::getParam("use_rgb", use_rgb_, pnh_);
  quadrotor_common::getParam("use_depth", use_depth_, pnh_);
  quadrotor_common::getParam("use_semantics", use_semantics_, pnh_);
  quadrotor_common::getParam("use_optical_flow", use_optical_flow_, pnh_);

  return true;
}

}  // namespace flightros
