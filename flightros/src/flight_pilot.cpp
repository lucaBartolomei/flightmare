#include "flightros/flight_pilot.hpp"
#include <cv_bridge/cv_bridge.h>

namespace flightros
{

FlightPilot::FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), it_(nh), scene_id_(UnityScene::WAREHOUSE),
      unity_ready_(false), unity_render_(false), receive_id_(0),
      main_loop_freq_(50.0), use_rgb_(false), use_depth_(false),
      use_semantics_(false), use_optical_flow_(false)
{
  // load parameters
  if (!loadParams())
  {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  }
  else
  {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Matrix<3, 3> R_BC = q_BC_.toRotationMatrix();
  std::cout << "Rotation Camera to Body: \n" << R_BC << std::endl;
  rgb_camera_->setFOV(80);
  rgb_camera_->setWidth(752);
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
  if (use_rgb_)
  {
    rbg_img_pub_ = it_.advertise("rgb_image", 1);
    rgb_camera_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("rgb_image/camera_info", 1);
  }
  if (use_depth_)
  {
    depth_img_pub_ = it_.advertise("depth_image", 1);
    depth_camera_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("depth_image/camera_info", 1);
    dense_pcl_pub_ =
        nh_.advertise<sensor_msgs::PointCloud>("depth_image/pointcloud", 1);
  }
  if (use_semantics_)
  {
    semantic_img_pub_ = it_.advertise("semantic_image", 1);
  }
  if (use_optical_flow_)
  {
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

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_)
  {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    // Output msg
    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = "cam0";
    image_msg.header.stamp = ros::Time::now();
    image_msg.encoding = "bgr8";

    // read the image
    if (use_rgb_)
    {
      cv::Mat rgb_image;
      if (!rgb_camera_->getRGBImage(rgb_image))
      {
        ROS_ERROR("[%s] Could not fetch RGB image.",
                  pnh_.getNamespace().c_str());
        return;
      }

      // publish image over ROS
      image_msg.image = rgb_image;
      rbg_img_pub_.publish(image_msg.toImageMsg());

      // Generate the camera info message
      sensor_msgs::CameraInfo camera_info;
      camera_info.width = rgb_camera_->getWidth();
      camera_info.height = rgb_camera_->getHeight();
      camera_info.distortion_model = "plum_bob";
      float f = (camera_info.height / 2.0f) /
                tan((M_PI * (rgb_camera_->getFOV() / 180.0f)) / 2.0f);
      float cx = camera_info.width / 2.0;
      float cy = camera_info.height / 2.0;
      float tx = 0.0;
      float ty = 0.0;
      camera_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      camera_info.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
      camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0, 0.0};
      rgb_camera_info_pub_.publish(camera_info);
    }

    // Insert data in the map
    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = "world";
    pcl_msg.header.stamp = ros::Time::now();

    if (use_depth_)
    {
      cv::Mat depth_image;
      if (!rgb_camera_->getDepthMap(depth_image))
      {
        ROS_ERROR("[%s] Could not fetch depth image.",
                  pnh_.getNamespace().c_str());
        return;
      }

      // publish image over ROS
      image_msg.image = depth_image;
      // image_msg.encoding = "mono8";
      depth_img_pub_.publish(image_msg.toImageMsg());

      // Generate the camera info message
      sensor_msgs::CameraInfo depth_camera_info;
      depth_camera_info.width = rgb_camera_->getWidth();
      depth_camera_info.height = rgb_camera_->getHeight();
      depth_camera_info.distortion_model = "plum_bob";
      float f = (depth_camera_info.width / 2.0f) /
                tan((M_PI * (rgb_camera_->getFOV() / 180.0f)) / 2.0f);
      float cx = depth_camera_info.width / 2.0f;
      float cy = depth_camera_info.height / 2.0f;
      float tx = 0.0;
      float ty = 0.0;
      depth_camera_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      depth_camera_info.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
      depth_camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      depth_camera_info.P = {f,  0.0, cx,  tx,  0.0, f,
                             cy, ty,  0.0, 0.0, 1.0, 0.0};
      depth_camera_info_pub_.publish(depth_camera_info);

      // Generate dense point cloud
      // 1- get transform
      Eigen::Matrix4f T_WB(Eigen::Matrix4f::Identity());
      T_WB.block<3, 3>(0, 0) = quad_state_.q().toRotationMatrix();
      T_WB.block<3, 1>(0, 3) = quad_state_.p;

      Eigen::Matrix4f tran_mat = Eigen::Matrix4f::Zero();
      tran_mat(0, 0) = 1.0;
      tran_mat(1, 2) = 1.0;
      tran_mat(2, 1) = 1.0;
      tran_mat(3, 3) = 1.0;

      //Eigen::Matrix4f T_BC = tran_mat.transpose() * rgb_camera_->getRelPose() * tran_mat;

      Eigen::Matrix4f T_BC = rgb_camera_->getRelPose();

      // This additional matrix is to account for the rotations on unity's side
      Eigen::Matrix4f T_rot(Eigen::Matrix4f::Zero());
      //T_rot << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1;
      T_rot << 1, 0, 0, 0,
              0, 0, 1, 0,
               0, -1, 0, 0,
              0, 0, 0, 1;

      Eigen::Matrix4f T_u(Eigen::Matrix4f::Zero());
      T_u << 1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

      Eigen::Matrix4f T_90(Eigen::Matrix4f::Zero());
      T_90 << 1, 0, 0, 0,
              0, 0, 1, 0,
              0, -1, 0, 0,
              0, 0, 0, 1;

      //Eigen::Matrix4f T_counter = Eigen::Matrix4f::Identity();
      //T_counter.block<3,3>(0,0) = Eigen::Quaternionf(0.966, 0.259, 0.0, 0.0).toRotationMatrix();

      Eigen::Matrix4f T_WC(T_WB * T_BC * T_rot);

//      T_WC = T_90 * (tran_mat * T_BC * tran_mat.transpose()).inverse();
//      T_WC = T_WB * T_BC * T_90;

//      Eigen::Matrix4f T_rot(Eigen::Matrix4f::Zero());
//      T_rot << 1, 0, 0, 0,
//          0, 0, 1, 0,
//          0, 1, 0, 0,
//          0, 0, 0, 1;
////      T_rot << 1, 0, 0, 0,
////               0, -1, 0, 0,
////               0, 0, 1, 0,
////               0, 0, 0, 1;
//      Eigen::Matrix4f T_180(Eigen::Matrix4f::Zero());
////      T_180 <<  -1, 0,  0, 0,
////                0, -1, 0, 0,
////                0,  0, 1, 0,
////                0,  0, 0, 1;
//      T_180 <<  -1, 0,  0, 0,
//                0, -1, 0, 0,
//                0,  0, 1, 0,
//                0,  0, 0, 1;
//      Eigen::Matrix4f T_WC(T_WB * T_BC  * T_rot);

      // Extract the only useful channel after transforming data into floats
      cv::Mat depth_image_float;
      depth_image.convertTo(depth_image_float, CV_32FC3);
      cv::Mat depth_channel;
      extractChannel(depth_image_float, depth_channel, 0);

      float* data = (float*)depth_channel.data;
      const float far_distance = 200.f;
      const float scale_factor = far_distance / 255.f; // FIXME This should be dep on params
      for (int u = 0; u < depth_channel.cols; u += 10)
      {
        for (int v = 0; v < depth_channel.rows; v += 10)
        {
          float range = data[v * depth_channel.cols + u] * scale_factor;
          float xd = (u - cx) / f;
          float yd = (v - cy) / f;

          if(range > 30.f) {
              //continue;
          }

          Eigen::Vector4f p(Eigen::Vector4f::Ones());
          p[0] = range * xd;
          p[1] = range * yd;
          p[2] = range;

          Eigen::Vector4f p_W = T_WC * p;
          geometry_msgs::Point32 point_msg;
          point_msg.x = p_W.x();
          point_msg.y = p_W.y();
          point_msg.z = p_W.z();
          pcl_msg.points.push_back(point_msg);
        }
      }
    }

    if (use_semantics_)
    {
      cv::Mat semantic_image;
      if (!rgb_camera_->getSegmentation(semantic_image))
      {
        ROS_ERROR("[%s] Could not fetch semantic image.",
                  pnh_.getNamespace().c_str());
        return;
      }

      // publish image over ROS
      image_msg.image = semantic_image;
      image_msg.encoding = "bgr8";
      semantic_img_pub_.publish(image_msg.toImageMsg());

      // Assign class to the points in point cloud
      if (!pcl_msg.points.empty())
      {
        cv::Mat sem_image_float;
        semantic_image.convertTo(sem_image_float, CV_32FC3);
        cv::Mat sem_bgr[3];
        cv::split(sem_image_float, sem_bgr);

        pcl_msg.channels.resize(2);
        sensor_msgs::ChannelFloat32 semantic_channel, confidence_channel;
        semantic_channel.name = "quality";
        confidence_channel.name = "confidence";
        for (int u = 0; u < sem_image_float.cols; u += 10)
        {
          for (int v = 0; v < sem_image_float.rows; v += 10)
          {
            float blue = sem_bgr[0].at<float>(v, u);
            float green = sem_bgr[1].at<float>(v, u);
            float red = sem_bgr[2].at<float>(v, u);

            // Decide class (just a random example)
            float sem_class;
            if (scene_id_ == UnityScene::WAREHOUSE)
            {
              if (red == 255.f)
              {
                sem_class = 0;
              }
              else if (green == 255.f)
              {
                sem_class = 1;
              }
              else
              {
                sem_class = 2;
              }
            }
            else if (scene_id_ == UnityScene::FLATGROUND)
            {
              if (blue == 255.f)
              {
                sem_class = 0; // water
              }
              else if (red == 255.f)
              {
                sem_class = 1; // land
              }
              else
              {
                sem_class = 2;
              }
            }

            confidence_channel.values.push_back(1.0);
            semantic_channel.values.push_back(sem_class);
          }
        }
        pcl_msg.channels[0] = semantic_channel;
        pcl_msg.channels[1] = confidence_channel;
      }
    }

    if (use_optical_flow_)
    {
      cv::Mat optical_flow;
      if (!rgb_camera_->getOpticalFlow(optical_flow))
      {
        ROS_ERROR("[%s] Could not fetch optical flow.",
                  pnh_.getNamespace().c_str());
        return;
      }

      // publish image over ROS
      image_msg.image = optical_flow;
      optical_flow_img_pub_.publish(image_msg.toImageMsg());
    }

    // Publish point cloud
    dense_pcl_pub_.publish(pcl_msg);
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent& event)
{
  // empty
}

bool FlightPilot::setUnity(const bool render)
{
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr)
  {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity()
{
  if (!unity_render_ || unity_bridge_ptr_ == nullptr)
    return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void)
{
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

} // namespace flightros
