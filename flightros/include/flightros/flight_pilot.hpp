
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// rpg quadrotor
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightPilot();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  // publisher
  image_transport::Publisher rbg_img_pub_;
  image_transport::Publisher depth_img_pub_;
  image_transport::Publisher semantic_img_pub_;
  image_transport::Publisher optical_flow_img_pub_;

  ros::Publisher rgb_camera_info_pub_;
  ros::Publisher depth_camera_info_pub_;
  ros::Publisher dense_pcl_pub_;

  // subscriber
  ros::Subscriber sub_state_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{50.0};

  Vector<3> B_r_BC_;
  Quaternion q_BC_;

  // output
  bool use_rgb_;
  bool use_depth_;
  bool use_semantics_;
  bool use_optical_flow_;
};
}  // namespace flightros
