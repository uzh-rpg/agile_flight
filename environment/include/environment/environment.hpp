#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>

// -- ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "std_msgs/String.h"

// -- agilicious
#include "agilib/base/parameter_base.hpp"
#include "agilib/estimator/mock_vio/mock_vio.hpp"
#include "agilib/simulator/model_body_drag.hpp"
#include "agilib/simulator/model_drag.hpp"
#include "agilib/simulator/model_init.hpp"
#include "agilib/simulator/model_motor.hpp"
#include "agilib/simulator/model_propeller_bem.hpp"
#include "agilib/simulator/model_rigid_body.hpp"
#include "agilib/simulator/model_thrust_torque_simple.hpp"
#include "agilib/simulator/quadrotor_simulator.hpp"
#include "agilib/utils/timer.hpp"
#include "agiros/ros_pilot.hpp"

// -- flightmare
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"


namespace fli = flightlib;

namespace agi {

class Environment {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Environment(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  Environment() : Environment(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~Environment();

  void configUnityCamera();

 private:
  void resetCallback(const std_msgs::EmptyConstPtr& msg);
  void loadQuadrotorCallback(const std_msgs::StringConstPtr& msg);

  void simLoop();
  void publishStates(const QuadState& state, const QuadState& delayed_state,
                     const QuadState& mockvio_state);
  void publishImages(const QuadState& state);

  bool setUnity(const bool render);
  bool connectUnity();

  bool loadRacetrack();
  void loadMockVIOParamsCallback(const std_msgs::StringConstPtr& msg);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber reset_sub_;
  ros::Subscriber reload_quad_sub_;
  ros::Subscriber reload_mockvio_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher state_pub_;
  ros::Publisher delayed_state_pub_;
  ros::Publisher mockvio_state_pub_;
  ros::Publisher clock_pub_;
  image_transport::Publisher image_pub_;

  Quadrotor quad_;
  QuadrotorSimulator simulator_;
  RosPilot ros_pilot_;
  Scalar sim_dt_ = 0.01;
  Scalar camera_dt_ = 0.04;  // 20 Hz. Should be a multiple of sim_dt_
  int render_every_n_steps_ = camera_dt_ / sim_dt_;
  int step_counter_ = 0;
  Scalar real_time_factor_ = 1.0;
  bool render_ = false;
  ros::WallTime t_start_;

  // -- Flightmare Unity3D
  std::shared_ptr<fli::Quadrotor> unity_quad_;
  std::shared_ptr<fli::RGBCamera> unity_camera_;
  std::vector<std::shared_ptr<fli::StaticGate>> unity_gates_;
  std::shared_ptr<fli::UnityBridge> unity_bridge_;
  fli::SceneID unity_scene_id_;
  fli::RenderMessage_t unity_output_;
  bool unity_ready_, unity_render_;
  std::string param_directory_;
  fli::FrameID frame_id_ = 0;

  // -- Race tracks
  Vector<3> start_pos_;
  Vector<3> goal_pos_;

  std::mutex sim_mutex_;
  std::mutex mockvio_mutex_;
  std::thread sim_thread_;
  std::thread render_thread_;

  // Mock VIO stuff
  std::shared_ptr<MockVio> mock_vio_;
  std::shared_ptr<MockVioParams> mock_vio_params_;
  std::shared_ptr<MockVio> delayed_estimate_;
  std::shared_ptr<MockVioParams> delayed_estimate_params_;
};

}  // namespace agi