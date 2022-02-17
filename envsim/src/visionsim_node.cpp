#include <ros/ros.h>

#include "agiros_msgs/QuadState.h"
#include "envsim/visionsim.hpp"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"

using namespace agi;

VisionSim::VisionSim(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh) {
  bool use_bem = false;
  pnh_.getParam("use_bem_propeller_model", use_bem);
  pnh_.getParam("real_time_factor", real_time_factor_);
  pnh_.getParam("render", render_);
  pnh_.getParam("param_dir", param_directory_);

  // Logic subscribers
  reset_sub_ = pnh_.subscribe("reset_sim", 1, &VisionSim::resetCallback, this);
  reload_quad_sub_ = pnh_.subscribe("reload_quadrotor", 1,
                                    &VisionSim::loadQuadrotorCallback, this);
  reload_mockvio_sub_ = pnh_.subscribe(
    "reload_mockvio", 1, &VisionSim::loadMockVIOParamsCallback, this);

  // Publishers
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
  odometry_pub_ = pnh_.advertise<nav_msgs::Odometry>("groundtruth/odometry", 1);
  state_pub_ = pnh_.advertise<agiros_msgs::QuadState>("groundtruth/state", 1);
  delayed_state_pub_ =
    pnh_.advertise<agiros_msgs::QuadState>("groundtruth_bf_vel/state", 1);
  mockvio_state_pub_ =
    pnh_.advertise<agiros_msgs::QuadState>("mockvio/state", 1);

  image_transport::ImageTransport it(pnh_);
  image_pub_ = it.advertise("unity/image", 1);

  ros_pilot_.getQuadrotor(&quad_);


  std::cout << "Hellow world" << std::endl;
  std::cout << vision_env_.getActDim() << std::endl;

  // hacky solution to pass the thrust map to the low level controller
  simulator_.setParamRoot(ros_pilot_.getPilot().getParams().directory_);

  // Build simulation pipeline with simple model
  simulator_.updateQuad(quad_);
  simulator_.addModel(ModelInit{quad_});
  simulator_.addModel(ModelMotor{quad_});
  if (use_bem) {
    std::cout << "Using BEM to model thrust" << std::endl;
    auto bem_model = simulator_.addModel(ModelPropellerBEM{quad_});
    auto drag_model = simulator_.addModel(ModelBodyDrag{quad_});
    std::string directory;
    const bool got_directory = pnh.getParam("param_dir", directory);
    if (got_directory) {
      bem_model->ModelBase::setParameters(directory +
                                          "/quads/sim_kingfisher.yaml");
      drag_model->ModelBase::setParameters(directory +
                                           "/quads/sim_kingfisher.yaml");
    } else {
      ROS_WARN("Could not load BEM parameters!");
    }
  } else {
    std::cout << "Not using BEM to model thrust" << std::endl;
    simulator_.addModel(ModelThrustTorqueSimple{quad_});
    simulator_.addModel(ModelDrag{quad_});
  }
  simulator_.addModel(ModelRigidBody{quad_});

  QuadState quad_state;

  // mock VIO stuff
  delayed_estimate_params_ = std::make_shared<MockVioParams>();
  delayed_estimate_params_->bodyframe_vel = true;
  delayed_estimate_params_->latency = 0.0;  // TODO: parameterize
  delayed_estimate_ =
    std::make_shared<MockVio>(quad_, delayed_estimate_params_);

  mock_vio_params_ = std::make_shared<MockVioParams>();
  std::string mock_vio_param_file = param_directory_ + "/mock_vio.yaml";
  mock_vio_params_->load(mock_vio_param_file);
  mock_vio_ = std::make_shared<MockVio>(quad_, mock_vio_params_);

  t_start_ = ros::WallTime::now();

  sim_thread_ = std::thread(&VisionSim::simLoop, this);
}

VisionSim::~VisionSim() {
  if (sim_thread_.joinable()) sim_thread_.join();
  if (render_thread_.joinable()) render_thread_.join();
}

void VisionSim::resetCallback(const std_msgs::EmptyConstPtr &msg) {
  ROS_INFO("Resetting simulator!");
  QuadState reset_state;
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    simulator_.reset(false);
    simulator_.setCommand(Command(0.0, 0.0, Vector<3>::Zero()));
    simulator_.getState(&reset_state);
  }

  reset_state.t += t_start_.toSec();
  {
    const std::lock_guard<std::mutex> lock(mockvio_mutex_);
    delayed_estimate_->reset(reset_state);
    mock_vio_->reset(reset_state);
  }
}

void VisionSim::loadQuadrotorCallback(const std_msgs::StringConstPtr &msg) {
  // this changes the simulated quadrotor, the controllers or the pilot are not
  // aware of the change
  ROS_INFO("Reloading quadrotor from [%s]", msg->data.c_str());

  Quadrotor quad;
  if (!quad.load(msg->data)) {
    ROS_FATAL("Could not load quadrotor.");
    ros::shutdown();
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    if (!simulator_.updateQuad(quad)) {
      ROS_FATAL("Failed to update quadrotor.");
      ros::shutdown();
      return;
    }
  }
}


void VisionSim::simLoop() {
  while (ros::ok()) {
    ros::WallTime t_start_sim = ros::WallTime::now();
    QuadState quad_state;
    {
      const std::lock_guard<std::mutex> lock(sim_mutex_);
      simulator_.getState(&quad_state);
    }

    // we add an offset to have realistic timestamps
    Scalar sim_time = quad_state.t;
    quad_state.t += t_start_.toSec();

    rosgraph_msgs::Clock curr_time;
    curr_time.clock.fromSec(quad_state.t);
    clock_pub_.publish(curr_time);

    // sleep for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // feed mock vio estimates to the pilot
    QuadState delayed_state, mock_vio_state;
    delayed_state.setZero();
    mock_vio_state.setZero();
    {
      const std::lock_guard<std::mutex> lock(mockvio_mutex_);
      if (!delayed_estimate_->addState(quad_state)) {
        ROS_WARN("Could not add state at %1.6g\n", quad_state.t);
      }
      if (!delayed_estimate_->getAt(quad_state.t, &delayed_state)) {
        ROS_WARN("Could not get state at %1.6g\n", quad_state.t);
      }

      if (!mock_vio_->addState(quad_state)) {
        ROS_WARN("Could not add state at %1.6g\n", quad_state.t);
      }
      if (!mock_vio_->getAt(quad_state.t, &mock_vio_state)) {
        ROS_WARN("Could not get state at %1.6g\n", quad_state.t);
      }
    }
    publishStates(quad_state, delayed_state, mock_vio_state);

    // if we want that MPC also operates on the mockvio estimates, toggle the
    // lines below
    ros_pilot_.getPilot().odometryCallback(quad_state);
    //    ros_pilot_.getPilot().odometryCallback(mock_vio_state);

    Command cmd = ros_pilot_.getCommand();
    // investigate the round trip time from the cmd timestamp
    //    ROS_INFO("Cmd delay: %.3f", quad_state.t - cmd.t);
    cmd.t -= t_start_.toSec();
    if (ros_pilot_.feedthroughActive()) {
      // we only add delay to this command type, otherwise MPC fails
      cmd.t += 0.04;  // TODO: parameterize
      //      ROS_INFO("Cmd delay: %.3f", quad_state.t - t_start_.toSec() -
      //      cmd.t);
    }
    if (cmd.valid()) {
      {
        const std::lock_guard<std::mutex> lock(sim_mutex_);
        simulator_.setCommand(cmd);
      }
    } else {
      Command zero_cmd;
      zero_cmd.t = sim_time;  // quad_state.t;
      zero_cmd.thrusts.setZero();
      {
        const std::lock_guard<std::mutex> lock(sim_mutex_);
        simulator_.setCommand(zero_cmd);
      }
    }
    {
      const std::lock_guard<std::mutex> lock(sim_mutex_);
      if (!simulator_.run(sim_dt_))
        ROS_WARN_THROTTLE(1.0, "Simulation failed!");
    }
    // Render here stuff
    if (render_) {
      if ((step_counter_ + 1) % render_every_n_steps_ == 0) {
        publishImages(quad_state);
        step_counter_ = 0;
      } else {
        step_counter_ += 1;
      }
    }
    Scalar sleep_time = 1.0 / real_time_factor_ * sim_dt_ -
                        (ros::WallTime::now() - t_start_sim).toSec();
    ros::WallDuration(std::max(sleep_time, 0.0)).sleep();
  }
}

void VisionSim::publishStates(const QuadState &state,
                              const QuadState &delayed_state,
                              const QuadState &mockvio_state) {
  agiros_msgs::QuadState msg_state;
  msg_state.header.frame_id = "world";
  msg_state.header.stamp = ros::Time(state.t);
  msg_state.t = state.t;
  msg_state.pose.position = toRosPoint(state.p);
  msg_state.pose.orientation = toRosQuaternion(state.q());
  msg_state.velocity.linear = toRosVector(state.v);
  msg_state.velocity.angular = toRosVector(state.w);
  msg_state.acceleration.linear = toRosVector(state.a);
  msg_state.acceleration.angular = toRosVector(state.tau);

  agiros_msgs::QuadState msg_delayed_state;
  msg_delayed_state.header.frame_id = "world";
  msg_delayed_state.header.stamp = ros::Time(delayed_state.t);
  msg_delayed_state.t = delayed_state.t;
  msg_delayed_state.pose.position = toRosPoint(delayed_state.p);
  msg_delayed_state.pose.orientation = toRosQuaternion(delayed_state.q());
  msg_delayed_state.velocity.linear = toRosVector(delayed_state.v);
  msg_delayed_state.velocity.angular = toRosVector(delayed_state.w);
  msg_delayed_state.acceleration.linear = toRosVector(delayed_state.a);
  msg_delayed_state.acceleration.angular = toRosVector(delayed_state.tau);

  agiros_msgs::QuadState mockvio_msg;
  mockvio_msg.header.frame_id = "world";
  mockvio_msg.header.stamp = ros::Time(mockvio_state.t);
  mockvio_msg.t = mockvio_state.t;
  mockvio_msg.pose.position = toRosPoint(mockvio_state.p);
  mockvio_msg.pose.orientation = toRosQuaternion(mockvio_state.q());
  mockvio_msg.velocity.linear = toRosVector(mockvio_state.v);
  mockvio_msg.velocity.angular = toRosVector(mockvio_state.w);
  mockvio_msg.acceleration.linear = toRosVector(mockvio_state.a);
  mockvio_msg.acceleration.angular = toRosVector(mockvio_state.tau);

  nav_msgs::Odometry msg_odo;
  msg_odo.header.frame_id = "world";
  msg_odo.header.stamp = ros::Time(state.t);
  msg_odo.pose.pose = msg_state.pose;
  msg_odo.twist.twist = msg_state.velocity;

  odometry_pub_.publish(msg_odo);
  state_pub_.publish(msg_state);
  delayed_state_pub_.publish(msg_delayed_state);
  mockvio_state_pub_.publish(mockvio_msg);
}

void VisionSim::publishImages(const QuadState &state) {
  sensor_msgs::ImagePtr rgb_msg;
  //   frame_id_ += 1;
  //   // render the frame
  //   flightlib::QuadState unity_quad_state;
  //   unity_quad_state.setZero();
  //   unity_quad_state.p = state.p.cast<flightlib::Scalar>();
  //   unity_quad_state.qx = state.qx.cast<flightlib::Scalar>();
  //   unity_quad_->setState(unity_quad_state);
  //   // Warning, delay
  //   unity_bridge_->getRender(frame_id_);
  //   unity_bridge_->handleOutput(frame_id_);
  //   cv::Mat img;
  //   unity_camera_->getRGBImage(img);
  //   rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
  //   img).toImageMsg(); rgb_msg->header.stamp = ros::Time(state.t);
  //   image_pub_.publish(rgb_msg);
  // }

  // bool VisionSim::setUnity(const bool render) {
  //   unity_render_ = render;
  //   if (unity_render_ && unity_bridge_ == nullptr) {
  //     unity_bridge_ = flightlib::UnityBridge::getInstance();
  //     unity_bridge_->addQuadrotor(unity_quad_);
  //     //
  //     if (!loadRacetrack(pnh_)) {
  //       ROS_WARN("[%s] No race track is specified.",
  //       pnh_.getNamespace().c_str());
  //     } else {
  //     };

  //     ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  //     return true;
  //   } else {
  //     return false;
  //   }
}

void VisionSim::loadMockVIOParamsCallback(const std_msgs::StringConstPtr &msg) {
  ROS_INFO("Reloading MockVIO parameters from [%s]", msg->data.c_str());
  {
    const std::lock_guard<std::mutex> lock(mockvio_mutex_);
    mock_vio_params_ = std::make_shared<MockVioParams>();
    if (!mock_vio_params_->load(msg->data)) {
      ROS_FATAL("Could not load MockVIO parameters.");
      ros::shutdown();
      return;
    }
    mock_vio_ = std::make_shared<MockVio>(quad_, mock_vio_params_);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visionsim_node");

  VisionSim vision_sim;

  ros::spin();
  return 0;
}