#include <ros/ros.h>

#include "agiros_msgs/QuadState.h"
#include "environment/environment.hpp"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"


namespace fli = flightlib;
using namespace agi;


Environment::Environment(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    unity_scene_id_(fli::UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(true) {
  bool use_bem = false;
  pnh_.getParam("use_bem_propeller_model", use_bem);
  pnh_.getParam("real_time_factor", real_time_factor_);
  pnh_.getParam("render", render_);
  pnh_.getParam("param_dir", param_directory_);

  // Logic subscribers
  reset_sub_ =
    pnh_.subscribe("reset_sim", 1, &Environment::resetCallback, this);
  reload_quad_sub_ = pnh_.subscribe("reload_quadrotor", 1,
                                    &Environment::loadQuadrotorCallback, this);
  reload_mockvio_sub_ = pnh_.subscribe(
    "reload_mockvio", 1, &Environment::loadMockVIOParamsCallback, this);

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

  if (render_) {
    // Flightmare Quadrotor and Unity Camera
    unity_quad_ = std::make_shared<fli::Quadrotor>();
    fli::Vector<3> quad_size(0.5, 0.5, 0.2);
    unity_quad_->setSize(quad_size);

    unity_camera_ = std::make_shared<fli::RGBCamera>();
    fli::Vector<3> B_r_BC(0.0, 0.0, 0.3);
    fli::Scalar pitch_angle_deg = 0.0;
    fli::Matrix<3, 3> R_BC =
      (Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(-pitch_angle_deg / 180.0 * M_PI,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
    double hor_fov_radians = (M_PI * (110 / 180.0));
    double width = 640.0;
    double height = 480.0;
    // Recalculate here: https://themetalmuncher.github.io/fov-calc/;
    double vertical_fov =
      2. * std::atan(std::tan(hor_fov_radians / 2) * height / width);
    vertical_fov = (vertical_fov / M_PI) * 180.0;  // convert back to degrees
    std::cout << "Vertical FoV is " << vertical_fov << std::endl;
    unity_camera_->setFOV(vertical_fov);
    unity_camera_->setWidth(width);
    unity_camera_->setHeight(height);
    unity_camera_->setRelPose(B_r_BC, R_BC);
    unity_quad_->addRGBCamera(unity_camera_);

    // Connect Unity
    setUnity(unity_render_);
    connectUnity();
  }

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

  sim_thread_ = std::thread(&Environment::simLoop, this);
}

Environment::~Environment() {
  if (sim_thread_.joinable()) sim_thread_.join();
  if (render_thread_.joinable()) render_thread_.join();
}

void Environment::resetCallback(const std_msgs::EmptyConstPtr &msg) {
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

void Environment::loadQuadrotorCallback(const std_msgs::StringConstPtr &msg) {
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


void Environment::simLoop() {
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

void Environment::publishStates(const QuadState &state,
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

void Environment::publishImages(const QuadState &state) {
  sensor_msgs::ImagePtr rgb_msg;
  frame_id_ += 1;
  // render the frame
  fli::QuadState unity_quad_state;
  unity_quad_state.setZero();
  unity_quad_state.p = state.p.cast<fli::Scalar>();
  unity_quad_state.qx = state.qx.cast<fli::Scalar>();
  unity_quad_->setState(unity_quad_state);
  // Warning, delay
  unity_bridge_->getRender(frame_id_);
  unity_bridge_->handleOutput(frame_id_);
  cv::Mat img;
  unity_camera_->getRGBImage(img);
  rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = ros::Time(state.t);
  image_pub_.publish(rgb_msg);
}

bool Environment::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ == nullptr) {
    unity_bridge_ = fli::UnityBridge::getInstance();
    unity_bridge_->addQuadrotor(unity_quad_);
    //
    if (!loadRacetrack(pnh_)) {
      ROS_WARN("[%s] No race track is specified.", pnh_.getNamespace().c_str());
    } else {
    };

    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
    return true;
  } else {
    return false;
  }
}

bool Environment::connectUnity() {
  if (!unity_render_ || unity_bridge_ == nullptr) return false;
  unity_ready_ = unity_bridge_->connectUnity(unity_scene_id_);
  return unity_ready_;
}

bool Environment::loadRacetrack(const ros::NodeHandle &nh) {
  std::string config_file;
  if (!nh.getParam("race_config", config_file)) {
    ROS_ERROR("[%s] Could not load race track configuration.",
              pnh_.getNamespace().c_str());
    return false;
  } else {
    ROS_INFO("[%s] Loading race track configuration.",
             pnh_.getNamespace().c_str());
    YAML::Node config_node = YAML::LoadFile(config_file);
    Scalar num_gate = config_node["gates"]["N"].as<Scalar>();
    for (size_t i = 0; i < num_gate; i++) {
      std::string gate_id = "Gate" + std::to_string(i + 1);
      std::string prefab_id = "rpg_gate";

      // load gate position, rotation, and scale
      std::vector<fli::Scalar> pos_vec =
        config_node["gates"][gate_id]["position"]
          .as<std::vector<fli::Scalar>>();
      std::vector<fli::Scalar> quat_vec =
        config_node["gates"][gate_id]["rotation"]
          .as<std::vector<fli::Scalar>>();
      std::vector<fli::Scalar> scale_vec =
        config_node["gates"][gate_id]["scale"].as<std::vector<fli::Scalar>>();

      // create gate
      std::shared_ptr<fli::StaticGate> gate =
        std::make_shared<fli::StaticGate>(gate_id, prefab_id);
      gate->setPosition(fli::Vector<3>(pos_vec.data()));
      gate->setRotation(
        fli::Quaternion(quat_vec[0], quat_vec[1], quat_vec[2], quat_vec[3]));
      gate->setSize(fli::Vector<3>(
        scale_vec.data()));  // for visualization, not the acutal physical size

      unity_gates_.push_back(gate);
      unity_bridge_->addStaticObject(gate);

      //
      std::vector<fli::Scalar> start_pos_vec =
        config_node["start_pos"].as<std::vector<fli::Scalar>>();
      std::vector<fli::Scalar> goal_pos_vec =
        config_node["goal_pos"].as<std::vector<fli::Scalar>>();
      start_pos_ << start_pos_vec[0], start_pos_vec[1], start_pos_vec[2];
      goal_pos_ << goal_pos_vec[0], goal_pos_vec[1], goal_pos_vec[2];
    }
    return true;
  }
}

void Environment::loadMockVIOParamsCallback(
  const std_msgs::StringConstPtr &msg) {
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
  ros::init(argc, argv, "environment_node");

  Environment environment;

  ros::spin();
  //    ros::MultiThreadedSpinner spinner(2);
  //    spinner.spin();

  return 0;
}