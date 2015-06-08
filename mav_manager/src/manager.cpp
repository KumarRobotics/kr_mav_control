// mav_manager
#include <manager.h>

// Standard C++
#include <math.h>
#include <string>

// ROS Related
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

// quadrotor_control
#include <trackers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/SO3Command.h>

// Strings
static const std::string line_tracker_distance("std_trackers/LineTrackerDistance");
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerk");
static const std::string velocity_tracker_str("std_trackers/VelocityTracker");
static const std::string null_tracker_str("std_trackers/NullTracker");
static const std::string pos_cmd_tracker_str("trajectory_trackers/PositionCommandTracker");

// Helper functions
double deadband(double val, double deadband);

MAVManager::MAVManager()
    : nh_(""),
      priv_nh_("~"),
      active_tracker_(""),
      last_odom_t_(0),
      last_output_data_t_(0),
      last_imu_t_(0),
      last_heartbeat_t_(0),
      offsets_(0, 0, 0, 0), // TODO: The offsets need to be implemented throughout. Or, maybe not used.
      kGravity_(9.81),
      useRadioForVelocity_(false) {

  // Publishers
  pub_goal_line_tracker_distance_ = nh_.advertise<quadrotor_msgs::LineTrackerGoal>(
      "trackers_manager/line_tracker_distance/goal", 10);
  pub_goal_min_jerk_ = nh_.advertise<quadrotor_msgs::LineTrackerGoal>(
      "trackers_manager/line_tracker_min_jerk/goal", 10);
  pub_goal_velocity_ = nh_.advertise<quadrotor_msgs::FlatOutputs>(
      "trackers_manager/velocity_tracker/goal", 10);
  pub_motors_ = nh_.advertise<std_msgs::Bool>("motors", 10);
  pub_estop_ = nh_.advertise<std_msgs::Empty>("estop", 10);
  pub_so3_command_ = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  pub_position_command_ = nh_.advertise<quadrotor_msgs::PositionCommand>(
      "trackers_manager/position_command_tracker/goal", 10);
  // pwm_command_pub_ = nh_ ...

  // Subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &MAVManager::odometry_cb, this);
  output_data_sub_ =
      nh_.subscribe("output_data", 10, &MAVManager::output_data_cb, this);
  imu_sub_ = nh_.subscribe("imu", 10, &MAVManager::imu_cb, this);
  heartbeat_sub_ =
      nh_.subscribe("/heartbeat", 10, &MAVManager::heartbeat_cb, this);

  // Services
  srv_transition_ = nh_.serviceClient<trackers_manager::Transition>(
      "trackers_manager/transition");

  // Disable motors
  this->motors(false);

  if (!nh_.getParam("mass", mass_))
    ROS_ERROR("Mass must be set");
  else
    ROS_INFO("Using mass = %2.2f", mass_);

  // Sleep to ensure service server initialized
  double duration;
  priv_nh_.param("startup_sleep_duration", duration, 0.25);
  ros::Duration(duration).sleep();

  if (!(this->transition(null_tracker_str)))
    ROS_ERROR("Initial transition to NullTracker failed");

  // Publish so3_command to ensure motors are stopped
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.orientation.x = 0;
  so3_cmd.orientation.y = 0;
  so3_cmd.orientation.z = 0;
  so3_cmd.orientation.w = 1;
  so3_cmd.aux.enable_motors = false;
  pub_so3_command_.publish(so3_cmd);

  motors_ = false;
}

void MAVManager::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  odom_q_ = Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = msg->header.stamp;

  this->heartbeat();
}

bool MAVManager::takeoff() {
  if (!this->have_recent_odom()) {
    ROS_WARN("Cannot takeoff without odometry.");
    return false;
  }

  if (!this->setHome()) return false;

  if (!motors_) {
    ROS_WARN("Cannot takeoff until motors are enabled.");
    return false;
  }

  // Only takeoff if currently under NULL_TRACKER
  if (active_tracker_.compare(null_tracker_str) != 0) {
    ROS_WARN("The Null Tracker must be active before taking off");
    return false;
  }

  // Read takeoff height
  ros::param::param<double>("takeoff_height", takeoff_height_, 0.2);

  if (takeoff_height_ > 3.0) {
    ROS_ERROR("Takeoff Height is Dangerously High");
    return false;
  }

  ROS_INFO("Initiating launch sequence...");
  quadrotor_msgs::LineTrackerGoal goal;
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = pos_(2) + takeoff_height_;
  pub_goal_line_tracker_distance_.publish(goal);

  return this->transition(line_tracker_distance);
}

bool MAVManager::setHome() {
  bool flag = this->have_recent_odom();

  if (flag) {
    home_ = pos_;
    home_set_ = true;

    home_yaw_ = yaw_;
    home_yaw_set_ = true;
  } else
    ROS_WARN(
        "Cannot set home unless current pose is set or an argument is "
        "provided.");

  return flag;
}

bool MAVManager::goHome() {
  Vec4 goal(pos_(0), pos_(1), pos_(2), yaw_);
  if (home_set_) {
    goal(0) = home_(0);
    goal(1) = home_(1);
    goal(2) = home_(2);
  }

  if (home_yaw_set_) goal(3) = home_yaw_;

  if (home_set_ || home_yaw_set_) {
    return this->goTo(goal);
  } else {
    ROS_WARN("Home not set. Cannot go home.");
    return false;
  }
}

bool MAVManager::goTo(double x, double y, double z, double yaw, double v_des, double a_des) {

  quadrotor_msgs::LineTrackerGoal goal;
  goal.x   = x   + offsets_(0);
  goal.y   = y   + offsets_(1);
  goal.z   = z   + offsets_(2);
  goal.yaw = yaw + offsets_(3);
  goal.v_des = v_des;
  goal.a_des = a_des;

  pub_goal_min_jerk_.publish(goal);
  ROS_INFO("Attempting to go to {%2.2f, %2.2f, %2.2f, %2.2f}", goal.x, goal.y,
           goal.z, goal.yaw);

  return this->transition(line_tracker_min_jerk);
}
bool MAVManager::goTo(Vec4 xyz_yaw, Vec2 v_and_a_des) {
  return this->goTo(xyz_yaw(0), xyz_yaw(1), xyz_yaw(2), xyz_yaw(3),
                    v_and_a_des(0), v_and_a_des(1));
}
bool MAVManager::goTo(Vec3 xyz, double yaw, Vec2 v_and_a_des) {
  return this->goTo(xyz(0), xyz(1), xyz(2), yaw,
                    v_and_a_des(0), v_and_a_des(1));
}
bool MAVManager::goTo(Vec3 xyz, Vec2 v_and_a_des) {
  return this->goTo(xyz(0), xyz(1), xyz(2), yaw_,
                    v_and_a_des(0), v_and_a_des(1));
}
bool MAVManager::goToYaw(double yaw) {
  return this->goTo(pos_(0), pos_(1), pos_(2), yaw);
}

// World Velocity commands
bool MAVManager::setDesVelWorld(double x, double y, double z, double yaw) {

  quadrotor_msgs::FlatOutputs goal;
  goal.x = x;
  goal.y = y;
  goal.z = z;
  goal.yaw = yaw;
  pub_goal_velocity_.publish(goal);
  ROS_INFO("Desired World velocity: (%1.4f, %1.4f, %1.4f, %1.4f)",
      goal.x, goal.y, goal.z, goal.yaw);

  // Since this could be called quite often by output_data_cb,
  // only try to transition if it is not the active tracker.
  if (active_tracker_.compare(velocity_tracker_str) != 0)
    return this->transition(velocity_tracker_str);

  return true;
}
bool MAVManager::setDesVelWorld(Vec4 vel) {
  return this->setDesVelWorld(vel(0), vel(1), vel(2), vel(3));
}
bool MAVManager::setDesVelWorld(Vec3 xyz) {
  return this->setDesVelWorld(xyz(0), xyz(1), xyz(2), 0);
}
bool MAVManager::setDesVelWorld(Vec3 xyz, double yaw) {
  return this->setDesVelWorld(xyz(0), xyz(1), xyz(2), yaw);
}
bool MAVManager::setDesVelWorld(double x, double y, double z) {
  return this->setDesVelWorld(x, y, z, 0);
}

// Body Velocity commands
bool MAVManager::setDesVelBody(Vec3 xyz, double yaw) {
  Vec3 vel(odom_q_ * xyz);
  return this->setDesVelWorld(vel(0), vel(1), vel(2), yaw);
}
bool MAVManager::setDesVelBody(Vec4 xyz_yaw) {
  return this->setDesVelBody(
      Vec3(xyz_yaw(0), xyz_yaw(1), xyz_yaw(2)), xyz_yaw(3));
}
bool MAVManager::setDesVelBody(Vec3 xyz) {
  return this->setDesVelBody(xyz, 0);
}
bool MAVManager::setDesVelBody(double x, double y, double z) {
  return this->setDesVelBody(Vec3(x, y, z), 0);
}
bool MAVManager::setDesVelBody(double x, double y, double z, double yaw) {
  return this->setDesVelBody(Vec3(x, y, z), yaw);
}

bool MAVManager::setPositionCommand(const quadrotor_msgs::PositionCommand msg) {

  // quadrotor_msgs::PositionCommand goal;
  pub_position_command_.publish(msg);

  // Since this could be called quite often,
  // only try to transition if it is not the active tracker.
  if (active_tracker_.compare(pos_cmd_tracker_str) != 0)
    return this->transition(pos_cmd_tracker_str);

  return true;
}


bool MAVManager::motors(bool motors) {

  // Do nothing if we ask for motors to be turned on when they already are on
  if (motors && motors_ == true)
    return true;

  bool null_tkr = this->transition(null_tracker_str);

  // Make sure null_tracker is active before starting rotors. If turning motors
  // off, continue anyway.
  if (motors && !null_tkr) {
    ROS_WARN("Could not transition to null_tracker before starting rotors");
    return false;
  }

  // Enable/Disable motors
  if (motors)
    ROS_INFO("Motors starting...");
  else
    ROS_INFO("Motors stopping...");

  std_msgs::Bool motors_cmd;
  motors_cmd.data = motors;
  pub_motors_.publish(motors_cmd);

  // Publish so3_command ensure motors are or are not spinning
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.force.z = 0.01 * mass_ * kGravity_;
  so3_cmd.orientation.w = 1;
  so3_cmd.aux.use_external_yaw = true;
  so3_cmd.aux.current_yaw = 0;
  so3_cmd.aux.enable_motors = motors;
  pub_so3_command_.publish(so3_cmd);

  motors_ = motors;
  return true;
}

void MAVManager::output_data_cb(
    const quadrotor_msgs::OutputData::ConstPtr &msg) {

  last_output_data_t_ = msg->header.stamp;

  for (unsigned int i = 0; i < 8; i++)
    radio_channel_[i] = msg->radio_channel[i];

  serial_ = radio_channel_[4] > 0;

  if (useRadioForVelocity_) {
    // constants
    double scale = 255.0 / 2.0;
    double rc_max_v = 1.0;
    double rc_max_w = 15.0 * M_PI / 180.0;

    // scale radio
    Vec4 vel(0, 0, 0, 0);
    vel(0) = -((double)radio_channel_[0] - scale) / scale * rc_max_v;
    vel(1) = -((double)radio_channel_[1] - scale) / scale * rc_max_v;

    // Only consider z velocity if the FLT Mode switch is toggled
    if (radio_channel_[5] > 0)
      vel(2) = ((double)radio_channel_[2] - scale) / scale * rc_max_v;

    vel(3) = -((double)radio_channel_[3] - scale) / scale * rc_max_w;

    // Deadbands on velocity
    Vec4 db(0.1, 0.1, 0.15, 0.03);
    for (unsigned int i = 0; i < 4; i++) vel(i) = deadband(vel(i), db(i));

    this->setDesVelWorld(vel(0), vel(1), vel(2), vel(3));
  }

  this->heartbeat();
}

void MAVManager::imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
  last_imu_t_ = msg->header.stamp;
  this->heartbeat();
}

void MAVManager::heartbeat_cb(const std_msgs::Empty::ConstPtr &msg) {
  this->heartbeat();
}

// TODO: This should be done in a separate thread
void MAVManager::heartbeat() {
  ros::Time t = ros::Time::now();

  // Only need to do monitoring at 10 Hz
  if ((t - last_heartbeat_t_).toSec() < 0.1)
    return;
  else
    last_heartbeat_t_ = t;

  // Checking the last odom and imu/output_data messages
  if (motors_ && !this->have_recent_odom()) {
    ROS_WARN("No recent odometry!");
    this->eland();
  }

  // TODO: This isn't necessary if we are flying in a MoCap. Should we even monitor?
  // if (motors_ && !this->have_recent_imu() && !this->have_recent_output_data()) {
  //   ROS_WARN("No recent imu or output_data");
  //   this->eland();
  // }

  // TODO: Put safety monitoring in here
}

bool MAVManager::useRadioForVelocity(bool b) {

  useRadioForVelocity_ = b;

  if (b && !this->have_recent_output_data()) {
    ROS_WARN("useRadioForVelocity: Not a recent enough output_data update");
    useRadioForVelocity_ = false;
    return false;
  }

  if (!b)
    return this->hover();

  return true;
}

bool MAVManager::eland() {
  ROS_WARN("Emergency Land");
  bool flag = this->transition(null_tracker_str);

  // Publish so3_command
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.force.z = 0.9 * mass_ * kGravity_;
  so3_cmd.orientation.x = 0;
  so3_cmd.orientation.y = 0;
  so3_cmd.orientation.z = 0;
  so3_cmd.orientation.w = 1;
  so3_cmd.aux.use_external_yaw = true;
  so3_cmd.aux.current_yaw = 0;
  so3_cmd.aux.enable_motors = motors_;
  pub_so3_command_.publish(so3_cmd);

  return flag;
}

void MAVManager::estop() {
  // Publish the E-Stop command
  ROS_WARN("E-STOP");
  std_msgs::Empty estop_cmd;
  pub_estop_.publish(estop_cmd);

  // Disarm motors
  this->motors(false);
  this->transition(null_tracker_str);

  // Publish so3_command to ensure motors are stopped
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.orientation.x = 0;
  so3_cmd.orientation.y = 0;
  so3_cmd.orientation.z = 0;
  so3_cmd.orientation.w = 1;
  so3_cmd.aux.enable_motors = false;
  pub_so3_command_.publish(so3_cmd);
}

bool MAVManager::hover() {

  double a_des(0.8); //, yaw_a_des(0.1);

  double v_norm = vel_.norm();
  Vec3 dir = vel_ / v_norm;

  // Acceleration should be opposite the velocity component
  Vec3 acc = -dir * a_des;

  // acc(3) = - copysign(yaw_a_des, yaw_dot_);

  // vf = vo + a t   ->    t = (vf - vo) / a
  double t = v_norm / a_des;
  // double t_yaw = - yaw_dot_ / yaw_a_des;

  // xf = xo + vo * t + 1/2 * a * t^2
  Vec4 goal(
      pos_(0) + vel_(0)  * t     + 0.5 * acc(0)    * t     * t,
      pos_(1) + vel_(1)  * t     + 0.5 * acc(1)    * t     * t,
      pos_(2) + vel_(2)  * t     + 0.5 * acc(2)    * t     * t,
      yaw_);//    + yaw_dot_ * t_yaw + 0.5 * yaw_a_des * t_yaw * t_yaw);

  Vec2 v_and_a_des(std::sqrt(vel_.dot(vel_)), a_des);

  ROS_DEBUG("Coasting to hover...");
  return this->goTo(goal, v_and_a_des);
}

bool MAVManager::ehover() {
  quadrotor_msgs::LineTrackerGoal goal;
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = pos_(2);
  pub_goal_line_tracker_distance_.publish(goal);

  return this->transition(line_tracker_distance);
}

bool MAVManager::transition(const std::string &tracker_str) {
  // usleep(100000);
  trackers_manager::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd)) {
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }

  return false;
}

bool MAVManager::have_recent_odom() {
  return (ros::Time::now() - last_odom_t_).toSec() < 0.1;
}

bool MAVManager::have_recent_imu() {
  return (ros::Time::now() - last_imu_t_).toSec() < 0.1;
}

bool MAVManager::have_recent_output_data() {
  return (ros::Time::now() - last_output_data_t_).toSec() < 0.1;
}

double deadband(double val, double deadband) {
  if (val > deadband)
    return val - deadband;
  else if (val < -deadband)
    return val + deadband;
  else
    return 0.0;

  // // Could also do it this way
  // double v = val;
  // v = std::min(v,  deadband);
  // v = std::max(v, -deadband);
  // return = val - v;
}
