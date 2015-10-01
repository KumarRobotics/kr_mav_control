// mav_manager
#include <mav_manager/manager.h>

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

// Strings
static const std::string line_tracker_distance("std_trackers/LineTrackerDistance");
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerk");
static const std::string velocity_tracker_str("std_trackers/VelocityTracker");
static const std::string null_tracker_str("std_trackers/NullTracker");

MAVManager::MAVManager()
    : nh_(""),
      priv_nh_("~"),
      active_tracker_(""),
      last_odom_t_(0.0),
      last_imu_t_(0.0),
      last_output_data_t_(0.0),
      last_heartbeat_t_(0.0),
      mass_(-1.0),
      kGravity_(9.81),
      odom_q_(1.0, 0.0, 0.0, 0.0),
      imu_q_(1.0, 0.0 ,0.0 ,0.0),
      max_attitude_angle_(45.0 / 180.0 * M_PI),
      need_imu_(false),
      need_output_data_(true),
      need_odom_(true),
      use_attitude_safety_catch_(true) {

  // Load Params
  if (!priv_nh_.getParam("need_imu", need_imu_))
    ROS_WARN("Couldn't find need_imu param");

  if (!priv_nh_.getParam("need_output_data", need_output_data_))
    ROS_WARN("Couldn't find need_output_data param");

  if (!priv_nh_.getParam("use_attitude_safety_catch", use_attitude_safety_catch_))
    ROS_WARN("Couldn't find use_attitude_safety_catch param");

  if (!priv_nh_.getParam("max_attitude_angle", max_attitude_angle_))
    ROS_WARN("Couldn't find max_attitude_angle param");

  // Publishers
  pub_goal_line_tracker_distance_ = nh_.advertise<quadrotor_msgs::LineTrackerGoal>("trackers_manager/line_tracker_distance/goal", 10);
  pub_goal_min_jerk_ = nh_.advertise<quadrotor_msgs::LineTrackerGoal>("trackers_manager/line_tracker_min_jerk/goal", 10);
  pub_goal_velocity_ = nh_.advertise<quadrotor_msgs::FlatOutputs>("trackers_manager/velocity_tracker/goal", 10);
  pub_motors_ = nh_.advertise<std_msgs::Bool>("motors", 10);
  pub_estop_ = nh_.advertise<std_msgs::Empty>("estop", 10);
  pub_so3_command_ = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  pub_position_command_ = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);
  // pwm_command_pub_ = nh_ ...

  // Subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &MAVManager::odometry_cb, this);
  heartbeat_sub_ = nh_.subscribe("/heartbeat", 10, &MAVManager::heartbeat_cb, this);
  tracker_status_sub_ = nh_.subscribe("trackers_manager/status", 10, &MAVManager::tracker_status_cb, this);

  // Conditional subscribers
  if (need_output_data_)
    output_data_sub_ = nh_.subscribe("quad_decode_msg/output_data", 10, &MAVManager::output_data_cb, this);

  if (need_imu_)
    imu_sub_ = nh_.subscribe("quad_decode_msg/imu", 10, &MAVManager::imu_cb, this);

  // Services
  srv_transition_ = nh_.serviceClient<trackers_manager::Transition>("trackers_manager/transition");

  double m;
  if (!nh_.getParam("mass", m))
    ROS_ERROR("Mass must be set as param.");
  else if (this->set_mass(m))
    ROS_INFO("MAVManager using mass = %2.2f.", mass_);
  else
    ROS_ERROR("Mass failed to set. Perhaps mass <= 0?");

  // Wait until the service server is started
  while (!this->transition(null_tracker_str)) {
    ROS_WARN("Activation of NullTracker failed. Trying again shortly...");
    ros::Duration(0.25).sleep();
  }

  // Disable motors
  if (!this->set_motors(false))
    ROS_ERROR("Could not disable motors");
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

  last_odom_t_ = ros::Time::now();

  this->heartbeat();
}

bool MAVManager::takeoff() {
  if (!this->have_recent_odom()) {
    ROS_WARN("Cannot takeoff without odometry.");
    return false;
  }

  if (!this->setHome()) return false;

  if (!this->motors()) {
    ROS_WARN("Cannot takeoff until motors are enabled.");
    return false;
  }

  // Only takeoff if currently under NULL_TRACKER
  if (active_tracker_.compare(null_tracker_str) != 0) {
    ROS_WARN("The Null Tracker must be active before taking off");
    return false;
  }

  // Read takeoff height
  ros::param::param<double>("takeoff_height", takeoff_height_, 0.1);

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

bool MAVManager::set_mass(double m) {
  if (m>0)
  {
    mass_ = m;
    return true;
  }
  else
  {
    ROS_ERROR("Mass must be > 0");
    return false;
  }
}

bool MAVManager::setHome() {

  if (this->have_recent_odom()) {
    home_ = pos_;
    home_yaw_ = yaw_;
    home_set_ = true;

    return true;

  } else
    ROS_WARN("Cannot set home unless current pose is known.");

  return false;
}

bool MAVManager::goHome() {

  if (home_set_)
    return this->goTo(home_ + Vec3(0,0,0.15), home_yaw_);
  else {
    ROS_WARN("Home not set. Cannot go home.");
    return false;
  }
}

bool MAVManager::land() {

  ROS_INFO("Initiating landing sequence...");
  quadrotor_msgs::LineTrackerGoal goal;
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = home_(2) - 0.1;
  pub_goal_line_tracker_distance_.publish(goal);

  return this->transition(line_tracker_distance);
}

bool MAVManager::goTo(double x, double y, double z, double yaw, double v_des, double a_des) {

  quadrotor_msgs::LineTrackerGoal goal;
  goal.x   = x;
  goal.y   = y;
  goal.z   = z;
  goal.yaw = yaw;
  goal.v_des = v_des;
  goal.a_des = a_des;

  pub_goal_min_jerk_.publish(goal);
  ROS_INFO("Attempting to go to {%2.2f, %2.2f, %2.2f, %2.2f}", x, y, z, yaw);

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

  // Since this could be called quite often,
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

bool MAVManager::setPositionCommand(const quadrotor_msgs::PositionCommand &msg) {

  // TODO: Need to keep publishing a position command if there is no update.
  // Otherwise, no so3_command will be published.

  if (this->motors())
  {
    bool flag(true);

    // Since this could be called quite often,
    // only try to transition if it is not the active tracker.
    if (active_tracker_.compare(null_tracker_str) != 0)
      flag = this->transition(null_tracker_str);

    if (flag)
      pub_position_command_.publish(msg);

    return flag;
  }
  else
  {
    ROS_WARN("You are trying to set a position command, but the motors have not been enabled yet");
    return false;
  }
}

bool MAVManager::setSO3Command(const quadrotor_msgs::SO3Command &msg) {

  // Note: To enable motors, the motors method must be used
  if (!this->motors())
  {
    ROS_WARN("Cannot publish an SO3Command until motors have been enabled using the motors method");
    return false;
  }

  // Since this could be called quite often,
  // only try to transition if it is not the active tracker.
  bool flag(true);
  if (active_tracker_.compare(null_tracker_str) != 0)
    flag = this->transition(null_tracker_str);

  if (flag)
    pub_so3_command_.publish(msg);

  return flag;
}

bool MAVManager::useNullTracker() {

  if (active_tracker_.compare(null_tracker_str) != 0)
    return this->transition(null_tracker_str);

  return true;
}

bool MAVManager::set_motors(bool motors) {

  // Do nothing if we ask for motors to be turned on when they already are on
  if (motors && this->motors())
    return true;

  bool null_tkr = this->transition(null_tracker_str);

  // Make sure null_tracker is active before starting motors. If turning motors
  // off, continue anyway.
  if (motors && !null_tkr) {
    ROS_WARN("Could not transition to null_tracker before starting motors");
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

  // Publish a couple so3_commands to ensure motors are or are not spinning
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.force.z = FLT_MIN;
  so3_cmd.orientation.w = 1.0;
  so3_cmd.aux.enable_motors = motors;

  // Queue a few to make sure the signal gets through
  for (short i=0; i<10; i++)
    pub_so3_command_.publish(so3_cmd);

  motors_ = motors;
  return true;
}

void MAVManager::imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
  last_imu_t_ = ros::Time::now();

  imu_q_ = Quat(msg->orientation.w, msg->orientation.x,
      msg->orientation.y, msg->orientation.z);

  this->heartbeat();
}

void MAVManager::output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg) {
  last_output_data_t_ = ros::Time::now();
  last_imu_t_ = ros::Time::now();

  imu_q_ = Quat(msg->orientation.w, msg->orientation.x,
      msg->orientation.y, msg->orientation.z);

  voltage_ = msg->voltage;
  pressure_dheight_ = msg->pressure_dheight;
  pressure_height_ = msg->pressure_height;
  magnetic_field_[0] = msg->magnetic_field.x;
  magnetic_field_[1] = msg->magnetic_field.y;
  magnetic_field_[2] = msg->magnetic_field.z;
  for (unsigned char i=0; i<8; i++)
    radio_[i] = msg->radio_channel[i];

  this->heartbeat();
}

void MAVManager::tracker_status_cb(const quadrotor_msgs::TrackerStatus::ConstPtr &msg) {
  active_tracker_ = msg->tracker.c_str();
  tracker_status_ = msg->status;
}

void MAVManager::heartbeat_cb(const std_msgs::Empty::ConstPtr &msg) {
  this->heartbeat();
}

// TODO: This should be done in a separate thread
void MAVManager::heartbeat() {

  const float freq = 10; // Hz

  // Only need to do monitoring at the specified frequency
  ros::Time t = ros::Time::now();
  float dt = (t - last_heartbeat_t_).toSec();
  if (dt < 1/freq)
    return;
  else
    last_heartbeat_t_ = t;

  // Checking for odom
  if (this->motors() && need_odom_ && !this->have_recent_odom()) {
    ROS_WARN("No recent odometry!");
    this->eland();
  }

  // Checking for imu
  if (this->motors() && need_imu_ && !this->have_recent_imu()) {
    ROS_WARN("No recent imu!");
    this->eland();
  }

  if (use_attitude_safety_catch_)
  {
    // TODO: Currently this can be overridden if client is continually updating
    // position commands. Maybe put a timeout, but it could be dangerous? Maybe
    // require a call to hover before exiting a safety catch mode?

    // Convert quaterions to tf so we can compute Euler angles, etc.
    tf::Quaternion imu_q(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
    tf::Quaternion odom_q(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

    // Determine a geodesic angle from hover at the same yaw
    double yaw, pitch, roll;
    tf::Matrix3x3 R;

    // If we don't have IMU feedback, imu_q_ will be the identity rotation
    tf::Matrix3x3(imu_q).getEulerYPR(yaw, pitch, roll);
    R.setEulerYPR(0, pitch, roll);
    float imu_geodesic = std::fabs( std::acos(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1)));

    tf::Matrix3x3(odom_q).getEulerYPR(yaw, pitch, roll);
    R.setEulerYPR(0, pitch, roll);
    float odom_geodesic = std::fabs( std::acos(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1)));

    float geodesic = std::max(imu_geodesic, odom_geodesic);

    static float attitude_limit_timer = 0;
    if (geodesic > max_attitude_angle_)
      attitude_limit_timer += dt;
    else
      attitude_limit_timer = 0;

    if (attitude_limit_timer > 0.5)
    {
      // Reset the timer so we don't keep calling ehover
      attitude_limit_timer = 0;
      ROS_WARN("Attitude exceeded threshold of %2.2f deg! Geodesic = %2.2f deg. Entering emergency hover.",
          max_attitude_angle_ * 180.0 / M_PI, geodesic * 180.0 / M_PI);
      this->ehover();
    }
  }

  if (this->have_recent_output_data())
  {
    if (voltage_ < 10.0) // Note: Asctec firmware uses 9V
      ROS_WARN_THROTTLE(10, "Battery voltage = %2.2f V", voltage_);
  }

  // TODO: Incorporate bounding box constraints. Something along the lines of the following.
  // We may want to use a timer in case the boundary is crossed slowly.
  //
  //
  // bool flag = in_bounding_box(this->pos());
  // static bool lastflag = flag;
  // if (!flag && flag != lastflag)
  // {
  //   this->ehover();
  // }
  // lastflag = flag;

  // TODO: Also something along the following
  // // TF broadcaster to broadcast the quadrotor frame
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(pos_.x, pos_.y, pos_.z) );
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/simulator", "/quadrotor"));

}

bool MAVManager::eland() {

  // TODO: This should also check a height threshold or something along those
  // lines. For example, if the rotors are idle and the robot hasn't even
  // left the ground, we don't want them to spin up faster.
  if (this->motors())
  {
    ROS_WARN("Emergency Land");

    quadrotor_msgs::PositionCommand goal;
    goal.acceleration.z = - 0.45;
    goal.yaw = yaw_;

    return this->setPositionCommand(goal);
  }
  else
    return this->set_motors(false);
}

bool MAVManager::estop() {

  ROS_WARN("E-STOP");
  std_msgs::Empty estop_cmd;
  pub_estop_.publish(estop_cmd);

  // Disarm motors
  return this->set_motors(false);
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
    tracker_status_ = quadrotor_msgs::TrackerStatus::ACTIVE;
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
