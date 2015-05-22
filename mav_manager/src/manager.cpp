// mav_manager
#include <manager.h>

// Standard C++
#include <math.h>
#include <string>

// ROS Related
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

// quadrotor_control
#include <trackers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/SO3Command.h>


// Strings
static const std::string line_tracker_distance("std_trackers/LineTrackerDistance");
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerk");
static const std::string velocity_tracker_str("std_trackers/VelocityTracker");
static const std::string null_tracker_str("std_trackers/NullTracker");

// Helper functions
double deadband(double val, double deadband);

MAVManager::MAVManager():
  nh_(""),
  priv_nh_("~"),
  active_tracker_(""),
  offsets_(0,0,0,0), // TODO: The offsets need to be implemented throughout
  last_odom_t_(0),
  last_imu_t_(0),
  last_output_data_t_(0),
  kGravity_(9.81),
  useRadioForVelocity_(false)
{
  // Publishers
  pub_goal_line_tracker_distance_ = nh_.advertise<geometry_msgs::Vector3>("trackers_manager/line_tracker_distance/goal", 10);
  pub_goal_min_jerk_ = nh_.advertise<quadrotor_msgs::FlatOutputs>("trackers_manager/line_tracker_min_jerk/goal", 10);
  pub_goal_velocity_ = nh_.advertise<quadrotor_msgs::FlatOutputs>("trackers_manager/velocity_tracker/vel_cmd_with_yaw", 10);
  pub_motors_ = nh_.advertise<std_msgs::Bool>("motors", 10);
  pub_estop_ = nh_.advertise<std_msgs::Empty>("estop", 10);
  so3_command_pub_ = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  // position_command_pub_ = nh_ ...
  // pwm_command_pub_ = nh_ ...

  // Subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &MAVManager::odometry_cb, this);
  output_data_sub_ = nh_.subscribe("output_data", 10, &MAVManager::output_data_cb, this);
  imu_sub_ = nh_.subscribe("imu", 10, &MAVManager::imu_cb, this);
  heartbeat_sub_ = nh_.subscribe("/heartbeat", 10, &MAVManager::heartbeat_cb, this);

  // Services
  srv_transition_ = nh_.serviceClient<trackers_manager::Transition>("trackers_manager/transition");

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
  so3_command_pub_.publish(so3_cmd);

  motors_ = false;
}

void MAVManager::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  odom_q_ = Quat(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = msg->header.stamp;

  this->heartbeat();
}

bool MAVManager::takeoff()
{
  if (!this->have_recent_odom())
  {
    ROS_WARN("Cannot takeoff without odometry.");
    return false;
  }

  if (!this->setHome())
    return false;

  if (!motors_)
  {
    ROS_WARN("Cannot takeoff until motors are enabled.");
    return false;
  }

  // Only takeoff if currently under NULL_TRACKER
  if (active_tracker_.compare(null_tracker_str) != 0)
  {
    ROS_WARN("The Null Tracker must be active before taking off");
    return false;
  }

  ROS_INFO("Initiating launch sequence...");
  geometry_msgs::Point goal;
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = pos_(2) + 0.2;
  pub_goal_line_tracker_distance_.publish(goal);

  return this->transition(line_tracker_distance);
}

bool MAVManager::setHome()
{
  bool flag = this->have_recent_odom();

  if (flag)
  {
    home_ = pos_;
    home_set_ = true;

    home_yaw_ = yaw_;
    home_yaw_set_ = true;
  }
  else
    ROS_WARN("Cannot set home unless current pose is set or an argument is provided.");

  return flag;
}

bool MAVManager::goHome()
{
  Vec4 goal(pos_(0), pos_(1), pos_(2), yaw_);
  if (home_set_)
  {
    goal(0) = home_(0);
    goal(1) = home_(1);
    goal(2) = home_(2);
  }

  if (home_yaw_set_)
    goal(3) = home_yaw_;

  if (home_set_ || home_yaw_set_)
  {
    return this->goTo(goal);
  }
  else
  {
    ROS_WARN("Home not set. Cannot go home.");
    return false;
  }
}

bool MAVManager::goTo(Vec4 target) // (xyz(psi))
{
  quadrotor_msgs::FlatOutputs goal;
  goal.x   = target(0) + offsets_(0);
  goal.y   = target(1) + offsets_(1);
  goal.z   = target(2) + offsets_(2);
  goal.yaw = target(3) + offsets_(3);
  pub_goal_min_jerk_.publish(goal);
  ROS_INFO("Attempting to go to {%2.2f, %2.2f, %2.2f, %2.2f}", goal.x, goal.y, goal.z, goal.yaw);

  return this->transition(line_tracker_min_jerk);
}
bool MAVManager::goTo(Vec3 xyz)
{
  Vec4 goal(xyz(0), xyz(1), xyz(2), yaw_);
  return this->goTo(goal);
}
bool MAVManager::goTo(Vec3 xyz, double yaw)
{
  Vec4 goal(xyz(0), xyz(1), xyz(2), yaw);
  return this->goTo(goal);
}
bool MAVManager::goTo(double x, double y, double z)
{
  Vec3 goal(x,y,z);
  return this->goTo(goal);
}
bool MAVManager::goTo(double x, double y, double z, double yaw)
{
  Vec4 goal(x,y,z,yaw);
  return this->goTo(goal);
}
bool MAVManager::goToYaw(double yaw)
{
  Vec4 goal(pos_(0), pos_(1), pos_(2), yaw);
  return this->goTo(goal);
}

// World Velocity commands
bool MAVManager::setDesVelWorld(Vec4 vel)
{
  quadrotor_msgs::FlatOutputs goal;
  goal.x = vel(0);
  goal.y = vel(1);
  goal.z = vel(2);
  goal.yaw = vel(3);
  pub_goal_velocity_.publish(goal);
  ROS_INFO("Desired World velocity: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw);

  // Only try to transition if it is not the active tracker
  if (active_tracker_.compare(velocity_tracker_str) != 0)
    return this->transition(velocity_tracker_str);

  return true;
}
bool MAVManager::setDesVelWorld(Vec3 xyz)
{
  Vec4 goal(xyz(0), xyz(1), xyz(2), 0);
  return this->setDesVelWorld(goal);
}
bool MAVManager::setDesVelWorld(Vec3 xyz, double yaw)
{
  Vec4 goal(xyz(0), xyz(1), xyz(2), yaw);
  return this->setDesVelWorld(goal);
}
bool MAVManager::setDesVelWorld(double x, double y, double z)
{
  Vec4 goal(x,y,z,0);
  return this->setDesVelWorld(goal);
}
bool MAVManager::setDesVelWorld(double x, double y, double z, double yaw)
{
  Vec4 goal(x,y,z,yaw);
  return this->setDesVelWorld(goal);
}

// Body Velocity commands
bool MAVManager::setDesVelBody(Vec3 xyz, double yaw)
{
  Vec3 vel(odom_q_ * xyz);
  return this->setDesVelWorld(vel, yaw);
}
bool MAVManager::setDesVelBody(Vec4 vel)
{
  Vec3 v(vel(0), vel(1), vel(2));
  return this->setDesVelBody(v, vel(3));
}
bool MAVManager::setDesVelBody(Vec3 xyz)
{
  return this->setDesVelBody(xyz, 0);
}
bool MAVManager::setDesVelBody(double x, double y, double z)
{
  Vec3 vel(x,y,z);
  return this->setDesVelBody(vel, 0);
}
bool MAVManager::setDesVelBody(double x, double y, double z, double yaw)
{
  Vec3 vel(x,y,z);
  return this->setDesVelBody(vel, yaw);
}

void MAVManager::motors(bool flag)
{
  std_msgs::Bool motors_cmd;
  motors_cmd.data = flag;
  pub_motors_.publish(motors_cmd);

  motors_ = flag;

  // Enable/Disable motors
  if (flag)
    ROS_INFO("Motors Armed...");
  else
    ROS_INFO("Motors Disarmed...");
}

void MAVManager::output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  last_output_data_t_ = msg->header.stamp;

  for (unsigned int i = 0; i < 8; i++)
    radio_channel_[i] = msg->radio_channel[i];

  serial_ = radio_channel_[4] > 0;

  if (useRadioForVelocity_)
  {
    // constants
    double scale    = 255.0 / 2.0;
    double rc_max_v = 1.0;
    double rc_max_w = 15.0*M_PI/180.0;

    // scale radio
    Vec4 vel(0,0,0,0);
    vel(0) = -((double)radio_channel_[0] - scale) / scale * rc_max_v;
    vel(1) = -((double)radio_channel_[1] - scale) / scale * rc_max_v;

    // Only consider z velocity if the FLT Mode switch is toggled
    if(radio_channel_[5] > 0)
      vel(2) = ((double)radio_channel_[2] - scale) / scale * rc_max_v;

    vel(3) = -((double)radio_channel_[3] - scale) / scale * rc_max_w;

    // Deadbands on velocity
    Vec4 db(0.1, 0.1, 0.15, 0.03);
    for (unsigned int i = 0; i < 4; i++)
      vel(i) = deadband(vel(i), db(i));

    this->setDesVelWorld(vel);
  }

  this->heartbeat();
}

void MAVManager::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  last_imu_t_ = msg->header.stamp;
  this->heartbeat();
}

void MAVManager::heartbeat_cb(const std_msgs::Empty::ConstPtr &msg)
{
  this->heartbeat();
}

// TODO: This should be done in a separate thread
void MAVManager::heartbeat()
{
  ros::Time t = ros::Time::now();

  // Only need to do monitoring at 10 Hz
  if ((t - last_heartbeat_t_).toSec() < 0.1)
    return;
  else
    last_heartbeat_t_ = t;

  // Checking the last odom and imu/output_data messages
  if (motors_ && !this->have_recent_odom())
    this->eland();

  if (motors_ && !this->have_recent_imu() && !this->have_recent_output_data())
    this->eland();

  // TODO: Put safety monitoring in here
}

bool MAVManager::useRadioForVelocity(bool b)
{
  useRadioForVelocity_ = b;

  if (b && !this->have_recent_output_data())
  {
    ROS_WARN("useRadioForVelocity: Not a recent enough output_data update");
    useRadioForVelocity_ = false;
    return false;
  }

  if (!b)
    return this->hover();
}

bool MAVManager::eland()
{
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
  so3_command_pub_.publish(so3_cmd);

  return flag;
}

void MAVManager::estop()
{
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
  so3_command_pub_.publish(so3_cmd);
}

bool MAVManager::hover()
{
  // Assuming constant-deceleration (same as defaults in min_jerk)
  // TODO: Maybe set these as params
  double xy_acc(0.5), z_acc(0.5), yaw_acc(0.1);

  // Acceleration should be opposite the velocity component
  Vec4 acc;
  acc(0) = - copysign(xy_acc,  vel_(0));
  acc(1) = - copysign(xy_acc,  vel_(1));
  acc(2) = - copysign(z_acc,   vel_(2));
  acc(3) = - copysign(yaw_acc, yaw_dot_);

  // vf = vo + a t   ->    t = (vf - vo) / a
  double tx, ty, tz, t_yaw;
  tx = - vel_(0) / acc(0);
  ty = - vel_(1) / acc(1);
  tz = - vel_(2) / acc(2);
  t_yaw = - yaw_dot_ / acc(3);

  Vec4 goal(
      pos_(0) + 1/2 * acc(0) * tx * tx,
      pos_(1) + 1/2 * acc(1) * ty * ty,
      pos_(2) + 1/2 * acc(2) * tz * tz,
      yaw_    + 1/2 * acc(3) * t_yaw * t_yaw);

  ROS_DEBUG("Coasting to hover...");
  return this->goTo(goal);
}

bool MAVManager::ehover()
{
  geometry_msgs::Point goal;
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = pos_(2);
  pub_goal_line_tracker_distance_.publish(goal);

  return this->transition(line_tracker_distance);
}

bool MAVManager::transition(const std::string &tracker_str)
{
  // usleep(100000);
  trackers_manager::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd))
  {
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }

  return false;
}

bool MAVManager::have_recent_odom()
{
  return (ros::Time::now() - last_odom_t_).toSec() < 0.1;
}

bool MAVManager::have_recent_imu()
{
  return (ros::Time::now() - last_imu_t_).toSec() < 0.1;
}

bool MAVManager::have_recent_output_data()
{
  return (ros::Time::now() - last_output_data_t_).toSec() < 0.1;
}

double deadband(double val, double deadband)
{
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
