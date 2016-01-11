#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/OutputData.h>
#include <tf/transform_datatypes.h>
#include <std_trackers/RadioTracker.h>
#include <Eigen/Geometry>

class RadioTrackerSimulator : public trackers_manager::Tracker, public std_trackers::RadioTracker
{
 public:
  RadioTrackerSimulator(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:

  quadrotor_msgs::PositionCommand position_cmd_;
  float cur_yaw_, cmd_yaw_;
  bool odom_set_, active_;
  ros::Time last_t_;
};

RadioTrackerSimulator::RadioTrackerSimulator(void) :
  RadioTracker::RadioTracker(),
  odom_set_(false),
  active_(false),
  last_t_(0.0)
{
}

void RadioTrackerSimulator::Initialize(const ros::NodeHandle &nh)
{
  RadioTracker::StartOutputDataSubscriber(nh);

  position_cmd_.kx[0] = 0.0, position_cmd_.kx[1] = 0.0, position_cmd_.kx[2] = 0.0;
  position_cmd_.kv[0] = 0.0, position_cmd_.kv[1] = 0.0, position_cmd_.kv[2] = 0.0;
}

bool RadioTrackerSimulator::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if (!radio.params_set)
  {
    ROS_WARN("Radio params not set. Cannot activate.");
    return false;
  }
  else if (!odom_set_)
  {
    ROS_WARN("Odometry not set. Cannot activate.");
    return false;
  }
  else if (
      (ros::Time::now() - radio.last_update_t).toSec() > 0.1 ||
      (ros::Time::now() - radio.last_update_t).toSec() < 0.0)
  {
    ROS_WARN("Last radio update is nonexistant or timing is bad.");
    return false;
  }
  else
  {
    active_ = true;
    last_t_ = ros::Time::now();
    cmd_yaw_ = cur_yaw_;
  }

  return active_;
}

void RadioTrackerSimulator::Deactivate(void)
{
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr RadioTrackerSimulator::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  position_cmd_.position = msg->pose.pose.position;
  position_cmd_.velocity = msg->twist.twist.linear;

  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);

  odom_set_ = true;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  // constant (for now)
  float rc_max_pitch = 50.0 * M_PI / 180.0; // With a f_max / mg = 2.4, max angle = 65 deg
  float rc_max_roll = rc_max_pitch;

  Eigen::Matrix3f Rdes;
  Rdes = Eigen::AngleAxisf(   cur_yaw_,                 Eigen::Vector3f::UnitZ())
       * Eigen::AngleAxisf(   radio.y() * rc_max_roll,  Eigen::Vector3f::UnitX())
       * Eigen::AngleAxisf( - radio.x() * rc_max_pitch, Eigen::Vector3f::UnitY());

  Eigen::Vector3f b3 = Rdes * Eigen::Vector3f::UnitZ();

  // constant (for now)
  float max_thrust_to_weight_ratio = 2.4;
  float max_thrust_to_mass = max_thrust_to_weight_ratio * 9.81;
  float cmd_thrust_to_mass = std::max(0.0, max_thrust_to_mass * (radio.z() + 1.0) / 2.0);

  Eigen::Vector3f acc = cmd_thrust_to_mass * b3 - (9.81 * Eigen::Vector3f::UnitZ());

  float rc_max_w = 180.0 * M_PI / 180.0;
  float yaw_dot = - radio.yaw() * rc_max_w;

  double dt = ros::Time::now().toSec() - last_t_.toSec();
  last_t_ = ros::Time::now();

  cmd_yaw_ = cur_yaw_ + dt * yaw_dot;

  position_cmd_.header.stamp = msg->header.stamp;
  position_cmd_.header.frame_id = msg->header.frame_id;

  position_cmd_.acceleration.x = acc[0];
  position_cmd_.acceleration.y = acc[1];
  position_cmd_.acceleration.z = std::max(-9.8f, acc[2]);
  position_cmd_.yaw_dot        = yaw_dot;
  position_cmd_.yaw            = cmd_yaw_;

  return quadrotor_msgs::PositionCommand::Ptr(new quadrotor_msgs::PositionCommand(position_cmd_));
}

const quadrotor_msgs::TrackerStatus::Ptr RadioTrackerSimulator::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RadioTrackerSimulator, trackers_manager::Tracker)
