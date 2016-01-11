#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/OutputData.h>
#include <tf/transform_datatypes.h>
#include <std_trackers/RadioTracker.h>


class RadioTrackerVelocity : public trackers_manager::Tracker, public std_trackers::RadioTracker
{
 public:
  RadioTrackerVelocity(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:

  quadrotor_msgs::PositionCommand position_cmd_;
  bool odom_set_, active_, use_position_gains_;
  double cur_yaw_, cmd_yaw_;
  ros::Time last_t_;
  double kx_[3], kv_[3], pos_[3];
};

RadioTrackerVelocity::RadioTrackerVelocity(void) :
  RadioTracker::RadioTracker(),
  odom_set_(false),
  active_(false),
  use_position_gains_(false)
{
}

void RadioTrackerVelocity::Initialize(const ros::NodeHandle &nh)
{
  RadioTracker::StartOutputDataSubscriber(nh);

  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  position_cmd_.kv[0] = kv_[0], position_cmd_.kv[1] = kv_[1], position_cmd_.kv[2] = kv_[2];
}

bool RadioTrackerVelocity::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
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
    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
  }

  return active_;
}

void RadioTrackerVelocity::Deactivate(void)
{
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr RadioTrackerVelocity::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;
  pos_[2] = msg->pose.pose.position.z;

  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  tf::Matrix3x3 R;
  R.setEulerYPR(cur_yaw_, 0.0, 0.0);

  odom_set_ = true;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  // TODO: Make this a param or toggle from the radio
  use_position_gains_ = true;

  // constants (for now)
  float rc_max_v = 1.0;
  float rc_max_w = 90.0 * M_PI / 180.0;

  std::array<float, 4> vel;
  vel[0] = - radio.x() * rc_max_v;
  vel[1] = - radio.y() * rc_max_v;
  vel[2] = (radio.channel(5) > 0.0) ? radio.z() * rc_max_v : 0.0;
  vel[3] = - radio.yaw() * rc_max_w;

  // ROS_INFO("vel[2]: %2.2f", vel[2]);

  tf::Vector3 vel_in_rotated_frame(vel[0], vel[1], vel[2]);
  tf::Vector3 vel_in_wrld_frame = R * vel_in_rotated_frame;

  position_cmd_.velocity.x = vel_in_wrld_frame[0];
  position_cmd_.velocity.y = vel_in_wrld_frame[1];
  position_cmd_.velocity.z = vel_in_wrld_frame[2];
  position_cmd_.yaw_dot    = vel[3];

  // ROS_INFO("velz: %2.2f", position_cmd_.velocity.z);

  double dt = ros::Time::now().toSec() - last_t_.toSec();
  last_t_ = ros::Time::now();

  cmd_yaw_ = cur_yaw_ + dt * position_cmd_.yaw_dot;

  if (use_position_gains_)
  {
    position_cmd_.kx[0] = kx_[0], position_cmd_.kx[1] = kx_[1], position_cmd_.kx[2] = kx_[2];

    position_cmd_.position.x = position_cmd_.position.x + dt * position_cmd_.velocity.x;
    position_cmd_.position.y = position_cmd_.position.y + dt * position_cmd_.velocity.y;
    position_cmd_.position.z = std::max(0.0, position_cmd_.position.z + dt * position_cmd_.velocity.z);
  }
  else
  {
    position_cmd_.kx[0] = 0, position_cmd_.kx[1] = 0, position_cmd_.kx[2] = 0;

    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
  }

  position_cmd_.header.stamp = msg->header.stamp;
  position_cmd_.header.frame_id = msg->header.frame_id;
  position_cmd_.yaw = cmd_yaw_;

  return quadrotor_msgs::PositionCommand::Ptr(new quadrotor_msgs::PositionCommand(position_cmd_));
}

const quadrotor_msgs::TrackerStatus::Ptr RadioTrackerVelocity::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RadioTrackerVelocity, trackers_manager::Tracker)
