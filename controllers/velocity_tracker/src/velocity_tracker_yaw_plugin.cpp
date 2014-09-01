#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <tf/transform_datatypes.h>

class VelocityTrackerYaw : public controllers_manager::Controller
{
 public:
  VelocityTrackerYaw(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);
  void velocity_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

  ros::Subscriber sub_vel_cmd_;
  quadrotor_msgs::PositionCommand position_cmd_;
  bool odom_set_, active_;
  double cur_yaw_, cmd_yaw_;
  double dt_;
  ros::Time last_t_;
  double kv_[3];
};

VelocityTrackerYaw::VelocityTrackerYaw(void) :
    odom_set_(false),
    active_(false)
{
}

void VelocityTrackerYaw::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "velocity_tracker");

  sub_vel_cmd_ = priv_nh.subscribe("vel_cmd_with_yaw", 10, &VelocityTrackerYaw::velocity_cmd_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  position_cmd_.kv[0] = kv_[0], position_cmd_.kv[1] = kv_[1], position_cmd_.kv[2] = kv_[2];
}

bool VelocityTrackerYaw::Activate(void)
{
  if(odom_set_)
  {
    cmd_yaw_ = cur_yaw_;
    active_ = true;

    ROS_INFO("Activated Velocity Tracker");
    last_t_ = ros::Time::now();
  }

  return active_;
}

void VelocityTrackerYaw::Deactivate(void)
{
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr VelocityTrackerYaw::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  odom_set_ = true;

  dt_ = ros::Time::now().toSec() - last_t_.toSec();
  last_t_ = ros::Time::now();
  
  cmd_yaw_ += dt_ * position_cmd_.yaw_dot;
  
  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  position_cmd_.header.stamp = msg->header.stamp;
  position_cmd_.header.frame_id = msg->header.frame_id;
  position_cmd_.yaw = cmd_yaw_;

  return quadrotor_msgs::PositionCommand::Ptr(new quadrotor_msgs::PositionCommand(position_cmd_));
}

void VelocityTrackerYaw::velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(velocity_tracker, VelocityTrackerYaw, VelocityTrackerYaw, controllers_manager::Controller)
