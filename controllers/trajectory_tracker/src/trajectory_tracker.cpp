#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <trajectory_tracker/GoalCommand.h>
#include <math.h>

class TrajectoryTracker : public controllers_manager::Controller
{
 public:
  TrajectoryTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void goal_callback(const trajectory_tracker::GoalCommand::ConstPtr &msg);

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_;
  bool active_;

  Eigen::Vector3f pos_, vel_;
  Eigen::Vector4f xdes_, vdes_, ades_; //  goal_
  float yaw_;
  double kx_[3], kv_[3];
};

TrajectoryTracker::TrajectoryTracker(void) :
    pos_set_(false),
    goal_set_(false),
    active_(false)
{
}

void TrajectoryTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "trajectory_tracker");

  sub_goal_ = priv_nh.subscribe("goal", 10, &TrajectoryTracker::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
}

bool TrajectoryTracker::Activate(void)
{
  // Only allow activation if we have a trajectory
  if(goal_set_ && pos_set_)
  {
    active_ = true;
    ROS_INFO("TrajectoryTracker activated");
  }
  else
    ROS_WARN("goal or pos not set");

  return active_;
}

void TrajectoryTracker::Deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr TrajectoryTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  
  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;
  
  pos_set_ = true;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = xdes_(3);
  cmd->yaw_dot = vdes_(3);
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  cmd->position.x = xdes_(0), cmd->position.y = xdes_(1), cmd->position.z = xdes_(2);
  cmd->velocity.x = vdes_(0), cmd->velocity.y = vdes_(1), cmd->velocity.z = vdes_(2);
  cmd->acceleration.x = ades_(0), cmd->acceleration.y = ades_(1), cmd->acceleration.z = ades_(2);
  return cmd;
}

void TrajectoryTracker::goal_callback(const trajectory_tracker::GoalCommand::ConstPtr &msg)
{
  xdes_(0) = msg->pos.x;
  xdes_(1) = msg->pos.y;
  xdes_(2) = msg->pos.z;
  xdes_(3) = msg->pos.yaw;

  vdes_(0) = msg->vel.x;
  vdes_(1) = msg->vel.y;
  vdes_(2) = msg->vel.z;
  vdes_(3) = msg->vel.yaw;

  ades_(0) = msg->acc.x;
  ades_(1) = msg->acc.y;
  ades_(2) = msg->acc.z;
  ades_(3) = msg->acc.yaw;

  goal_set_ = true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(trajectory_tracker, TrajectoryTracker, TrajectoryTracker, controllers_manager::Controller);
