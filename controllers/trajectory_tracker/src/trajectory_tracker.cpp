#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <math.h>

class TrajectoryTracker : public controllers_manager::Controller
{
 public:
  TrajectoryTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  
  void goal_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  quadrotor_msgs::PositionCommand::Ptr cmd_;

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_;
  bool active_;
};

TrajectoryTracker::TrajectoryTracker(void) :
    pos_set_(false),
    goal_set_(false),
    active_(false)
{
}

void TrajectoryTracker::Initialize(const ros::NodeHandle &nh)
{
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
  pos_set_ = true;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();
  
  // Update the header
  cmd_->header.stamp = ros::Time::now();
  cmd_->header.frame_id = msg->header.frame_id;

  return cmd_;
}

void TrajectoryTracker::goal_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  cmd_.reset(new quadrotor_msgs::PositionCommand(*msg));
  goal_set_ = true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrajectoryTracker, controllers_manager::Controller);
