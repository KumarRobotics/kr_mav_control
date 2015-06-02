#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/TrackerStatus.h>

class PositionCommandTracker : public trackers_manager::Tracker
{
 public:
  PositionCommandTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::PositionCommand::ConstPtr msg);

  ros::Subscriber sub_goal_;
  bool pos_set_, goal_set_;
  bool active_;

  quadrotor_msgs::PositionCommand::ConstPtr cmd_;
};

PositionCommandTracker::PositionCommandTracker(void) :
    pos_set_(false),
    goal_set_(false),
    active_(false)
{
}

void PositionCommandTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "position_command_tracker");

  sub_goal_ = priv_nh.subscribe("goal", 10, &PositionCommandTracker::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
}

bool PositionCommandTracker::Activate(void)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    active_ = true;
  }
  return active_;
}

void PositionCommandTracker::Deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr PositionCommandTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;

  if(!active_ || !goal_set_)
    return quadrotor_msgs::PositionCommand::Ptr();
  else
    return cmd_;
}

void PositionCommandTracker::goal_callback(const quadrotor_msgs::PositionCommand::ConstPtr msg)
{
  cmd_ = msg;
  goal_set_ = true;
}

const quadrotor_msgs::TrackerStatus::Ptr PositionCommandTracker::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::ACTIVE;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PositionCommandTracker, trackers_manager::Tracker);
