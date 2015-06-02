#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/TrackerStatus.h>

class NullTracker : public trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();
};

void NullTracker::Initialize(const ros::NodeHandle &nh)
{
}

bool NullTracker::Activate(void)
{
  return true;
}

void NullTracker::Deactivate(void)
{
}

const quadrotor_msgs::PositionCommand::ConstPtr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  return quadrotor_msgs::PositionCommand::Ptr();
}

const quadrotor_msgs::TrackerStatus::Ptr NullTracker::status()
{
  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NullTracker, trackers_manager::Tracker);
