#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <tracker_msgs/TrackerStatus.h>

class NullTracker : public trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;
};

void NullTracker::Initialize(const ros::NodeHandle &nh)
{
}

bool NullTracker::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  return true;
}

void NullTracker::Deactivate(void)
{
}

quadrotor_msgs::PositionCommand::ConstPtr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  return quadrotor_msgs::PositionCommand::Ptr();
}

uint8_t NullTracker::status() const
{
  return tracker_msgs::TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NullTracker, trackers_manager::Tracker);
