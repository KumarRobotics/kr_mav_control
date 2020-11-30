#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>

class NullTracker : public kr_trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;
};

void NullTracker::Initialize(const ros::NodeHandle &nh) {}

bool NullTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  return true;
}

void NullTracker::Deactivate(void) {}

kr_mav_msgs::PositionCommand::ConstPtr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  return kr_mav_msgs::PositionCommand::Ptr();
}

uint8_t NullTracker::status() const
{
  return kr_tracker_msgs::TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NullTracker, kr_trackers_manager::Tracker);
