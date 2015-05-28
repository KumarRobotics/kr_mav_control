#ifndef TRACKERS_MANAGER_TRACKER_H_
#define TRACKERS_MANAGER_TRACKER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrackerStatus.h>

namespace trackers_manager
{
class Tracker
{
 public:
  virtual ~Tracker(void) {}

  virtual void Initialize(const ros::NodeHandle &nh) = 0;
  virtual bool Activate(void) = 0;
  virtual void Deactivate(void) = 0;

  virtual const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg) = 0;
  virtual const quadrotor_msgs::TrackerStatus::Ptr status() = 0;
};
}

#endif
