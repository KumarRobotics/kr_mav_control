#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <quadrotor_msgs/PositionCommand.h>

class NullTracker : public controllers_manager::Controller
{
 public:
  NullTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  bool active_;
};

NullTracker::NullTracker(void) :
    active_(false)
{
}

void NullTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "null_tracker");
}

bool NullTracker::Activate(void)
{
  active_ = true;
  return active_;
}

void NullTracker::Deactivate(void)
{
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  return quadrotor_msgs::PositionCommand::Ptr();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NullTracker, controllers_manager::Controller);
