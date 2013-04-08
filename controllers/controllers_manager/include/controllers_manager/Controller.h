#ifndef CONTROLLERS_MANAGER_CONTROLLER_H_
#define CONTROLLERS_MANAGER_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace controllers_manager
{

class Controller
{
 public:
  virtual ~Controller(void) = 0;

  virtual void Initialize(const ros::NodeHandle &nh) = 0;
  virtual bool Activate(void) = 0;
  virtual void Deactivate(void) = 0;

  virtual const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg) = 0;
};

Controller::~Controller(void)
{
}

}

#endif
