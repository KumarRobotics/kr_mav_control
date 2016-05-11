#ifndef STD_TRACKERS_INITIAL_CONDITIONS_H
#define STD_TRACKERS_INITIAL_CONDITIONS_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

class InitialConditions
{
 public:
  void set_from_last_cmd(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void set_from_odom(const nav_msgs::Odometry::ConstPtr &msg);
  Eigen::Vector3f pos() const { return pos_; }
  Eigen::Vector3f vel() const { return vel_; }
  Eigen::Vector3f acc() const { return acc_; }
  Eigen::Vector3f jrk() const { return jrk_; }
  float yaw() const { return yaw_; }
  float yaw_dot() const { return yaw_dot_; }
  ros::Time time() const { return time_; }
  void reset();

 private:
  Eigen::Vector3f pos_, vel_, acc_, jrk_;
  float yaw_, yaw_dot_;
  bool last_cmd_valid_;
  ros::Time time_;
};

#endif // STD_TRACKERS_INITIAL_CONDITIONS_H
