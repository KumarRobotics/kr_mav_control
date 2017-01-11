#ifndef STD_TRACKERS_FLAT_STATE_H
#define STD_TRACKERS_FLAT_STATE_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

class FlatState
{
 public:
  FlatState();
   
  void set_from_cmd(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void set_from_odom(const nav_msgs::Odometry::ConstPtr &msg);
  Eigen::Vector3f pos() const { return pos_; }
  Eigen::Vector3f vel() const { return vel_; }
  Eigen::Vector3f acc() const { return acc_; }
  Eigen::Vector3f jrk() const { return jrk_; }
  Eigen::Vector3f snp() const { return snp_; }
  float yaw() const { return yaw_; }
  float yaw_dot() const { return yaw_dot_; }
  float yaw_ddot() const { return yaw_ddot_; }
  void reset();

 private:
  Eigen::Vector3f pos_, vel_, acc_, jrk_, snp_;
  float yaw_, yaw_dot_, yaw_ddot_;
  bool cmd_valid_;
};

#endif // STD_TRACKERS_FLAT_STATE_H
