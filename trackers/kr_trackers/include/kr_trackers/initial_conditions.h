#ifndef STD_TRACKERS_INITIAL_CONDITIONS_H
#define STD_TRACKERS_INITIAL_CONDITIONS_H

#include <kr_mav_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Core>

class InitialConditions
{
 public:
  InitialConditions();

  void set_from_cmd(const kr_mav_msgs::msg::PositionCommand::SharedPtr &msg);
  void set_from_odom(const nav_msgs::msg::Odometry::SharedPtr &msg);
  Eigen::Vector3f pos() const { return pos_; }
  Eigen::Vector3f vel() const { return vel_; }
  Eigen::Vector3f acc() const { return acc_; }
  Eigen::Vector3f jrk() const { return jrk_; }
  float yaw() const { return yaw_; }
  float yaw_dot() const { return yaw_dot_; }
  void reset();

 private:
  Eigen::Vector3f pos_, vel_, acc_, jrk_;
  float yaw_, yaw_dot_;
  bool cmd_valid_;
};

#endif  // STD_TRACKERS_INITIAL_CONDITIONS_H
