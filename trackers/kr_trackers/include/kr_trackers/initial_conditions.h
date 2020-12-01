#ifndef STD_TRACKERS_INITIAL_CONDITIONS_H
#define STD_TRACKERS_INITIAL_CONDITIONS_H

#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Core>

class InitialConditions
{
 public:
  InitialConditions();

  void set_from_cmd(const kr_mav_msgs::PositionCommand::ConstPtr &msg);
  void set_from_odom(const nav_msgs::Odometry::ConstPtr &msg);
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
