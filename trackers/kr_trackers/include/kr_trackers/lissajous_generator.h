#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/lissajous_adder.hpp"
#include "kr_tracker_msgs/action/lissajous_tracker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class LissajousGenerator
{
 public:
  LissajousGenerator();
  void setParams(const std::shared_ptr<const kr_tracker_msgs::action::LissajousTracker::Goal> &msg);
  void setParams(const std::shared_ptr<const kr_tracker_msgs::action::LissajousAdder::Goal> &msg, int num);
  void generatePath(nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &initial_pt, double dt);
  kr_mav_msgs::msg::PositionCommand::SharedPtr getPositionCmd();
  bool activate();
  void deactivate();
  bool isActive() const;
  bool goalIsSet() const;
  bool status() const;
  float timeRemaining() const;
  float timeElapsed() const;

 private:
  double ramp_time_, total_time_, ramp_s_, total_s_, const_time_, period_;
  double x_amp_, y_amp_, z_amp_, yaw_amp_;
  double x_num_periods_, y_num_periods_, z_num_periods_, yaw_num_periods_;
  double a7_, a6_, a5_, a4_;
  double num_cycles_;
  bool active_, goal_set_, goal_reached_;
  rclcpp::Time start_time_;
  rclcpp::Clock::SharedPtr clock_;
};
