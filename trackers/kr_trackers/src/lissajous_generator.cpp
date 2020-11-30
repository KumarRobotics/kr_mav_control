#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/lissajous_generator.h>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

LissajousGenerator::LissajousGenerator()
{
  active_ = false;
  goal_reached_ = false;
  goal_set_ = false;
}

void LissajousGenerator::setParams(const kr_tracker_msgs::LissajousTrackerGoal::ConstPtr &msg)
{
  x_amp_ = msg->x_amp;
  y_amp_ = msg->y_amp;
  z_amp_ = msg->z_amp;
  yaw_amp_ = msg->yaw_amp;
  x_num_periods_ = msg->x_num_periods;
  y_num_periods_ = msg->y_num_periods;
  z_num_periods_ = msg->z_num_periods;
  yaw_num_periods_ = msg->yaw_num_periods;
  period_ = msg->period;
  num_cycles_ = msg->num_cycles;
  ramp_time_ = msg->ramp_time;

  // Set goal stuff
  total_s_ = period_ * num_cycles_;

  // Compute a3_ and a2_
  double tr = ramp_time_;
  double tr2 = tr * tr;
  double tr3 = tr2 * tr;
  double tr4 = tr3 * tr;
  double tr5 = tr4 * tr;
  double tr6 = tr5 * tr;
  double tr7 = tr6 * tr;
  double tr8 = tr7 * tr;

  a4_ = 35.0 / tr4;
  a5_ = -84.0 / tr5;
  a6_ = 70.0 / tr6;
  a7_ = -20.0 / tr7;

  // Compute ramp_s_, const_time_
  ramp_s_ = a7_ * tr8 / 8.0 + a6_ * tr7 / 7.0 + a5_ * tr6 / 6.0 + a4_ * tr5 / 5.0;
  const_time_ = total_s_ - 2.0 * ramp_s_;
  total_time_ = 2.0 * ramp_time_ + const_time_;

  // Set the start position and time
  goal_set_ = true;
  goal_reached_ = false;
}

void LissajousGenerator::setParams(const kr_tracker_msgs::LissajousAdderGoal::ConstPtr &msg, int num)
{
  x_amp_ = msg->x_amp[num];
  y_amp_ = msg->y_amp[num];
  z_amp_ = msg->z_amp[num];
  yaw_amp_ = msg->yaw_amp[num];
  x_num_periods_ = msg->x_num_periods[num];
  y_num_periods_ = msg->y_num_periods[num];
  z_num_periods_ = msg->z_num_periods[num];
  yaw_num_periods_ = msg->yaw_num_periods[num];
  period_ = msg->period[num];
  num_cycles_ = msg->num_cycles[num];
  ramp_time_ = msg->ramp_time[num];

  // Set goal stuff
  total_s_ = period_ * num_cycles_;

  // Compute a3_ and a2_
  double tr = ramp_time_;
  double tr2 = tr * tr;
  double tr3 = tr2 * tr;
  double tr4 = tr3 * tr;
  double tr5 = tr4 * tr;
  double tr6 = tr5 * tr;
  double tr7 = tr6 * tr;
  double tr8 = tr7 * tr;

  a4_ = 35.0 / tr4;
  a5_ = -84.0 / tr5;
  a6_ = 70.0 / tr6;
  a7_ = -20.0 / tr7;

  // Compute ramp_s_, const_time_
  ramp_s_ = a7_ * tr8 / 8.0 + a6_ * tr7 / 7.0 + a5_ * tr6 / 6.0 + a4_ * tr5 / 5.0;
  const_time_ = total_s_ - 2.0 * ramp_s_;
  total_time_ = 2.0 * ramp_time_ + const_time_;

  // Set the start position and time
  goal_set_ = true;
  goal_reached_ = false;
}

const kr_mav_msgs::PositionCommand::Ptr LissajousGenerator::getPositionCmd(void)
{
  if(!active_)
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  // Set gains
  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);

  // Get elapsed time
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - start_time_;
  double t = elapsed_time.toSec();
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  double t6 = t5 * t;
  double t7 = t6 * t;
  double t8 = t7 * t;
  double s, sdot, sddot, sdddot;

  Eigen::Vector3f pos, vel, acc, jrk;
  double yaw, yaw_dot;
  if(t > total_time_)
  {
    pos = Eigen::Vector3f::Zero();
    yaw = 0;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->jerk.x = 0, cmd->jerk.y = 0, cmd->jerk.z = 0;
    cmd->yaw = yaw;
    cmd->yaw_dot = 0;
    goal_set_ = false;
    goal_reached_ = true;
  }
  else
  {
    if(t < ramp_time_)
    {
      s = a7_ * t8 / 8.0 + a6_ * t7 / 7.0 + a5_ * t6 / 6.0 + a4_ * t5 / 5.0;
      sdot = a7_ * t7 + a6_ * t6 + a5_ * t5 + a4_ * t4;
      sddot = 7.0 * a7_ * t6 + 6.0 * a6_ * t5 + 5.0 * a5_ * t4 + 4.0 * a4_ * t3;
      sdddot = 42.0 * a7_ * t5 + 30.0 * a6_ * t4 + 20.0 * a5_ * t3 + 12.0 * a4_ * t2;
    }
    else if(t < total_time_ - ramp_time_)
    {
      s = ramp_s_ + t - ramp_time_;
      sdot = 1;
      sddot = 0;
      sdddot = 0;
    }
    else
    {
      double te = total_time_ - t;
      double te2 = te * te;
      double te3 = te2 * te;
      double te4 = te3 * te;
      double te5 = te4 * te;
      double te6 = te5 * te;
      double te7 = te6 * te;
      double te8 = te7 * te;

      s = 2.0 * ramp_s_ + const_time_ - a7_ * te8 / 8.0 - a6_ * te7 / 7.0 - a5_ * te6 / 6.0 - a4_ * te5 / 5.0;
      sdot = a7_ * te7 + a6_ * te6 + a5_ * te5 + a4_ * te4;
      sddot = -7.0 * a7_ * te6 - 6.0 * a6_ * te5 - 5.0 * a5_ * te4 - 4.0 * a4_ * te3;
      sdddot = 42.0 * a7_ * te5 + 30.0 * a6_ * te4 + 20.0 * a5_ * te3 + 12.0 * a4_ * te2;
    }
    double T = period_;
    double T2 = T * T;
    double T3 = T2 * T;
    pos(0) = x_amp_ * (1 - std::cos(2 * M_PI * x_num_periods_ * s / T));
    pos(1) = y_amp_ * std::sin(2 * M_PI * y_num_periods_ * s / T);
    pos(2) = z_amp_ * std::sin(2 * M_PI * z_num_periods_ * s / T);
    vel(0) = x_amp_ * 2 * M_PI * x_num_periods_ * std::sin(2 * M_PI * x_num_periods_ * s / T) * sdot / T;
    vel(1) = y_amp_ * 2 * M_PI * y_num_periods_ * std::cos(2 * M_PI * y_num_periods_ * s / T) * sdot / T;
    vel(2) = z_amp_ * 2 * M_PI * z_num_periods_ * std::cos(2 * M_PI * z_num_periods_ * s / T) * sdot / T;
    acc(0) = x_amp_ * (4 * M_PI * M_PI * x_num_periods_ * x_num_periods_ * std::cos(2 * M_PI * x_num_periods_ * s / T) *
                           sdot * sdot / T2 +
                       2 * M_PI * x_num_periods_ * std::sin(2 * M_PI * x_num_periods_ * s / T) * sddot / T);
    acc(1) = y_amp_ * (-4 * M_PI * M_PI * y_num_periods_ * y_num_periods_ *
                           std::sin(2 * M_PI * y_num_periods_ * s / T) * sdot * sdot / T2 +
                       2 * M_PI * y_num_periods_ * std::cos(2 * M_PI * y_num_periods_ * s / T) * sddot / T);
    acc(2) = z_amp_ * (-4 * M_PI * M_PI * z_num_periods_ * z_num_periods_ *
                           std::sin(2 * M_PI * z_num_periods_ * s / T) * sdot * sdot / T2 +
                       2 * M_PI * z_num_periods_ * std::cos(2 * M_PI * z_num_periods_ * s / T) * sddot / T);
    jrk(0) = x_amp_ * (-8 * M_PI * M_PI * M_PI * x_num_periods_ * x_num_periods_ * x_num_periods_ *
                           std::sin(2 * M_PI * x_num_periods_ * s / T) * sdot * sdot * sdot / T3 +
                       4 * M_PI * M_PI * x_num_periods_ * x_num_periods_ * std::cos(2 * M_PI * x_num_periods_ * s / T) *
                           sdot * sddot / T2 +
                       2 * M_PI * x_num_periods_ * std::sin(2 * M_PI * x_num_periods_ * s / T) * sdddot / T);
    jrk(1) = y_amp_ * (-8 * M_PI * M_PI * M_PI * y_num_periods_ * y_num_periods_ * y_num_periods_ *
                           std::cos(2 * M_PI * y_num_periods_ * s / T) * sdot * sdot * sdot / T3 -
                       4 * M_PI * M_PI * y_num_periods_ * y_num_periods_ * std::sin(2 * M_PI * y_num_periods_ * s / T) *
                           sdot * sddot / T2 +
                       2 * M_PI * y_num_periods_ * std::cos(2 * M_PI * y_num_periods_ * s / T) * sdddot / T);
    jrk(2) = z_amp_ * (-8 * M_PI * M_PI * M_PI * z_num_periods_ * z_num_periods_ * z_num_periods_ *
                           std::cos(2 * M_PI * z_num_periods_ * s / T) * sdot * sdot * sdot / T3 -
                       4 * M_PI * M_PI * z_num_periods_ * z_num_periods_ * std::sin(2 * M_PI * z_num_periods_ * s / T) *
                           sdot * sddot / T2 +
                       2 * M_PI * z_num_periods_ * std::cos(2 * M_PI * z_num_periods_ * s / T) * sdddot / T);
    yaw = yaw_amp_ * (1 - std::cos(2 * M_PI * yaw_num_periods_ * s / T));
    yaw_dot = yaw_amp_ * 2 * M_PI * yaw_num_periods_ * std::sin(2 * M_PI * yaw_num_periods_ * s / T) * sdot / T;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
    cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);
    cmd->yaw = yaw;
    cmd->yaw_dot = yaw_dot;
  }
  return cmd;
}

void LissajousGenerator::generatePath(nav_msgs::Path &path, geometry_msgs::Point &initial_pt, double dt)
{
  if(goal_set_)
  {
    double s = 0.0;
    double T = period_;

    while(s < period_)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = x_amp_ * (1 - std::cos(2 * M_PI * x_num_periods_ * s / T)) + initial_pt.x;
      ps.pose.position.y = y_amp_ * std::sin(2 * M_PI * y_num_periods_ * s / T) + initial_pt.y;
      ps.pose.position.z = z_amp_ * std::sin(2 * M_PI * z_num_periods_ * s / T) + initial_pt.z;

      path.poses.push_back(ps);
      s += dt;  // increment by 0.1s
    }
  }
}

bool LissajousGenerator::activate(void)
{
  if(goal_set_)
  {
    active_ = true;
    start_time_ = ros::Time::now();
  }
  return active_;
}

void LissajousGenerator::deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

bool LissajousGenerator::isActive(void)
{
  return active_;
}

bool LissajousGenerator::goalIsSet(void)
{
  return goal_set_;
}

bool LissajousGenerator::status() const
{
  return goal_reached_ ? kr_tracker_msgs::TrackerStatus::SUCCEEDED : kr_tracker_msgs::TrackerStatus::ACTIVE;
}

float LissajousGenerator::timeRemaining(void)
{
  ros::Time t_now = ros::Time::now();
  float time_elapsed = (t_now - start_time_).toSec();
  return total_time_ - time_elapsed;
}

float LissajousGenerator::timeElapsed(void)
{
  ros::Time t_now = ros::Time::now();
  return (t_now - start_time_).toSec();
}
