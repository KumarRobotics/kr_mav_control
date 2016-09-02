#include <iostream>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <initial_conditions.h>
#include <cmath>
#define DEBUG

class SmoothVelTracker : public trackers_manager::Tracker
{
 public:
  SmoothVelTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg);

  ros::Subscriber sub_goal_;
  bool goal_set_, goal_reached_;
  float target_speed_; 
  float ramp_dist_, total_dist_;
  float ramp_time_, total_time_;
  float start_yaw_, goal_yaw_;
  bool active_;
  InitialConditions ICs_;
  ros::Time start_time_;
  Eigen::Vector3f start_pos_, dir_;
  Eigen::Matrix<float,7,1> vel_coeffs_;
  float kx_[3], kv_[3];
};

SmoothVelTracker::SmoothVelTracker(void)
    : goal_set_(false), goal_reached_(true), active_(false)
{
}

void SmoothVelTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5f);
  nh.param("gains/pos/y", kx_[1], 2.5f);
  nh.param("gains/pos/z", kx_[2], 5.0f);
  nh.param("gains/vel/x", kv_[0], 2.2f);
  nh.param("gains/vel/y", kv_[1], 2.2f);
  nh.param("gains/vel/z", kv_[2], 4.0f);

  ros::NodeHandle priv_nh(nh, "smooth_vel_tracker");

  sub_goal_ = priv_nh.subscribe("goal", 10, &SmoothVelTracker::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
}

bool SmoothVelTracker::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_)
    active_ = true;
  return active_;
}

void SmoothVelTracker::Deactivate(void)
{
  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr SmoothVelTracker::update(
    const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!active_)
  {
    ICs_.set_from_odom(msg);
    return quadrotor_msgs::PositionCommand::Ptr();
  }

  if(goal_set_)
  {
    start_time_ = ros::Time::now();
    goal_set_ = false;
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // Get elapsed time
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - start_time_;
  float t = elapsed_time.toSec();
  float ts = t / ramp_time_; //scaled time
  float ts2 = ts*ts;
  float ts3 = ts2*ts;
  float ts4 = ts3*ts;
  float ts5 = ts4*ts;
  float ts6 = ts5*ts;
  float ts7 = ts6*ts;
  float ts8 = ts7*ts;

  // Test each case to generate trajectory
  Eigen::Vector3f pos, vel, acc, jrk;
  if(t > total_time_)
  {
    pos = start_pos_ + total_dist_*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->jerk.x = 0, cmd->jerk.y = 0, cmd->jerk.z = 0;
    cmd->yaw = goal_yaw_;
    goal_reached_ = true;
  }
  else if(t < ramp_time_)
  {
    float dist = ramp_time_*(vel_coeffs_(0)/2*ts2 + vel_coeffs_(1)/3*ts3 + vel_coeffs_(2)/4*ts4 
                           + vel_coeffs_(3)/5*ts5 + vel_coeffs_(4)/6*ts6 + vel_coeffs_(5)/7*ts7
                           + vel_coeffs_(6)/8*ts8);
    float speed = vel_coeffs_(0)*ts + vel_coeffs_(1)*ts2 + vel_coeffs_(2)*ts3 + vel_coeffs_(3)*ts4
                + vel_coeffs_(4)*ts5 + vel_coeffs_(5)*ts6 + vel_coeffs_(6)*ts7;
    float accel = (vel_coeffs_(0) + 2*vel_coeffs_(1)*ts + 3*vel_coeffs_(2)*ts2 + 4*vel_coeffs_(3)*ts3
                 + 5*vel_coeffs_(4)*ts4 + 6*vel_coeffs_(5)*ts5 + 7*vel_coeffs_(6)*ts6)/ramp_time_;
    float jerk = (2*vel_coeffs_(1) + 6*vel_coeffs_(2)*ts + 12*vel_coeffs_(3)*ts2
                + 20*vel_coeffs_(4)*ts3 + 30*vel_coeffs_(5)*ts4 + 42*vel_coeffs_(6)*ts5)/(ramp_time_*ramp_time_);
    pos = start_pos_ + dist*dir_;
    vel = speed*dir_;
    acc = accel*dir_;
    jrk = jerk*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
    cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);
    cmd->yaw = start_yaw_ + (goal_yaw_-start_yaw_)*t/total_time_;
  }
  else if(t < total_time_ - ramp_time_)
  {
    float dist = ramp_dist_ + target_speed_*(t - ramp_time_);
    pos = start_pos_ + dist*dir_;
    vel = target_speed_*dir_; 
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->jerk.x = 0, cmd->jerk.y = 0, cmd->jerk.z = 0;
    cmd->yaw = start_yaw_ + (goal_yaw_-start_yaw_)*t/total_time_;
  }
  else
  {
    float te = total_time_ - elapsed_time.toSec(); // time from end
    float tes = te/ramp_time_; // scaled time from end
    float tes2 = tes*tes; 
    float tes3 = tes2*tes; 
    float tes4 = tes3*tes;
    float tes5 = tes4*tes;
    float tes6 = tes5*tes;
    float tes7 = tes6*tes;
    float tes8 = tes7*tes;
    float dist_from_end = ramp_time_*(vel_coeffs_(0)/2*tes2 + vel_coeffs_(1)/3*tes3 + vel_coeffs_(2)/4*tes4 
                           + vel_coeffs_(3)/5*tes5 + vel_coeffs_(4)/6*tes6 + vel_coeffs_(5)/7*tes7
                           + vel_coeffs_(6)/8*tes8);
    float dist = total_dist_ - dist_from_end;
    float speed = vel_coeffs_(0)*tes + vel_coeffs_(1)*tes2 + vel_coeffs_(2)*tes3 + vel_coeffs_(3)*tes4
                + vel_coeffs_(4)*tes5 + vel_coeffs_(5)*tes6 + vel_coeffs_(6)*tes7;
    float accel = (vel_coeffs_(0) + 2*vel_coeffs_(1)*tes + 3*vel_coeffs_(2)*tes2 + 4*vel_coeffs_(3)*tes3
                 + 5*vel_coeffs_(4)*tes4 + 6*vel_coeffs_(5)*tes5 + 7*vel_coeffs_(6)*tes6)/ramp_time_;
    float jerk = (2*vel_coeffs_(1) + 6*vel_coeffs_(2)*tes + 12*vel_coeffs_(3)*tes2
          + 20*vel_coeffs_(4)*tes3 + 30*vel_coeffs_(5)*tes4 + 42*vel_coeffs_(6)*tes5)/(ramp_time_*ramp_time_);
    pos = start_pos_ + dist*dir_;
    vel = speed*dir_;
    acc = -accel*dir_;
    jrk = -jerk*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
    cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);
    cmd->yaw = start_yaw_ + (goal_yaw_-start_yaw_)*t/total_time_;
  }
  ICs_.set_from_cmd(cmd);
  return cmd;
}

void SmoothVelTracker::goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{

  // Make sure user specifies desired velocity
  if(msg->v_des > 0 && msg->a_des > 0)
  {
    // Set the start position
    Eigen::Vector3f start_pos = ICs_.pos();

    // Get target speed and acceleration
    target_speed_ = msg->v_des;
    float target_accel = msg->a_des;
    ramp_time_ = target_speed_/target_accel;

    // Find goal position
    Eigen::Vector3f goal_pos;
    goal_pos(0) = msg->x, goal_pos(1) = msg->y, goal_pos(2) = msg->z;
    if(msg->relative)
      goal_pos += start_pos; 

#ifdef DEBUG
    std::cout << "goal_pos: " << goal_pos.transpose() << std::endl;
    std::cout << "start_pos: " << start_pos.transpose() << std::endl;
#endif

    // Find distance and direction to goal
    const float total_dist = (goal_pos - start_pos).norm();
    Eigen::Vector3f dir = (goal_pos - start_pos).normalized();

    // Compute the coefficients
    Eigen::Matrix<float,7,7> Ainv;
    Ainv <<  0, 1, 0, 0, 0, 0, 0, 
             0, 0, 0, 0.5, 0, 0, 0,
             0, 0, 0, 0, 0, 1.0/6.0, 0,
             35, -20, -15, -5, 2.5, -2.0/3.0, -1.0/6.0,
             -84, 45, 39, 10, -7, 1, 0.5, 
             70, -36, -34, -7.5, 6.5, -2.0/3.0, -0.5, 
             -20, 10, 10, 2, -2, 1.0/6.0, 1.0/6.0;

    Eigen::Matrix<float,7,1> b;
    b << target_speed_, 0, 0, 0, 0, 0, 0;
    vel_coeffs_ = Ainv*b;

    // Compute the ramp distance
    ramp_dist_ = (vel_coeffs_(0)/2 + vel_coeffs_(1)/3 + vel_coeffs_(2)/4 + vel_coeffs_(3)/5
                + vel_coeffs_(4)/6 + vel_coeffs_(5)/7 + vel_coeffs_(6)/8)*ramp_time_;

#ifdef DEBUG
    std::cout << "ramp dist: " << ramp_dist_ << std::endl;
    std::cout << "total dist: " << total_dist << std::endl;
#endif

    // Check to make sure that twice the ramp distance is less than the entire distance
    if(2*ramp_dist_ < total_dist)
    {
      // Set the parameters for the trajectory
      start_pos_ = start_pos;
      total_dist_ = total_dist;
      dir_ = dir;

      // Compute total time of trajectory
      total_time_ = (total_dist_ - 2*ramp_dist_)/target_speed_ + 2*ramp_time_;

#ifdef DEBUG
      std::cout << "total time: " << total_time_ << std::endl;
#endif

      // Set the target yaw
      start_yaw_ = ICs_.yaw();
      goal_yaw_ = msg->yaw;

      // Set goal_set to true and goal_reached to false
      goal_set_ = true;
      goal_reached_ = false;
    }
    else
    {
      ROS_ERROR("increase the ramp acceleration");
    }
  }
  else
  {
    ROS_ERROR("v_des and a_des must be nonzero!");
  }
}


const quadrotor_msgs::TrackerStatus::Ptr SmoothVelTracker::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);

  msg->status = goal_reached_ ?
          static_cast<uint8_t>(quadrotor_msgs::TrackerStatus::SUCCEEDED) :
          static_cast<uint8_t>(quadrotor_msgs::TrackerStatus::ACTIVE);

  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SmoothVelTracker, trackers_manager::Tracker);
