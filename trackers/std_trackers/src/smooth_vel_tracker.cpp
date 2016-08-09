#include <iostream>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <initial_conditions.h>

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
  double target_speed_; 
  double ramp_dist_, total_dist_;
  double ramp_time_, total_time_;
  bool active_;
  InitialConditions ICs_;
  ros::Time start_time_, test_start_time_;
  Eigen::Vector3f start_pos_, dir_;
  Eigen::Matrix<double,5,1> vel_coeffs_;
  double kx_[3], kv_[3];
};

SmoothVelTracker::SmoothVelTracker(void)
    : goal_set_(false), goal_reached_(true), active_(false)
{
  test_start_time_ = ros::Time::now();
}

void SmoothVelTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  nh.param("ramp_time", ramp_time_, 1.0);

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

  if(goal_reached_)
    ICs_.set_from_odom(msg);

  if(goal_set_)
  {
    start_time_ = ros::Time::now();
    goal_set_ = false;
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // Get elapsed time
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - start_time_;
  double t = elapsed_time.toSec()/ramp_time_;
  double t2 = t*t, t3 = t*t*t, t4 = t*t*t*t, t5 = t*t*t*t*t;

  // Test each case to generate trajectory
  Eigen::Vector3f pos, vel, acc;
  if(elapsed_time.toSec() > total_time_)
  {
    pos = start_pos_ + total_dist_*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    goal_reached_ = true;
  }
  else if(elapsed_time.toSec() < ramp_time_)
  {
    double dist = vel_coeffs_(4)*t5/5.0 + vel_coeffs_(3)*t4/4.0 + vel_coeffs_(2)*t3/3.0 + vel_coeffs_(1)*t2/2.0 + vel_coeffs_(0)*t;
    double speed = vel_coeffs_(4)*t4 + vel_coeffs_(3)*t3 + vel_coeffs_(2)*t2 + vel_coeffs_(1)*t + vel_coeffs_(0);
    double accel = 4*vel_coeffs_(4)*t3 + 3*vel_coeffs_(3)*t2 + 2*vel_coeffs_(2)*t + vel_coeffs_(1);
    pos = start_pos_ + dist*dir_;
    vel = speed*dir_;
    acc = accel*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
  }
  else if(elapsed_time.toSec() < total_time_ - ramp_time_)
  {
    double dist = ramp_dist_ + target_speed_*(t - ramp_time_);
    pos = start_pos_ + dist*dir_;
    vel = target_speed_*dir_; 
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
  }
  else
  {
    double te = (total_time_ - elapsed_time.toSec())/ramp_time_;
    double te2 = te*te, te3 = te*te*te, te4 = te*te*te*te, te5 = te*te*te*te*te;
    double dist_from_end = vel_coeffs_(4)*te5/5.0 + vel_coeffs_(3)*te4/4.0 + vel_coeffs_(2)*te3/3.0 + vel_coeffs_(1)*te2/2.0 + vel_coeffs_(0)*te;
    double dist = total_dist_ - dist_from_end;
    double speed = vel_coeffs_(4)*te4 + vel_coeffs_(3)*te3 + vel_coeffs_(2)*te2 + vel_coeffs_(1)*te + vel_coeffs_(0);
    double accel = 4*vel_coeffs_(4)*te3 + 3*vel_coeffs_(3)*te2 + 2*vel_coeffs_(2)*te + vel_coeffs_(1);
    pos = start_pos_ + dist*dir_;
    vel = speed*dir_;
    acc = -accel*dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
  }
  return cmd;
}

void SmoothVelTracker::goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{
  // Set the start position
  start_pos_ = ICs_.pos();

  // Make sure user specifies desired velocity
  if(msg->v_des > 0)
  {
    target_speed_ = msg->v_des;

    // Find goal position
    Eigen::Vector3f goal_pos;
    goal_pos(0) = msg->x, goal_pos(1) = msg->y, goal_pos(2) = msg->z;
    if(msg->relative)
      goal_pos += start_pos_; 

    // Find distance and direction to goal
    total_dist_ = (goal_pos - start_pos_).norm();
    dir_ = (goal_pos - start_pos_).normalized();

    // Compute the coefficients
    Eigen::Matrix<double,5,5> Ainv;
    Ainv <<  1,  0,  0,  0,   0, 
             0,  0,  1,  0,   0,
             0,  0,  0,  0, 0.5,
            -4,  4, -3, -1,  -1,
             3, -3,  2,  1, 0.5;
    Eigen::Matrix<double,5,1> b;
    b << 0, target_speed_, 0, 0, 0;
    vel_coeffs_ = Ainv*b;

    // Compute the ramp distance
    ramp_dist_ = vel_coeffs_(4)/5.0 + vel_coeffs_(3)/4.0 + vel_coeffs_(2)/3.0 + vel_coeffs_(1)/2.0 + vel_coeffs_(0);

    // Check to make sure that twice the ramp distance is less than the entire distance
    if(2*ramp_dist_ < total_dist_)
    {
      // Compute total time of trajectory
      total_time_ = (total_dist_ - 2*ramp_dist_)/target_speed_ + 2*ramp_time_;

      // Set goal_set to true and goal_reached to false
      goal_set_ = true;
      goal_reached_ = false;
    }
    else
    {
      ROS_ERROR("ramp time is too high");
    }
  }
  else
  {
    ROS_ERROR("v_des must be set to the target velocity!");
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
