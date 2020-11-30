#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <memory>

class SmoothVelTracker : public kr_trackers_manager::Tracker
{
 public:
  SmoothVelTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

 private:
  void goal_callback();
  void preempt_callback();

  using ServerType = actionlib::SimpleActionServer<kr_tracker_msgs::LineTrackerAction>;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::unique_ptr<ServerType> tracker_server_;

  bool goal_set_, goal_reached_;
  float target_speed_;
  float ramp_dist_, total_dist_;
  float ramp_time_, total_time_;
  float start_yaw_, goal_yaw_;
  float yaw_dot_max_;
  bool active_;
  InitialConditions ICs_;
  ros::Time start_time_;
  Eigen::Vector3f start_pos_, dir_;
  Eigen::Matrix<float, 7, 1> vel_coeffs_;
  float current_traj_length_;
  Eigen::Vector3f prev_pos_;
};

SmoothVelTracker::SmoothVelTracker(void) : goal_set_(false), goal_reached_(true), active_(false) {}

void SmoothVelTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "smooth_vel_tracker");

  // Set up the action server.
  tracker_server_ = std::unique_ptr<ServerType>(new ServerType(priv_nh, "SmoothVelTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&SmoothVelTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&SmoothVelTracker::preempt_callback, this));

  tracker_server_->start();
}

bool SmoothVelTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_)
    active_ = true;
  return active_;
}

void SmoothVelTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("SmoothVelTracker::Deactivate: deactivated tracker while "
             "still tracking the goal.");
    tracker_server_->setAborted();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr SmoothVelTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!active_)
  {
    ICs_.set_from_odom(msg);
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  // Record distance between last position and current.
  const Eigen::Vector3f current_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  current_traj_length_ += (current_pos - prev_pos_).norm();

  prev_pos_ = current_pos;

  const ros::Time t_now = ros::Time::now();

  if(goal_set_)
  {
    start_time_ = t_now;
    goal_set_ = false;
  }

  // Set gains
  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  // Get elapsed time
  const ros::Duration elapsed_time = t_now - start_time_;
  const float t = elapsed_time.toSec();
  const float ts = t / ramp_time_;  // scaled time
  const float ts2 = ts * ts;
  const float ts3 = ts2 * ts;
  const float ts4 = ts3 * ts;
  const float ts5 = ts4 * ts;
  const float ts6 = ts5 * ts;
  const float ts7 = ts6 * ts;
  const float ts8 = ts7 * ts;

  // Test each case to generate trajectory
  Eigen::Vector3f pos, vel, acc, jrk;
  if(t > total_time_)
  {
    pos = start_pos_ + total_dist_ * dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->jerk.x = 0, cmd->jerk.y = 0, cmd->jerk.z = 0;
    cmd->yaw = goal_yaw_;
    cmd->yaw_dot = 0;
    goal_reached_ = true;
    if(tracker_server_->isActive())
    {
      // Send a success message and reset the length and duration variables.
      kr_tracker_msgs::LineTrackerResult result;
      result.duration = total_time_;
      result.length = current_traj_length_;
      result.x = pos(0);
      result.y = pos(1);
      result.z = pos(2);
      result.yaw = goal_yaw_;

      tracker_server_->setSucceeded(result);
      current_traj_length_ = 0.0;
    }
  }
  else if(t < ramp_time_)
  {
    float dist = ramp_time_ * (vel_coeffs_(0) / 2 * ts2 + vel_coeffs_(1) / 3 * ts3 + vel_coeffs_(2) / 4 * ts4 +
                               vel_coeffs_(3) / 5 * ts5 + vel_coeffs_(4) / 6 * ts6 + vel_coeffs_(5) / 7 * ts7 +
                               vel_coeffs_(6) / 8 * ts8);
    float speed = vel_coeffs_(0) * ts + vel_coeffs_(1) * ts2 + vel_coeffs_(2) * ts3 + vel_coeffs_(3) * ts4 +
                  vel_coeffs_(4) * ts5 + vel_coeffs_(5) * ts6 + vel_coeffs_(6) * ts7;
    float accel = (vel_coeffs_(0) + 2 * vel_coeffs_(1) * ts + 3 * vel_coeffs_(2) * ts2 + 4 * vel_coeffs_(3) * ts3 +
                   5 * vel_coeffs_(4) * ts4 + 6 * vel_coeffs_(5) * ts5 + 7 * vel_coeffs_(6) * ts6) /
                  ramp_time_;
    float jerk = (2 * vel_coeffs_(1) + 6 * vel_coeffs_(2) * ts + 12 * vel_coeffs_(3) * ts2 + 20 * vel_coeffs_(4) * ts3 +
                  30 * vel_coeffs_(5) * ts4 + 42 * vel_coeffs_(6) * ts5) /
                 (ramp_time_ * ramp_time_);
    pos = start_pos_ + dist * dir_;
    vel = speed * dir_;
    acc = accel * dir_;
    jrk = jerk * dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
    cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);
    cmd->yaw = start_yaw_ + yaw_dot_max_ / (2 * ramp_time_) * t * t;
    cmd->yaw_dot = yaw_dot_max_ / ramp_time_ * t;
  }
  else if(t < total_time_ - ramp_time_)
  {
    float dist = ramp_dist_ + target_speed_ * (t - ramp_time_);
    pos = start_pos_ + dist * dir_;
    vel = target_speed_ * dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->jerk.x = 0, cmd->jerk.y = 0, cmd->jerk.z = 0;
    cmd->yaw = start_yaw_ + yaw_dot_max_ * (t - ramp_time_ / 2);
    cmd->yaw_dot = yaw_dot_max_;
  }
  else
  {
    const float te = total_time_ - elapsed_time.toSec();  // time from end
    const float tes = te / ramp_time_;                    // scaled time from end
    const float tes2 = tes * tes;
    const float tes3 = tes2 * tes;
    const float tes4 = tes3 * tes;
    const float tes5 = tes4 * tes;
    const float tes6 = tes5 * tes;
    const float tes7 = tes6 * tes;
    const float tes8 = tes7 * tes;
    const float dist_from_end =
        ramp_time_ *
        (vel_coeffs_(0) / 2 * tes2 + vel_coeffs_(1) / 3 * tes3 + vel_coeffs_(2) / 4 * tes4 + vel_coeffs_(3) / 5 * tes5 +
         vel_coeffs_(4) / 6 * tes6 + vel_coeffs_(5) / 7 * tes7 + vel_coeffs_(6) / 8 * tes8);
    const float dist = total_dist_ - dist_from_end;
    const float speed = vel_coeffs_(0) * tes + vel_coeffs_(1) * tes2 + vel_coeffs_(2) * tes3 + vel_coeffs_(3) * tes4 +
                        vel_coeffs_(4) * tes5 + vel_coeffs_(5) * tes6 + vel_coeffs_(6) * tes7;
    const float accel =
        (vel_coeffs_(0) + 2 * vel_coeffs_(1) * tes + 3 * vel_coeffs_(2) * tes2 + 4 * vel_coeffs_(3) * tes3 +
         5 * vel_coeffs_(4) * tes4 + 6 * vel_coeffs_(5) * tes5 + 7 * vel_coeffs_(6) * tes6) /
        ramp_time_;
    const float jerk = (2 * vel_coeffs_(1) + 6 * vel_coeffs_(2) * tes + 12 * vel_coeffs_(3) * tes2 +
                        20 * vel_coeffs_(4) * tes3 + 30 * vel_coeffs_(5) * tes4 + 42 * vel_coeffs_(6) * tes5) /
                       (ramp_time_ * ramp_time_);
    pos = start_pos_ + dist * dir_;
    vel = speed * dir_;
    acc = -accel * dir_;
    jrk = jerk * dir_;
    cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
    cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
    cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1), cmd->acceleration.z = acc(2);
    cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);
    cmd->yaw = goal_yaw_ - yaw_dot_max_ / (2 * ramp_time_) * (total_time_ - t) * (total_time_ - t);
    cmd->yaw_dot = yaw_dot_max_ - yaw_dot_max_ / ramp_time_ * (t - total_time_ + ramp_time_);
  }
  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    kr_tracker_msgs::LineTrackerFeedback feedback;
    Eigen::Vector3f goal = start_pos_ + total_dist_ * dir_;
    feedback.distance_from_goal = (current_pos - goal).norm();
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void SmoothVelTracker::preempt_callback()
{
  ROS_INFO("SmoothVelTracker goal preempted.");
  tracker_server_->setPreempted();

  // TODO: How much overshoot will this cause at high velocities?
  total_time_ = (ros::Time::now() - start_time_).toSec();
  total_dist_ = (ICs_.pos() - start_pos_).norm();
  dir_ = (ICs_.pos() - start_pos_).normalized();
  goal_set_ = false;
  goal_reached_ = true;
}

void SmoothVelTracker::goal_callback()
{
  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("SmoothVelTracker going to goal (%f, %f, %f, %f) preempted.", msg->x, msg->y, msg->z, msg->yaw);
    tracker_server_->setPreempted();
    return;
  }

  current_traj_length_ = 0.0;

  // Make sure user specifies desired velocity
  if(msg->v_des > 0 && msg->a_des > 0)
  {
    // Set the start position
    Eigen::Vector3f start_pos = ICs_.pos();

    // Get target speed and acceleration
    target_speed_ = msg->v_des;
    float target_accel = msg->a_des;
    ramp_time_ = target_speed_ / target_accel;

    // Find goal position
    Eigen::Vector3f goal_pos;
    goal_pos(0) = msg->x, goal_pos(1) = msg->y, goal_pos(2) = msg->z;
    if(msg->relative)
      goal_pos += start_pos;

    // Find distance and direction to goal
    const float total_dist = (goal_pos - start_pos).norm();
    Eigen::Vector3f dir = (goal_pos - start_pos).normalized();

    // Compute the coefficients
    Eigen::Matrix<float, 7, 7> Ainv;
    Ainv << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0 / 6.0, 0, 35, -20, -15, -5, 2.5, -2.0 / 3.0,
        -1.0 / 6.0, -84, 45, 39, 10, -7, 1, 0.5, 70, -36, -34, -7.5, 6.5, -2.0 / 3.0, -0.5, -20, 10, 10, 2, -2,
        1.0 / 6.0, 1.0 / 6.0;

    Eigen::Matrix<float, 7, 1> b;
    b << target_speed_, 0, 0, 0, 0, 0, 0;
    vel_coeffs_ = Ainv * b;

    // Compute the ramp distance
    ramp_dist_ = (vel_coeffs_(0) / 2 + vel_coeffs_(1) / 3 + vel_coeffs_(2) / 4 + vel_coeffs_(3) / 5 +
                  vel_coeffs_(4) / 6 + vel_coeffs_(5) / 7 + vel_coeffs_(6) / 8) *
                 ramp_time_;

    // Check to make sure that twice the ramp distance is less than the entire distance
    if(2 * ramp_dist_ < total_dist)
    {
      // Set the parameters for the trajectory
      start_pos_ = start_pos;
      total_dist_ = total_dist;
      dir_ = dir;

      // Compute total time of trajectory
      total_time_ = (total_dist_ - 2 * ramp_dist_) / target_speed_ + 2 * ramp_time_;

      // Set the target yaw and max yaw dot
      start_yaw_ = ICs_.yaw();
      goal_yaw_ = std::fmod(msg->yaw, 2 * M_PI);
      yaw_dot_max_ = (goal_yaw_ - start_yaw_) / (total_time_ - ramp_time_);

      // Set goal_set to true and goal_reached to false
      goal_set_ = true;
      goal_reached_ = false;
    }
    else
    {
      ROS_ERROR("increase the ramp acceleration");
      tracker_server_->setAborted();
    }
  }
  else
  {
    ROS_ERROR("v_des and a_des must be nonzero!");
    tracker_server_->setAborted();
  }
}

uint8_t SmoothVelTracker::status() const
{
  return goal_reached_ ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED) :
                         static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SmoothVelTracker, kr_trackers_manager::Tracker);
