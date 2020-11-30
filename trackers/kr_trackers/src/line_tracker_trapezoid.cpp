// TODO: make into actionlib

#include <initial_conditions.h>
#include <kr_mav_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

class LineTrackerTrapezoid : public kr_trackers_manager::Tracker
{
 public:
  LineTrackerTrapezoid(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  void goal_callback(const kr_mav_msgs::LineTrackerGoal::ConstPtr &msg);

  ros::Subscriber sub_goal_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f start_pos_, goal_, pos_;
  ros::Time traj_start_;
  float cur_yaw_, start_yaw_;
  float t_accel_, t_constant_;
};

LineTrackerTrapezoid::LineTrackerTrapezoid(void)
    : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false)
{
}

void LineTrackerTrapezoid::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "line_tracker_trapezoid");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ =
      priv_nh.subscribe("goal", 10, &LineTrackerTrapezoid::goal_callback, this, ros::TransportHints().tcpNoDelay());
}

bool LineTrackerTrapezoid::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    start_pos_ = pos_;
    start_yaw_ = cur_yaw_;
    active_ = true;
  }
  return active_;
}

void LineTrackerTrapezoid::Deactivate(void)
{
  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr LineTrackerTrapezoid::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  const ros::Time t_now = msg->header.stamp;

  if(!active_)
    return kr_mav_msgs::PositionCommand::Ptr();

  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;

  if(goal_set_)
  {
    traj_start_ = t_now;
    start_pos_ = ICs_.pos();
    start_yaw_ = ICs_.yaw();
    cmd->yaw = start_yaw_;

    const float total_dist = (goal_ - start_pos_).norm();
    if(total_dist > v_des_ * v_des_ / a_des_)
    {
      t_accel_ = v_des_ / a_des_;
      t_constant_ = total_dist / v_des_ - v_des_ / a_des_;
    }
    else
    {
      t_accel_ = std::sqrt(total_dist / a_des_);
      t_constant_ = 0;
    }

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_ - start_pos_).normalized();

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

  const float traj_time = (t_now - traj_start_).toSec();
  if(traj_time <= t_accel_)
  {
    // Accelerate
    const float dT = traj_time;
    a = a_des_ * dir;
    v = a_des_ * dir * dT;
    x = start_pos_ + 0.5 * a_des_ * dir * dT * dT;
  }
  else if(traj_time <= (t_accel_ + t_constant_))
  {
    // Constant speed
    const float dT = traj_time - t_accel_;
    a = Eigen::Vector3f::Zero();
    v = a_des_ * dir * t_accel_;
    x = (start_pos_ + 0.5 * a_des_ * dir * t_accel_ * t_accel_) + (v * dT);
  }
  else if(traj_time <= (t_accel_ + t_constant_ + t_accel_))
  {
    // Decelerate
    const float dT = traj_time - (t_accel_ + t_constant_);
    a = -a_des_ * dir;
    v = a_des_ * dir * t_accel_ - a_des_ * dir * dT;
    x = (start_pos_ + 0.5 * a_des_ * dir * t_accel_ * t_accel_) + (a_des_ * dir * t_accel_ * t_constant_) +
        (a_des_ * dir * t_accel_ * dT - 0.5 * a_des_ * dir * dT * dT);
  }
  else
  {
    // Reached goal
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  ICs_.set_from_cmd(cmd);
  return cmd;
}

void LineTrackerTrapezoid::goal_callback(const kr_mav_msgs::LineTrackerGoal::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  if(msg->relative)
    goal_ += ICs_.pos();

  if(msg->v_des > 0)
    v_des_ = msg->v_des;
  else
    v_des_ = default_v_des_;

  if(msg->a_des > 0)
    a_des_ = msg->a_des;
  else
    a_des_ = default_a_des_;

  goal_set_ = true;
  goal_reached_ = false;
}

uint8_t LineTrackerTrapezoid::status() const
{
  return goal_reached_ ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED) :
                         static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerTrapezoid, kr_trackers_manager::Tracker);
