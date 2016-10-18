#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <initial_conditions.h>

class LineTrackerDistance : public trackers_manager::Tracker
{
 public:
  LineTrackerDistance(void);

  void Initialize(const ros::NodeHandle &nh, const ros::NodeHandle &parent_nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg);

  ros::Subscriber sub_goal_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f start_, goal_, pos_;
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];
};

LineTrackerDistance::LineTrackerDistance(void) :
    pos_set_(false),
    goal_set_(false),
    goal_reached_(true),
    active_(false)
{
}

void LineTrackerDistance::Initialize(const ros::NodeHandle &nh, const ros::NodeHandle &parent_nh)
{
  parent_nh.param("gains/pos/x", kx_[0], 2.5);
  parent_nh.param("gains/pos/y", kx_[1], 2.5);
  parent_nh.param("gains/pos/z", kx_[2], 5.0);
  parent_nh.param("gains/vel/x", kv_[0], 2.2);
  parent_nh.param("gains/vel/y", kv_[1], 2.2);
  parent_nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker_distance");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerDistance::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
}

bool LineTrackerDistance::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    // Set start and start_yaw here so that even if the goal was sent at a
    // different position, we still use the current position as start
    start_ = pos_;
    start_yaw_ = yaw_;
    active_ = true;
  }
  return active_;
}

void LineTrackerDistance::Deactivate(void)
{
  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr LineTrackerDistance::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  static ros::Time t_prev;
  const double dT = (msg->header.stamp - t_prev).toSec();
  t_prev = msg->header.stamp;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if(goal_reached_)
  {
    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_ - start_).normalized();
  const float total_dist = (goal_ - start_).norm();
  const float d = (pos_ - start_).dot(dir);
  const Eigen::Vector3f proj = start_ + d*dir;

  const float v_max = std::min(std::sqrt(a_des_*total_dist), v_des_);
  const float ramp_dist = v_max*v_max/(2*a_des_);

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

  if((pos_ - goal_).norm() <= epsilon_) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(d > total_dist) // Overshoot
  {
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des_*dir;
    v = Eigen::Vector3f::Zero();
    x = goal_;
  }
  else if(d >= (total_dist - ramp_dist) && d <= total_dist) // Decelerate
  {
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des_*dir;
    v = std::sqrt(2*a_des_*(total_dist - d))*dir;
    x = proj + v*dT + 0.5*a*dT*dT;
  }
  else if(d > ramp_dist && d < total_dist - ramp_dist) // Constant velocity
  {
    ROS_DEBUG_THROTTLE(1, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max*dir;
    x = proj + v*dT;
  }
  else if(d >= 0 && d <= ramp_dist) // Accelerate
  {
    ROS_DEBUG_THROTTLE(1, "Accelerate");
    a = a_des_*dir;
    v = std::sqrt(2*a_des_*d)*dir;
    x = proj + v*dT + 0.5*a*dT*dT;
  }
  else if(d < 0)
  {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des_*dir;
    v = Eigen::Vector3f::Zero();
    x = start_ + 0.5*a*dT*dT;
  }
  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  ICs_.set_from_cmd(cmd);
  return cmd;
}

void LineTrackerDistance::goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  if (msg->relative)
    goal_ += ICs_.pos();

  if(msg->v_des > 0)
    v_des_ = msg->v_des;
  else
    v_des_ = default_v_des_;

  if(msg->a_des > 0)
    a_des_ = msg->a_des;
  else
    a_des_ = default_a_des_;

  start_ = pos_;
  start_yaw_ = yaw_;

  goal_set_ = true;
  goal_reached_ = false;
}

const quadrotor_msgs::TrackerStatus::Ptr LineTrackerDistance::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);

  msg->status = goal_reached_ ? (uint8_t)
    quadrotor_msgs::TrackerStatus::SUCCEEDED : quadrotor_msgs::TrackerStatus::ACTIVE;

  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerDistance, trackers_manager::Tracker);
