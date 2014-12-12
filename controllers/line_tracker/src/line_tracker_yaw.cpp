#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <line_tracker/DesVelAcc.h>

class LineTrackerYaw : public controllers_manager::Controller
{
 public:
  LineTrackerYaw(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void goal_callback(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);
  bool set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
                       line_tracker::DesVelAcc::Response &res);

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_,
         default_yaw_v_des_, default_yaw_a_des_, epsilon_;
  float v_des_, a_des_, yaw_v_des_, yaw_a_des_;
  bool active_;
  ros::Time traj_start_;

  Eigen::Vector3f start_pos_, goal_pos_, pos_;
  Eigen::Vector3f translation_dir_;
  float translation_dist_;
  float yaw_, start_yaw_, goal_yaw_;
  float yaw_dir_;
  float yaw_dist_;
  float t_yaw_accel_, t_yaw_constant_;
  double kx_[3], kv_[3];
};

LineTrackerYaw::LineTrackerYaw(void) :
    pos_set_(false),
    goal_set_(false),
    goal_reached_(true),
    active_(false)
{
}

void LineTrackerYaw::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker_yaw");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("default_yaw_v_des", default_yaw_v_des_, 0.8);
  priv_nh.param("default_yaw_a_des", default_yaw_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;
  yaw_v_des_ = default_yaw_v_des_;
  yaw_a_des_ = default_yaw_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerYaw::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
  srv_param_ = priv_nh.advertiseService("set_des_vel_acc", &LineTrackerYaw::set_des_vel_acc, this);
}

bool LineTrackerYaw::Activate(void)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    // Set start and start_yaw here so that even if the goal was sent at a
    // different position, we still use the current position as start
    start_pos_ = pos_;
    start_yaw_ = yaw_;

    active_ = true;

    traj_start_ = ros::Time::now();
  }
  return active_;
}

void LineTrackerYaw::Deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr LineTrackerYaw::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  // Timing
  ros::Time t_now = ros::Time::now(); //msg->header.stamp;
  static ros::Time t_prev = t_now;
  double dT = (t_now - t_prev).toSec();
  t_prev = t_now; // msg->header.stamp;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  bool goal_pos_reached(false), goal_yaw_reached(false);

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if(goal_reached_)
  {
    cmd->position.x = goal_pos_(0), cmd->position.y = goal_pos_(1), cmd->position.z = goal_pos_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    cmd->yaw = goal_yaw_, cmd->yaw_dot = 0;
    return cmd;
  }

  // ===========
  // Yaw control
  // ===========
  const float traj_time = (t_now - traj_start_).toSec();
  float yaw, yaw_dot;
  if(traj_time <= t_yaw_accel_)
  {
    // Accelerate
    const float dT_yaw = traj_time;
    yaw_dot = yaw_a_des_*yaw_dir_*dT_yaw;
    yaw = start_yaw_ + 0.5f*yaw_a_des_*yaw_dir_*dT_yaw*dT_yaw;
  }
  else if(traj_time <= (t_yaw_accel_ + t_yaw_constant_))
  {
    // Constant speed
    const float dT_yaw = traj_time - t_yaw_accel_;
    yaw_dot = yaw_a_des_*yaw_dir_*t_yaw_accel_;
    yaw = start_yaw_ + (0.5f*yaw_a_des_*yaw_dir_*t_yaw_accel_*t_yaw_accel_) + yaw_dot*dT_yaw;
  }
  else if(traj_time <= (t_yaw_accel_ + t_yaw_constant_ + t_yaw_accel_))
  {
    // Decelerate
    const float dT_yaw = traj_time - (t_yaw_accel_ + t_yaw_constant_);
    yaw_dot = yaw_a_des_*yaw_dir_*t_yaw_accel_ - yaw_a_des_*yaw_dir_*dT_yaw;
    yaw = start_yaw_ + (0.5f*yaw_a_des_*yaw_dir_*t_yaw_accel_*t_yaw_accel_) +
        (yaw_a_des_*yaw_dir_*t_yaw_accel_*t_yaw_constant_) +
        (yaw_a_des_*yaw_dir_*t_yaw_accel_*dT_yaw - 0.5f*yaw_a_des_*yaw_dir_*dT_yaw*dT_yaw);
  }
  else
  {
    // Reached goal
    goal_yaw_reached = true;
    yaw = goal_yaw_;
    yaw_dot = 0;
  }

  // ================
  // Position control
  // ================
  const float d = (pos_ - start_pos_).dot(translation_dir_);
  const Eigen::Vector3f proj = start_pos_ + d*translation_dir_;

  const float v_max = std::min(std::sqrt(a_des_*translation_dist_), v_des_);
  const float ramp_dist = v_max*v_max/(2*a_des_);

  Eigen::Vector3f x, v, a;
  x = start_pos_;
  v = Eigen::Vector3f::Zero();
  a = Eigen::Vector3f::Zero();

  if((pos_ - goal_pos_).norm() <= epsilon_) // Reached goal
  {
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_pos_;
    goal_pos_reached = true;
  }
  else if(d > translation_dist_) // Overshoot
  {
    a = -a_des_*translation_dir_;
    v = Eigen::Vector3f::Zero();
    x = goal_pos_;
  }
  else if(d >= (translation_dist_ - ramp_dist) && d <= translation_dist_) // Decelerate
  {
    a = -a_des_*translation_dir_;
    v = std::sqrt(2*a_des_*(translation_dist_ - d))*translation_dir_;
    x = proj + v*dT + 0.5f*a*dT*dT;
  }
  else if(d > ramp_dist && d < translation_dist_ - ramp_dist) // Constant velocity
  {
    a = Eigen::Vector3f::Zero();
    v = v_max*translation_dir_;
    x = proj + v*dT;
  }
  else if(d >= 0 && d <= ramp_dist) // Accelerate
  {
    a = a_des_*translation_dir_;
    v = std::sqrt(2*a_des_*d)*translation_dir_;
    x = proj + v*dT + 0.5f*a*dT*dT;
  }
  else if(d < 0) // Undershoot
  {
    a = a_des_*translation_dir_;
    v = Eigen::Vector3f::Zero();
    x = start_pos_ + 0.5f*a*dT*dT;
  }

  goal_reached_ = goal_pos_reached && goal_yaw_reached;

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  cmd->yaw = yaw, cmd->yaw_dot = yaw_dot;
  return cmd;
}

void LineTrackerYaw::goal_callback(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  goal_pos_(0) = msg->x;
  goal_pos_(1) = msg->y;
  goal_pos_(2) = msg->z;
  goal_yaw_ = msg->yaw;

  // Initial values
  start_pos_ = pos_;
  start_yaw_ = yaw_;
  traj_start_ = ros::Time::now();

  translation_dir_ = (goal_pos_ - start_pos_).normalized();
  translation_dist_ = (goal_pos_ - start_pos_).norm();

  // Find shortest angle and direction to go from start_yaw_ to goal_yaw_
  yaw_dist_ = goal_yaw_ - start_yaw_;
  const float pi(M_PI); // Defined so as to force float type
  yaw_dist_ = std::fmod(yaw_dist_, 2*pi);
  if(yaw_dist_ > pi)
    yaw_dist_ -= 2*pi;
  else if(yaw_dist_ < -pi)
    yaw_dist_ += 2*pi;
  yaw_dir_ = (yaw_dist_ >= 0) ? 1 : -1;
  yaw_dist_ = std::abs(yaw_dist_);

  // Compute times for accel and constant vel stages of trapezoidal yaw velocity profile
  if(yaw_dist_ > yaw_v_des_ * yaw_v_des_ / yaw_a_des_)
  {
    t_yaw_accel_ = yaw_v_des_ / yaw_a_des_;
    t_yaw_constant_ = yaw_dist_ / yaw_v_des_ - yaw_v_des_ / yaw_a_des_;
  }
  else
  {
    t_yaw_accel_ = std::sqrt(yaw_dist_ / yaw_a_des_);
    t_yaw_constant_ = 0;
  }

  goal_set_ = true;
  goal_reached_ = false;
}

bool LineTrackerYaw::set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
                                     line_tracker::DesVelAcc::Response &res)
{
  // Don't allow changes while already following a line
  if(!goal_reached_)
    return false;

  // Only non-negative v_des and a_des allowed
  if(req.v_des < 0 || req.a_des < 0)
    return false;

  if(req.v_des > 0)
    v_des_ = req.v_des;
  else
    v_des_ = default_v_des_;

  if(req.a_des > 0)
    a_des_ = req.a_des;
  else
    a_des_ = default_a_des_;

  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerYaw, controllers_manager::Controller);
