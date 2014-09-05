#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <line_tracker/DesVelAcc.h>

class LineTrackerTrapezoid : public controllers_manager::Controller
{
 public:
  LineTrackerTrapezoid(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  void goal_callback(const geometry_msgs::Vector3::ConstPtr &msg);
  bool set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
                       line_tracker::DesVelAcc::Response &res);

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  Eigen::Vector3f start_pos_, goal_, pos_;
  ros::Time traj_start_;
  float cur_yaw_, start_yaw_;
  float t_accel_, t_constant_;
  double kx_[3], kv_[3];
};

LineTrackerTrapezoid::LineTrackerTrapezoid(void) :
    pos_set_(false),
    goal_set_(false),
    goal_reached_(true),
    active_(false)
{
}

void LineTrackerTrapezoid::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerTrapezoid::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
  srv_param_ = priv_nh.advertiseService("set_des_vel_acc", &LineTrackerTrapezoid::set_des_vel_acc, this);
}

bool LineTrackerTrapezoid::Activate(void)
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
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr LineTrackerTrapezoid::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  const ros::Time t_now = msg->header.stamp;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if(goal_set_)
  {
    traj_start_ = t_now;
    start_pos_ = pos_;
    start_yaw_ = cur_yaw_;
    cmd->yaw = start_yaw_;

    const float total_dist = (goal_-pos_).norm();
    if(total_dist > v_des_*v_des_/a_des_)
    {
      t_accel_ = v_des_/a_des_;
      t_constant_ = total_dist/v_des_ - v_des_/a_des_;
    }
    else
    {
      t_accel_ = std::sqrt(total_dist/a_des_);
      t_constant_ = 0;
    }

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_ - start_pos_).normalized();

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

  const float traj_time = (t_now - traj_start_).toSec();
  if(traj_time <= t_accel_)
  {
    // Accelerate
    const float dT = traj_time;
    a = a_des_*dir;
    v = a_des_*dir*dT;
    x = start_pos_ + 0.5*a_des_*dir*dT*dT;
  }
  else if(traj_time <= (t_accel_ + t_constant_))
  {
    // Constant speed
    const float dT = traj_time - t_accel_;
    a = Eigen::Vector3f::Zero();
    v = a_des_*dir*t_accel_;
    x = (start_pos_ + 0.5*a_des_*dir*t_accel_*t_accel_) + (v*dT);
  }
  else if(traj_time <= (t_accel_ + t_constant_ + t_accel_))
  {
    // Decelerate
    const float dT = traj_time - (t_accel_ + t_constant_);
    a = -a_des_*dir;
    v = a_des_*dir*t_accel_ - a_des_*dir*dT;
    x = (start_pos_ + 0.5*a_des_*dir*t_accel_*t_accel_) + (a_des_*dir*t_accel_*t_constant_) +
        (a_des_*dir*t_accel_*dT - 0.5*a_des_*dir*dT*dT);
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
  return cmd;
}

void LineTrackerTrapezoid::goal_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  goal_set_ = true;
  goal_reached_ = false;
}

bool LineTrackerTrapezoid::set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
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
PLUGINLIB_EXPORT_CLASS(LineTrackerTrapezoid, controllers_manager::Controller);
