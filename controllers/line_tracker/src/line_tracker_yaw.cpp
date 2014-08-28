#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <quadrotor_msgs/PositionCommand.h>
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  void goal_callback(const std_msgs::Float64::ConstPtr &msg);
  bool set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
                       line_tracker::DesVelAcc::Response &res);

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_, dir_;
  bool active_;

  Eigen::Vector3f start_pos_, pos_;
  double goal_;
  ros::Time traj_start_;
  float cur_yaw_, start_yaw_;
  float t_accel_, t_constant_;
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

  // priv_nh.param("default_v_des", default_v_des_, 0.5);
  // priv_nh.param("default_a_des", default_a_des_, 0.5);
  // priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = 0.5; // default_v_des_;
  a_des_ = 0.3; //default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerYaw::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
  srv_param_ = priv_nh.advertiseService("set_des_vel_acc", &LineTrackerYaw::set_des_vel_acc, this);
}

bool LineTrackerYaw::Activate(void)
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
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  const ros::Time t_now = msg->header.stamp;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // Don't move while yawing
  Eigen::Vector3f x, v, a;
  x = start_pos_;
  v = Eigen::Vector3f::Zero();
  a = Eigen::Vector3f::Zero();
   
  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);

  if(goal_set_)
  {
    traj_start_ = t_now;
    start_yaw_ = cur_yaw_;
  
    // double angle, y1, y2;
    tf::Quaternion q0, q1, q2, q3;
  
    // Determine the quaternion representing the rotation between
    // the start pose (q1) and the desired pose (q2).
    q0.setRPY(0,0,0);
    q1.setRPY(0, 0, start_yaw_);
    q2.setRPY(0, 0, goal_);
    q3 = q2 * q1.inverse();
    q3 = q0.nearest(q3);
    // cout << "angle: " << q3.getAngle() << endl;

    // angle = q1.angleShortestPath(q2);
    // cout << "Shortest Angle: " << angle << endl;
    
    // Determine the direction of rotation based on the
    // sign of the z component of the axis of rotation
    tf::Vector3 axis;
    axis = q3.getAxis();
    // cout << "dir_ :" << axis[2] << endl;
    dir_ = axis[2]; 

    const float total_dist = q3.getAngle();
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
    cmd->yaw = goal_, cmd->yaw_dot = 0;
    return cmd;
  }

  // Determine the yaw and yaw_dot
  double yaw, yaw_dot;
  const float traj_time = (t_now - traj_start_).toSec();
  if(traj_time <= t_accel_)
  {
    // Accelerate
    const float dT = traj_time;
    yaw_dot = a_des_ * dir_ * dT;
    yaw = start_yaw_ + 0.5 * a_des_ * dir_ * dT * dT; 
  }
  else if(traj_time <= (t_accel_ + t_constant_))
  {
    // Constant speed
    const float dT = traj_time - t_accel_;
    yaw_dot = a_des_ * dir_ * t_accel_;
    yaw = start_yaw_ + (0.5 * a_des_ * dir_ * t_accel_ * t_accel_) + yaw_dot * dT; 
  }
  else if(traj_time <= (t_accel_ + t_constant_ + t_accel_))
  {
    // Decelerate
    const float dT = traj_time - (t_accel_ + t_constant_);
    yaw_dot = a_des_ * dir_ * t_accel_ - a_des_ * dir_ * dT; 
    yaw = start_yaw_ + ( 0.5 * a_des_ * dir_ * t_accel_ * t_accel_ ) + (a_des_ * dir_ * t_accel_ * t_constant_) +
        (a_des_ * dir_ * t_accel_ * dT - 0.5 * a_des_ * dir_ * dT * dT);
  }
  else
  {
    // Reached goal
    goal_reached_ = true;
    yaw = goal_;
    yaw_dot = 0;
  }

    cmd->yaw = yaw, cmd->yaw_dot = yaw_dot;
  return cmd;
}

void LineTrackerYaw::goal_callback(const std_msgs::Float64::ConstPtr &msg)
{
  goal_ = msg->data;

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
PLUGINLIB_DECLARE_CLASS(line_tracker, LineTrackerYaw, LineTrackerYaw, controllers_manager::Controller);
