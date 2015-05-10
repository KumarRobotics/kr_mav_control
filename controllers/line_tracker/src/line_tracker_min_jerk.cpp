#include <ros/ros.h>
#include <controllers_manager/Controller.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <line_tracker/DesVelAcc.h>

class LineTrackerMinJerk : public controllers_manager::Controller
{
 public:
  LineTrackerMinJerk(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void goal_callback(const geometry_msgs::Vector3::ConstPtr &msg);
  bool set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
                       line_tracker::DesVelAcc::Response &res);

  void gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                      const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                      const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                      float dt, Eigen::Vector3f coeffs[6]);

  ros::Subscriber sub_goal_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  Eigen::Vector3f goal_, pos_, vel_;
  ros::Time traj_start_;
  float traj_duration_;
  Eigen::Vector3f coeffs_[6];
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];
};

LineTrackerMinJerk::LineTrackerMinJerk(void) :
    pos_set_(false),
    goal_set_(false),
    goal_reached_(true),
    active_(false)
{
}

void LineTrackerMinJerk::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker_min_jerk");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerMinJerk::goal_callback, this,
                                ros::TransportHints().tcpNoDelay());
  srv_param_ = priv_nh.advertiseService("set_des_vel_acc", &LineTrackerMinJerk::set_des_vel_acc, this);
}

bool LineTrackerMinJerk::Activate(void)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    active_ = true;
  }
  return active_;
}

void LineTrackerMinJerk::Deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr LineTrackerMinJerk::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
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
    start_yaw_ = yaw_;
    // Min-Jerk trajectory
    const float total_dist = (goal_-pos_).norm();
    if(total_dist > v_des_*v_des_/a_des_)
      traj_duration_ = total_dist/v_des_ + v_des_/a_des_;
    else
      traj_duration_ = 2*std::sqrt(total_dist/a_des_);

    gen_trajectory(pos_, goal_, vel_, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                   traj_duration_, coeffs_);
    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    return cmd;
  }

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero()), j(Eigen::Vector3f::Zero());

  const float traj_time = (t_now - traj_start_).toSec();
  if(/*((pos_ - goal_).norm() <= epsilon_) ||*/ (traj_time >= traj_duration_) || (traj_duration_ < 0.3)) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    j = Eigen::Vector3f::Zero();
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else
  {
    const float t = traj_time;
    x = coeffs_[0] + t*(coeffs_[1] + t*(coeffs_[2] + t*(coeffs_[3] + t*(coeffs_[4] + t*coeffs_[5]))));
    v = coeffs_[1] + t*(2*coeffs_[2] + t*(3*coeffs_[3] + t*(4*coeffs_[4] + t*5*coeffs_[5])));
    a = 2*coeffs_[2] + t*(6*coeffs_[3] + t*(12*coeffs_[4] + t*20*coeffs_[5]));
    j = 6*coeffs_[3] + t*(24*coeffs_[4] + t*60*coeffs_[5]);
  }

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  cmd->jerk.x = j(0), cmd->jerk.y = j(1), cmd->jerk.z = j(2);
  return cmd;
}

void LineTrackerMinJerk::goal_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  goal_set_ = true;
  goal_reached_ = false;
}

bool LineTrackerMinJerk::set_des_vel_acc(line_tracker::DesVelAcc::Request &req,
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

void LineTrackerMinJerk::gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                                 const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                                 const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                                 float dt, Eigen::Vector3f coeffs[6])
{
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  float dt5 = dt4*dt;

  Eigen::Matrix<float,6,6> A;
  A << 1,  0,    0,     0,      0,      0,
    1, dt,  dt2,   dt3,    dt4,    dt5,
    0,  1,    0,     0,      0,      0,
    0,  1, 2*dt, 3*dt2,  4*dt3,  5*dt4,
    0,  0,    2,     0,      0,      0,
    0,  0,    2,  6*dt, 12*dt2, 20*dt3;
  Eigen::Matrix<float, 6, 3> b;
  b << xi.transpose(), xf.transpose(), vi.transpose(), vf.transpose(), ai.transpose(), af.transpose();

  Eigen::Matrix<float, 6, 3> x;
  x = A.colPivHouseholderQr().solve(b);
  for(int i = 0; i < 6; i++)
  {
    coeffs[i] = x.row(i).transpose();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerMinJerk, controllers_manager::Controller);
