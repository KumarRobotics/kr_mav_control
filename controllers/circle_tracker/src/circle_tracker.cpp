#include <controllers_manager/Controller.h>
#include <std_msgs/Empty.h>
#include <Eigen/Geometry>
#include <circle_tracker/DesVelAcc.h>
#include <tf/transform_datatypes.h>

class CircleTracker : public controllers_manager::Controller
{
 public:
  CircleTracker(void);
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(
      const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void trigger_callback(const std_msgs::Empty::ConstPtr &msg);
  bool set_des_vel_acc(circle_tracker::DesVelAcc::Request &req,
                       circle_tracker::DesVelAcc::Response &res);

  ros::Subscriber sub_trig_;
  ros::ServiceServer srv_param_;
  bool pos_set_, goal_reached_;
  double default_omega_des_, default_alpha_des_, epsilon_;
  float omega_des_, alpha_des_;
  bool active_;
  bool triggered_;

  Eigen::Vector3f cur_pos_;
  ros::Time traj_start_;
  float t_accel_, t_constant_;
  float cur_yaw_, start_yaw_;
  Eigen::Vector3f start_pos_;
  float start_angle_;
  float circle_radius_;
  double num_circles_;
  double kx_[3], kv_[3];
};

CircleTracker::CircleTracker(void)
    : pos_set_(false), goal_reached_(true), active_(false), triggered_(false)
{
}

void CircleTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "circle_tracker");

  priv_nh.param("default_omega_des", default_omega_des_, 0.5);
  priv_nh.param("default_alpha_des", default_alpha_des_, 0.5);
  priv_nh.param("num_circles", num_circles_, 3.0);

  omega_des_ = default_omega_des_;
  alpha_des_ = default_alpha_des_;

  sub_trig_ = priv_nh.subscribe("trigger", 1, &CircleTracker::trigger_callback,
                                this, ros::TransportHints().tcpNoDelay());
  srv_param_ = priv_nh.advertiseService("set_des_vel_acc",
                                        &CircleTracker::set_des_vel_acc, this);
}

bool CircleTracker::Activate(void)
{
  if(pos_set_ && triggered_)
    active_ = true;

  return active_;
}

void CircleTracker::Deactivate(void)
{
  active_ = false;
}

const quadrotor_msgs::PositionCommand::Ptr CircleTracker::update(
    const nav_msgs::Odometry::ConstPtr &msg)
{
  cur_pos_(0) = msg->pose.pose.position.x;
  cur_pos_(1) = msg->pose.pose.position.y;
  cur_pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  const ros::Time t_now = msg->header.stamp;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  if(triggered_)
  {
    traj_start_ = t_now;
    start_pos_ = cur_pos_;
    start_yaw_ = cur_yaw_;
    start_angle_ = std::atan2(cur_pos_(1), cur_pos_(0));
    circle_radius_ = hypotf(cur_pos_(0), cur_pos_(1));

    const float total_angle = num_circles_ * 2 * M_PI;
    if(total_angle > omega_des_ * omega_des_ / alpha_des_)
    {
      t_accel_ = omega_des_ / alpha_des_;
      t_constant_ = total_angle / omega_des_ - omega_des_ / alpha_des_;
    }
    else
    {
      t_accel_ = std::sqrt(total_angle / alpha_des_);
      t_constant_ = 0;
    }

    triggered_ = false;
  }

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  float alpha(0), omega(0), theta(0);
  const float traj_time = (t_now - traj_start_).toSec();
  if(traj_time <= t_accel_)
  {
    // Accelerate
    const float dT = traj_time;
    alpha = alpha_des_;
    omega = alpha * dT;
    theta = start_angle_ + 0.5 * alpha * dT * dT;
  }
  else if(traj_time <= (t_accel_ + t_constant_))
  {
    // Constant speed
    const float dT = traj_time - t_accel_;
    alpha = 0;
    omega = alpha_des_ * t_accel_;
    theta =
        start_angle_ + (0.5 * alpha_des_ * t_accel_ * t_accel_) + (omega * dT);
  }
  else if(traj_time <= (t_accel_ + t_constant_ + t_accel_))
  {
    // Decelerate
    const float dT = traj_time - (t_accel_ + t_constant_);
    alpha = -alpha_des_;
    omega = alpha_des_ * t_accel_ - alpha_des_ * dT;
    theta = (start_angle_ + 0.5 * alpha_des_ * t_accel_ * t_accel_) +
            (alpha_des_ * t_accel_ * t_constant_) +
            (alpha_des_ * t_accel_ * dT - 0.5 * alpha_des_ * dT * dT);
  }
  else
  {
    // Reached goal
    alpha = 0;
    omega = 0;
    theta = start_angle_ + num_circles_ * 2 * M_PI;
    goal_reached_ = true;
  }

  const float r = circle_radius_;
  cmd->position.x = r * std::cos(theta);
  cmd->position.y = r * std::sin(theta);
  cmd->position.z = start_pos_(2);
  cmd->velocity.x = -r * std::sin(theta) * omega;
  cmd->velocity.y = r * std::cos(theta) * omega;
  cmd->velocity.z = 0;
  cmd->acceleration.x =
      -r * std::cos(theta) * omega * omega - r * std::sin(theta) * alpha;
  cmd->acceleration.y =
      -r * std::sin(theta) * omega * omega + r * std::cos(theta) * alpha;
  cmd->acceleration.z = 0;

  return cmd;
}

void CircleTracker::trigger_callback(const std_msgs::Empty::ConstPtr &msg)
{
  triggered_ = true;
}

bool CircleTracker::set_des_vel_acc(circle_tracker::DesVelAcc::Request &req,
                                    circle_tracker::DesVelAcc::Response &res)
{
  // Don't allow changes while already running a trajectory
  if(!goal_reached_)
    return false;

  // Only non-negative omega_des and alpha_des allowed
  if(req.omega_des < 0 || req.alpha_des < 0)
    return false;

  if(req.omega_des > 0)
    omega_des_ = req.omega_des;
  else
    omega_des_ = default_omega_des_;

  if(req.alpha_des > 0)
    alpha_des_ = req.alpha_des;
  else
    alpha_des_ = default_alpha_des_;

  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CircleTracker, controllers_manager::Controller);
