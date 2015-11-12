#include <iostream>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

/**
 * Class to hold initial conditions for trajectory generation.
 * In most cases we would want to have consecutive trajectories to smoothly
 * transition from one to another, so we need to keep the desired command
 * continuous. This class stores the last command so as to use it as an initial
 * condition for the next trajectory leading to a smooth desired command.
 * Previously, we used the current odom of the robot to set the initial
 * condition and if the robot had some tracking error, it would lead to a jump
 * in the desired which we want to avoid.
 */
class InitialConditions
{
 public:
  void set_from_last_cmd(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void set_from_odom(const nav_msgs::Odometry::ConstPtr &msg);
  Eigen::Vector3f pos() const { return pos_; }
  Eigen::Vector3f vel() const { return vel_; }
  Eigen::Vector3f acc() const { return acc_; }
  Eigen::Vector3f jrk() const { return jrk_; }
  float yaw() const { return yaw_; }
  float yaw_dot() const { return yaw_dot_; }
  ros::Time time() const { return time_; }
  void reset();

 private:
  Eigen::Vector3f pos_, vel_, acc_, jrk_;
  float yaw_, yaw_dot_;
  bool last_cmd_valid_;
  ros::Time time_;
};

void InitialConditions::set_from_last_cmd(
    const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  pos_ = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  vel_ = Eigen::Vector3f(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  acc_ = Eigen::Vector3f(msg->acceleration.x, msg->acceleration.y,
                         msg->acceleration.z);
  jrk_ = Eigen::Vector3f(msg->jerk.x, msg->jerk.y, msg->jerk.z);
  yaw_ = msg->yaw;
  yaw_dot_ = msg->yaw_dot;

  time_ = msg->header.stamp;

  last_cmd_valid_ = true;
}

void InitialConditions::set_from_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!last_cmd_valid_)
  {
    pos_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
    vel_ = Eigen::Vector3f(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    acc_ = Eigen::Vector3f(0, 0, 0);
    jrk_ = Eigen::Vector3f(0, 0, 0);
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    yaw_dot_ = msg->twist.twist.angular.z; // TODO: Should double check which
                                           // frame (body or world) this is in
    time_ = ros::Time::now();
  }
}

void InitialConditions::reset()
{
  last_cmd_valid_ = false;
}

class LineTrackerMinJerk : public trackers_manager::Tracker
{
 public:
  LineTrackerMinJerk(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg);

  void gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                      const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                      const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                      const float &yawi, const float &yawf,
                      const float &yaw_dot_i, const float &yaw_dot_f, float dt,
                      Eigen::Vector3f coeffs[6], float yaw_coeffs[4]);

  ros::Subscriber sub_goal_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, default_yaw_v_des_, default_yaw_a_des_;
  float v_des_, a_des_, yaw_v_des_, yaw_a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f goal_;
  ros::Time traj_start_;
  float traj_duration_;
  Eigen::Vector3f coeffs_[6];
  float goal_yaw_, last_cmd_yaw_, yaw_coeffs_[4];

  double kx_[3], kv_[3];
};

LineTrackerMinJerk::LineTrackerMinJerk(void)
    : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false)
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
  priv_nh.param("default_a_des", default_a_des_, 0.3);
  priv_nh.param("default_yaw_v_des", default_yaw_v_des_, 0.8);
  priv_nh.param("default_yaw_a_des", default_yaw_a_des_, 0.2);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;
  yaw_v_des_ = default_yaw_v_des_;
  yaw_a_des_ = default_yaw_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &LineTrackerMinJerk::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
}

bool LineTrackerMinJerk::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    active_ = true;
  }
  ICs_.reset();
  return active_;
}

void LineTrackerMinJerk::Deactivate(void)
{
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr LineTrackerMinJerk::update(
    const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  const ros::Time t_now = ros::Time::now();

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if(goal_set_)
  {
    traj_start_ = ICs_.time();
    traj_duration_ = 0.5f;

    // Min-Jerk trajectory
    const float total_dist = (goal_ - ICs_.pos()).norm();
    const Eigen::Vector3f dir = (goal_ - ICs_.pos()) / total_dist;
    const float vel_proj = (ICs_.vel()).dot(dir);

    const float t_ramp = (v_des_ - vel_proj) / a_des_;

    const float distance_to_v_des =
        vel_proj * t_ramp + 0.5f * a_des_ * t_ramp * t_ramp;
    const float distance_v_des_to_stop = 0.5f * v_des_ * v_des_ / a_des_;

    const float ramping_distance = distance_to_v_des + distance_v_des_to_stop;

    if(total_dist > ramping_distance)
    {
      float t = (v_des_ - vel_proj) / a_des_ // Ramp up
                + (total_dist - ramping_distance) / v_des_ // Constant velocity
                + v_des_ / a_des_; // Ramp down

      traj_duration_ = std::max(traj_duration_, t);
    }
    else
    {
      // In this case, v_des_ is not reached. Assume bang bang acceleration.

      float vo = vel_proj;
      float distance_to_stop = 0.5f * vo * vo / a_des_;

      float t_dir; // The time required for the component along dir
      if(vo > 0.0f && total_dist < distance_to_stop)
      {
        // Currently traveling towards the goal and need to overshoot

        t_dir = vo / a_des_ +
                std::sqrt(2.0f) *
                    std::sqrt(vo * vo - 2.0f * a_des_ * total_dist) / a_des_;
      }
      else
      {
        // Ramp up to a velocity towards the goal before ramping down

        t_dir = -vo / a_des_ +
                std::sqrt(2.0f) *
                    std::sqrt(vo * vo + 2.0f * a_des_ * total_dist) / a_des_;
      }
      traj_duration_ = std::max(traj_duration_, t_dir);

      // The velocity component orthogonal to dir
      float v_ortho = (ICs_.vel() - dir * vo).norm();
      float t_non_dir =
          v_ortho / a_des_ // Ramp to zero velocity
          + std::sqrt(2.0f) * v_ortho / a_des_; // Get back to the dir line

      traj_duration_ = std::max(traj_duration_, t_non_dir);
    }

    // Find shortest angle and direction to go from yaw_ to goal_yaw_
    float yaw_dist, yaw_dir;
    yaw_dist = goal_yaw_ - ICs_.yaw();
    const float pi(M_PI); // Defined so as to force float type
    yaw_dist = std::fmod(yaw_dist, 2 * pi);
    if(yaw_dist > pi)
      yaw_dist -= 2 * pi;
    else if(yaw_dist < -pi)
      yaw_dist += 2 * pi;
    yaw_dir = (yaw_dist >= 0) ? 1 : -1;
    yaw_dist = std::abs(yaw_dist);
    goal_yaw_ = ICs_.yaw() + yaw_dir * yaw_dist;

    // Consider yaw in the trajectory duration
    if(yaw_dist > yaw_v_des_ * yaw_v_des_ / yaw_a_des_)
      traj_duration_ = std::max(
          traj_duration_, yaw_dist / yaw_v_des_ + yaw_v_des_ / yaw_a_des_);
    else
      traj_duration_ =
          std::max(traj_duration_, 2 * std::sqrt(yaw_dist / yaw_a_des_));

    // TODO: Should consider yaw_dot_. See hover() in MAVManager

    gen_trajectory(ICs_.pos(), goal_, ICs_.vel(), Eigen::Vector3f::Zero(),
                   ICs_.acc(), Eigen::Vector3f::Zero(), ICs_.yaw(), goal_yaw_,
                   ICs_.yaw_dot(), 0, traj_duration_, coeffs_, yaw_coeffs_);

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    cmd->position.x = goal_(0), cmd->position.y = goal_(1),
    cmd->position.z = goal_(2);
    cmd->yaw = goal_yaw_;
    cmd->yaw_dot = 0;
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;
    ICs_.set_from_last_cmd(cmd);
    return cmd;
  }

  Eigen::Vector3f x(ICs_.pos()), v(Eigen::Vector3f::Zero()),
      a(Eigen::Vector3f::Zero()), j(Eigen::Vector3f::Zero());
  float yaw_des, yaw_dot_des;

  const float traj_time = (t_now - traj_start_).toSec();
  if(traj_time >= traj_duration_) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    j = Eigen::Vector3f::Zero();
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    yaw_des = goal_yaw_;
    yaw_dot_des = 0;
    goal_reached_ = true;
  }
  else
  {
    float t = traj_time, t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    x = coeffs_[0] + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] +
        t4 * coeffs_[4] + t5 * coeffs_[5];
    v = coeffs_[1] + 2 * t * coeffs_[2] + 3 * t2 * coeffs_[3] +
        4 * t3 * coeffs_[4] + 5 * t4 * coeffs_[5];
    a = 2 * coeffs_[2] + 6 * t * coeffs_[3] + 12 * t2 * coeffs_[4] +
        20 * t3 * coeffs_[5];
    j = 6 * coeffs_[3] + 24 * t * coeffs_[4] + 60 * t2 * coeffs_[5];

    yaw_des = yaw_coeffs_[0] + t * yaw_coeffs_[1] + t2 * yaw_coeffs_[2] +
              t3 * yaw_coeffs_[3];
    yaw_dot_des =
        yaw_coeffs_[1] + 2 * t * yaw_coeffs_[2] + 3 * t2 * yaw_coeffs_[3];
  }

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->yaw = yaw_des;
  cmd->yaw_dot = yaw_dot_des;
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1),
  cmd->acceleration.z = a(2);
  cmd->jerk.x = j(0), cmd->jerk.y = j(1), cmd->jerk.z = j(2);

  ICs_.set_from_last_cmd(cmd);
  return cmd;
}

void LineTrackerMinJerk::goal_callback(
    const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
  goal_yaw_ = msg->yaw;

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

void LineTrackerMinJerk::gen_trajectory(
    const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
    const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
    const Eigen::Vector3f &ai, const Eigen::Vector3f &af, const float &yawi,
    const float &yawf, const float &yaw_dot_i, const float &yaw_dot_f, float dt,
    Eigen::Vector3f coeffs[6], float yaw_coeffs[4])
{
  float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;

  Eigen::Matrix<float, 6, 6> A;
  A << 1,  0,      0,       0,        0,        0,
       1, dt,    dt2,     dt3,      dt4,      dt5,
       0,  1,      0,       0,        0,        0,
       0,  1, 2 * dt, 3 * dt2,  4 * dt3,  5 * dt4,
       0,  0,      2,       0,        0,        0,
       0,  0,      2,  6 * dt, 12 * dt2, 20 * dt3;

  Eigen::Matrix<float, 6, 3> b;
  b << xi.transpose(), xf.transpose(), vi.transpose(), vf.transpose(),
      ai.transpose(), af.transpose();

  Eigen::Matrix<float, 6, 3> x;
  x = A.colPivHouseholderQr().solve(b);
  for(int i = 0; i < 6; i++)
  {
    coeffs[i] = x.row(i).transpose();
  }

  // Compute the trajectory for the yaw
  Eigen::Matrix<float, 4, 4> A_yaw;
  A_yaw << 1,  0,      0,       0,
           1, dt,    dt2,     dt3,
           0,  1,      0,       0,
           0,  1, 2 * dt, 3 * dt2;

  Eigen::Vector4f b_yaw;
  b_yaw << yawi, yawf, yaw_dot_i, yaw_dot_f;

  Eigen::Vector4f x_yaw;
  x_yaw = A_yaw.colPivHouseholderQr().solve(b_yaw);
  for(int i = 0; i < 4; i++)
  {
    yaw_coeffs[i] = x_yaw(i);
  }
}

const quadrotor_msgs::TrackerStatus::Ptr LineTrackerMinJerk::status()
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
PLUGINLIB_EXPORT_CLASS(LineTrackerMinJerk, trackers_manager::Tracker);
