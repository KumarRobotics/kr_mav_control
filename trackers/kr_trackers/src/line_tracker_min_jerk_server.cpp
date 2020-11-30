#include <actionlib/server/simple_action_server.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <memory>

class LineTrackerMinJerk : public kr_trackers_manager::Tracker
{
 public:
  LineTrackerMinJerk(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);

  uint8_t status() const;

 private:
  void goal_callback();

  void preempt_callback();

  void gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf, const Eigen::Vector3f &vi,
                      const Eigen::Vector3f &vf, const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                      const float &yawi, const float &yawf, const float &yaw_dot_i, const float &yaw_dot_f, float dt,
                      Eigen::Vector3f coeffs[6], float yaw_coeffs[4]);

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::LineTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::shared_ptr<ServerType> tracker_server_;

  Eigen::Vector3f current_pos_;

  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, default_yaw_v_des_, default_yaw_a_des_;
  float v_des_, a_des_, yaw_v_des_, yaw_a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f goal_;
  ros::Time traj_start_;
  float traj_duration_;
  ros::Duration goal_duration_;
  Eigen::Vector3f coeffs_[6];
  float goal_yaw_, yaw_coeffs_[4];
  bool traj_start_set_;

  float current_traj_length_;
};

LineTrackerMinJerk::LineTrackerMinJerk(void)
    : pos_set_(false),
      goal_set_(false),
      goal_reached_(true),
      active_(false),
      traj_start_(ros::Time::now()),
      traj_start_set_(false)
{
}

void LineTrackerMinJerk::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "line_tracker_min_jerk");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.3);
  priv_nh.param("default_yaw_v_des", default_yaw_v_des_, 0.8);
  priv_nh.param("default_yaw_a_des", default_yaw_a_des_, 0.2);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;
  yaw_v_des_ = default_yaw_v_des_;
  yaw_a_des_ = default_yaw_a_des_;

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LineTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&LineTrackerMinJerk::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LineTrackerMinJerk::preempt_callback, this));

  tracker_server_->start();
}

bool LineTrackerMinJerk::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    if(!tracker_server_->isActive())
    {
      ROS_WARN("LineTrackerMinJerk::Activate: goal_set_ is true but action server has no active goal - not "
               "activating.");
      active_ = false;
      return false;
    }
    active_ = true;

    current_traj_length_ = 0.0;
  }
  return active_;
}

void LineTrackerMinJerk::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("LineTrackerMinJerk::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr LineTrackerMinJerk::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Record distance between last position and current.
  const float dx =
      Eigen::Vector3f((current_pos_(0) - msg->pose.pose.position.x), (current_pos_(1) - msg->pose.pose.position.y),
                      (current_pos_(2) - msg->pose.pose.position.z))
          .norm();

  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;

  pos_set_ = true;
  ICs_.set_from_odom(msg);

  const ros::Time t_now = ros::Time::now();

  if(!active_)
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  current_traj_length_ += dx;

  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  if(goal_set_)
  {
    if(!traj_start_set_)
      traj_start_ = t_now;

    bool duration_set = false;
    traj_duration_ = 0.5f;

    // TODO: This should probably be after traj_duration_ is determined
    if(goal_duration_.toSec() > traj_duration_)
    {
      traj_duration_ = goal_duration_.toSec();
      duration_set = true;
    }

    // Min-Jerk trajectory
    const float total_dist = (goal_ - ICs_.pos()).norm();
    const Eigen::Vector3f dir = (goal_ - ICs_.pos()) / total_dist;
    const float vel_proj = (ICs_.vel()).dot(dir);

    const float t_ramp = (v_des_ - vel_proj) / a_des_;

    const float distance_to_v_des = vel_proj * t_ramp + 0.5f * a_des_ * t_ramp * t_ramp;
    const float distance_v_des_to_stop = 0.5f * v_des_ * v_des_ / a_des_;

    const float ramping_distance = distance_to_v_des + distance_v_des_to_stop;

    if(!duration_set)  // If duration is not set by the goal callback
    {
      if(total_dist > ramping_distance)
      {
        float t = (v_des_ - vel_proj) / a_des_                // Ramp up
                  + (total_dist - ramping_distance) / v_des_  // Constant velocity
                  + v_des_ / a_des_;                          // Ramp down

        traj_duration_ = std::max(traj_duration_, t);
      }
      else
      {
        // In this case, v_des_ is not reached. Assume bang bang acceleration.

        float vo = vel_proj;
        float distance_to_stop = 0.5f * vo * vo / a_des_;

        float t_dir;  // The time required for the component along dir
        if(vo > 0.0f && total_dist < distance_to_stop)
        {
          // Currently traveling towards the goal and need to overshoot

          t_dir = vo / a_des_ + std::sqrt(2.0f) * std::sqrt(vo * vo - 2.0f * a_des_ * total_dist) / a_des_;
        }
        else
        {
          // Ramp up to a velocity towards the goal before ramping down

          t_dir = -vo / a_des_ + std::sqrt(2.0f) * std::sqrt(vo * vo + 2.0f * a_des_ * total_dist) / a_des_;
        }
        traj_duration_ = std::max(traj_duration_, t_dir);

        // The velocity component orthogonal to dir
        float v_ortho = (ICs_.vel() - dir * vo).norm();
        float t_non_dir = v_ortho / a_des_                       // Ramp to zero velocity
                          + std::sqrt(2.0f) * v_ortho / a_des_;  // Get back to the dir line

        traj_duration_ = std::max(traj_duration_, t_non_dir);
      }
    }

    // Find shortest angle and direction to go from yaw_ to goal_yaw_
    float yaw_dist, yaw_dir;
    yaw_dist = goal_yaw_ - ICs_.yaw();
    const float pi(M_PI);  // Defined so as to force float type
    yaw_dist = std::fmod(yaw_dist, 2 * pi);
    if(yaw_dist > pi)
      yaw_dist -= 2 * pi;
    else if(yaw_dist < -pi)
      yaw_dist += 2 * pi;
    yaw_dir = (yaw_dist >= 0) ? 1 : -1;
    yaw_dist = std::abs(yaw_dist);
    goal_yaw_ = ICs_.yaw() + yaw_dir * yaw_dist;

    // Consider yaw in the trajectory duration
    if(!duration_set)  // Only if duration is not set from goal
    {
      if(yaw_dist > yaw_v_des_ * yaw_v_des_ / yaw_a_des_)
        traj_duration_ = std::max(traj_duration_, yaw_dist / yaw_v_des_ + yaw_v_des_ / yaw_a_des_);
      else
        traj_duration_ = std::max(traj_duration_, 2 * std::sqrt(yaw_dist / yaw_a_des_));
    }

    // TODO: Should consider yaw_dot_. See hover() in MAVManager

    gen_trajectory(ICs_.pos(), goal_, ICs_.vel(), Eigen::Vector3f::Zero(), ICs_.acc(), Eigen::Vector3f::Zero(),
                   ICs_.yaw(), goal_yaw_, ICs_.yaw_dot(), 0, traj_duration_, coeffs_, yaw_coeffs_);

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    if(tracker_server_->isActive())
    {
      ROS_ERROR("LineTrackerDistance::update: Action server not completed.\n");
    }

    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->yaw = goal_yaw_;
    cmd->yaw_dot = 0;
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  Eigen::Vector3f x(ICs_.pos()), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero()), j(Eigen::Vector3f::Zero());
  float yaw_des(ICs_.yaw()), yaw_dot_des(0);

  const float traj_time = (t_now - traj_start_).toSec();

  if(traj_time >= traj_duration_)  // Reached goal
  {
    // Send a success message and reset the length and duration variables.
    kr_tracker_msgs::LineTrackerResult result;
    result.duration = traj_time;
    result.length = current_traj_length_;
    result.x = goal_(0);
    result.y = goal_(1);
    result.z = goal_(2);
    result.yaw = goal_yaw_;

    tracker_server_->setSucceeded(result);

    current_traj_length_ = 0.0;

    ROS_DEBUG_THROTTLE(1, "Reached goal");
    j = Eigen::Vector3f::Zero();
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    yaw_des = goal_yaw_;
    yaw_dot_des = 0;
    goal_reached_ = true;
  }
  else if(traj_time >= 0)
  {
    float t = traj_time / traj_duration_, t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    x = coeffs_[0] + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] + t4 * coeffs_[4] + t5 * coeffs_[5];
    v = coeffs_[1] + 2 * t * coeffs_[2] + 3 * t2 * coeffs_[3] + 4 * t3 * coeffs_[4] + 5 * t4 * coeffs_[5];
    a = 2 * coeffs_[2] + 6 * t * coeffs_[3] + 12 * t2 * coeffs_[4] + 20 * t3 * coeffs_[5];
    j = 6 * coeffs_[3] + 24 * t * coeffs_[4] + 60 * t2 * coeffs_[5];

    yaw_des = yaw_coeffs_[0] + t * yaw_coeffs_[1] + t2 * yaw_coeffs_[2] + t3 * yaw_coeffs_[3];
    yaw_dot_des = yaw_coeffs_[1] + 2 * t * yaw_coeffs_[2] + 3 * t2 * yaw_coeffs_[3];

    // Scale based on the trajectory duration
    v = v / traj_duration_;
    a = a / (traj_duration_ * traj_duration_);
    j = j / (traj_duration_ * traj_duration_ * traj_duration_);
    yaw_dot_des = yaw_dot_des / traj_duration_;
  }
  else  // (traj_time < 0) can happen when t_start is set
    ROS_INFO_THROTTLE(1, "Trajectory hasn't started yet");

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->yaw = yaw_des;
  cmd->yaw_dot = yaw_dot_des;
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  cmd->jerk.x = j(0), cmd->jerk.y = j(1), cmd->jerk.z = j(2);

  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    kr_tracker_msgs::LineTrackerFeedback feedback;
    feedback.distance_from_goal = (current_pos_ - goal_).norm();
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void LineTrackerMinJerk::goal_callback()
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("LineTrackerMinJerk goal (%f, %f, %f) aborted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setAborted();
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  current_traj_length_ = 0.0;

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("LineTrackerMinJerk going to goal (%f, %f, %f, %f) preempted.", msg->x, msg->y, msg->z, msg->yaw);
    tracker_server_->setPreempted();
    return;
  }

  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
  goal_yaw_ = msg->yaw;
  goal_duration_ = msg->duration;
  if(msg->t_start != ros::Time(0))
  {
    traj_start_ = msg->t_start;
    traj_start_set_ = true;
  }
  else
    traj_start_set_ = false;

  if(msg->relative)
  {
    goal_ += ICs_.pos();
    goal_yaw_ += ICs_.yaw();
    ROS_INFO("line_tracker_min_jerk using relative command");
  }

  v_des_ = (msg->v_des > 0.0) ? msg->v_des : default_v_des_;
  a_des_ = (msg->a_des > 0.0) ? msg->a_des : default_a_des_;

  ROS_DEBUG("line_tracker_min_jerk using v_des = %2.2f m/s and a_des = %2.2f m/s^2", v_des_, a_des_);

  goal_set_ = true;
  goal_reached_ = false;
}

void LineTrackerMinJerk::preempt_callback()
{
  if(tracker_server_->isActive())
  {
    ROS_INFO("LineTrackerMinJerk going to goal (%f, %f, %f) aborted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("LineTrackerMinJerk going to goal (%f, %f, %f) preempted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setPreempted();
  }

  // TODO: How much overshoot will this cause at high velocities?
  goal_ = ICs_.pos();

  goal_set_ = false;
  goal_reached_ = true;
}

void LineTrackerMinJerk::gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf, const Eigen::Vector3f &vi,
                                        const Eigen::Vector3f &vf, const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                                        const float &yawi, const float &yawf, const float &yaw_dot_i,
                                        const float &yaw_dot_f, float dt, Eigen::Vector3f coeffs[6],
                                        float yaw_coeffs[4])
{
  // We can use a dt of 1 to ensure that our system will be numerically conditioned.
  // For more information, see line_tracker_min_jerk_numerical_issues.m

  Eigen::Matrix<float, 6, 6> Ainv;
  Ainv << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, -10, 10, -6, -4, -1.5, 0.5, 15, -15, 8, 7, 1.5, -1,
      -6, 6, -3, -3, -0.5, 0.5;

  Eigen::Matrix<float, 6, 3> b;
  b << xi.transpose(), xf.transpose(), dt * vi.transpose(), dt * vf.transpose(), dt * dt * ai.transpose(),
      dt * dt * af.transpose();

  Eigen::Matrix<float, 6, 3> x;
  x = Ainv * b;
  for(int i = 0; i < 6; i++)
  {
    coeffs[i] = x.row(i).transpose();
  }

  // Compute the trajectory for the yaw
  Eigen::Matrix<float, 4, 4> A_yaw_inv;
  A_yaw_inv << 1, 0, 0, 0, 0, 0, 1, 0, -3, 3, -2, -1, 2, -2, 1, 1;

  Eigen::Vector4f b_yaw;
  b_yaw << yawi, yawf, dt * yaw_dot_i, dt * yaw_dot_f;

  Eigen::Vector4f x_yaw;
  x_yaw = A_yaw_inv * b_yaw;
  for(int i = 0; i < 4; i++)
  {
    yaw_coeffs[i] = x_yaw(i);
  }
}

uint8_t LineTrackerMinJerk::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerMinJerk, kr_trackers_manager::Tracker);
