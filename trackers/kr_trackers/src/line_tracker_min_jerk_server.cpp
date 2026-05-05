#include "rclcpp_action/rclcpp_action.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/line_tracker.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_trackers/initial_conditions.hpp"
#include "kr_trackers/Tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Core>
#include <memory>

class LineTrackerMinJerk : public kr_trackers_manager::Tracker
{
 public:
  LineTrackerMinJerk(void);

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent);
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd);
  void Deactivate(void);

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg);

  uint8_t status();

 protected:
  rclcpp::Logger logger_{rclcpp::get_logger("trackers_manager")};
  rclcpp::Clock::SharedPtr clock_;

  using LineTracker = kr_tracker_msgs::action::LineTracker;
  using LineTrackerGoalHandle = rclcpp_action::ServerGoalHandle<LineTracker>;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LineTracker::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle);
  void handle_accepted_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle);

  void gen_trajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf, const Eigen::Vector3f &vi,
                      const Eigen::Vector3f &vf, const Eigen::Vector3f &ai, const Eigen::Vector3f &af,
                      const float &yawi, const float &yawf, const float &yaw_dot_i, const float &yaw_dot_f, float dt,
                      Eigen::Vector3f coeffs[6], float yaw_coeffs[4]);

  rclcpp_action::Server<LineTracker>::SharedPtr tracker_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<LineTrackerGoalHandle> current_goal_handle_;
  rclcpp_action::GoalUUID preempted_goal_id_;
  std::recursive_mutex mutex_;

  Eigen::Vector3f current_pos_;

  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, default_yaw_v_des_, default_yaw_a_des_;
  float v_des_, a_des_, yaw_v_des_, yaw_a_des_;
  bool active_;
  bool preempt_requested_;

  InitialConditions ICs_;
  Eigen::Vector3f goal_;
  bool traj_start_set_;
  rclcpp::Time traj_start_;
  float traj_duration_;
  rclcpp::Duration goal_duration_ = rclcpp::Duration(0, 0);
  Eigen::Vector3f coeffs_[6];
  float goal_yaw_, yaw_coeffs_[4];

  float current_traj_length_;
  std::shared_ptr<kr_tracker_msgs::action::LineTracker::Result> result_;
};

LineTrackerMinJerk::LineTrackerMinJerk(void)
  : pos_set_(false),
    goal_set_(false),
    goal_reached_(true),
    active_(false),
    preempt_requested_(false),
    traj_start_set_(false)
{
}

void LineTrackerMinJerk::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter("line_tracker_min_jerk/default_v_des", 0.5);
  node->declare_parameter("line_tracker_min_jerk/default_a_des", 0.3);
  node->declare_parameter("line_tracker_min_jerk/default_yaw_v_des", 0.8);
  node->declare_parameter("line_tracker_min_jerk/default_yaw_a_des", 0.2);

  default_v_des_ = node->get_parameter("line_tracker_min_jerk/default_v_des").as_double();
  default_a_des_ = node->get_parameter("line_tracker_min_jerk/default_a_des").as_double();
  yaw_v_des_ = node->get_parameter("line_tracker_min_jerk/default_yaw_v_des").as_double();
  yaw_a_des_ = node->get_parameter("line_tracker_min_jerk/default_yaw_a_des").as_double();

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;
  yaw_v_des_ = default_yaw_v_des_;
  yaw_a_des_ = default_yaw_a_des_;
  
  traj_start_ = clock_->now();

  // Set up the action server.
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tracker_server_ = rclcpp_action::create_server<LineTracker>(
    node,
    "~/line_tracker_min_jerk/LineTracker",
    std::bind(&LineTrackerMinJerk::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LineTrackerMinJerk::cancel_callback, this, std::placeholders::_1),
    std::bind(&LineTrackerMinJerk::handle_accepted_callback, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    cb_group_
  );

  RCLCPP_INFO(logger_, "Initialized Line Tracker Min Jerk");
}

bool LineTrackerMinJerk::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  (void)cmd;

  RCLCPP_INFO(logger_, "LTMJ ACTIVATE 1");
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
  RCLCPP_INFO(logger_, "LTMJ ACTIVATE 2");
    {
      std::lock_guard<std::recursive_mutex> lock_(mutex_);
      if(!current_goal_handle_ || !current_goal_handle_->is_active())
      {
        RCLCPP_WARN(logger_, "LineTrackerMinJerk::Activate: goal_set_ is true but action server has no active goal - not activating.");
        active_ = false;
        return active_;
      }
      active_ = true;

  RCLCPP_INFO(logger_, "LTMJ ACTIVATE 3");
      current_traj_length_ = 0.0;
    }
  }
  RCLCPP_INFO(logger_, "LTMJ ACTIVATE 4");
  return active_;
}

void LineTrackerMinJerk::Deactivate(void)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_WARN(logger_, "LineTrackerMinJerk::Deactivate: deactivated tracker while still tracking the goal.");
    current_goal_handle_->abort(result_);
    current_goal_handle_.reset();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr LineTrackerMinJerk::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

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

  const rclcpp::Time t_now = clock_->now();
  
  if(!active_)
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  current_traj_length_ += dx;

  auto result = std::make_shared<LineTracker::Result>();
  result->x = current_pos_(0);
  result->y = current_pos_(1);
  result->z = current_pos_(2);
  result->yaw = ICs_.yaw();
  result->length = current_traj_length_;
  result->duration = (t_now - traj_start_).seconds();
  result_ = result;

  if(current_goal_handle_ && current_goal_handle_->is_canceling())
  {
    RCLCPP_INFO(logger_, "LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) canceled.", goal_(0), goal_(1), goal_(2));
    current_goal_handle_->canceled(result);
    goal_ = current_pos_;
    goal_set_ = false;
    goal_reached_ = false;
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  auto cmd = std::make_shared<kr_mav_msgs::msg::PositionCommand>();
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  if(goal_set_)
  {
    if(!traj_start_set_)
      traj_start_ = t_now;

    bool duration_set = false;
    RCLCPP_INFO(logger_, "setting traj_duration_ here first");
    traj_duration_ = 0.5f;
    RCLCPP_INFO_STREAM(logger_, "traj_duration: " << traj_duration_);

    // TODO: This should probably be after traj_duration_ is determined
    if(goal_duration_.seconds() > traj_duration_)
    {
      RCLCPP_INFO(logger_, "setting traj_duration_ here second");
      traj_duration_ = goal_duration_.seconds();
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

        RCLCPP_INFO(logger_, "setting traj_duration_ here third");
        traj_duration_ = std::max(traj_duration_, t);
        RCLCPP_INFO_STREAM(logger_, "traj_duration: " << traj_duration_);
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

        RCLCPP_INFO(logger_, "setting traj_duration_ here fourth");
        traj_duration_ = std::max(traj_duration_, t_dir);

        // The velocity component orthogonal to dir
        float v_ortho = (ICs_.vel() - dir * vo).norm();
        float t_non_dir = v_ortho / a_des_                       // Ramp to zero velocity
                          + std::sqrt(2.0f) * v_ortho / a_des_;  // Get back to the dir line

        RCLCPP_INFO(logger_, "setting traj_duration_ here 1");
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
      if(yaw_dist > yaw_v_des_ * yaw_v_des_ / yaw_a_des_) {
        RCLCPP_INFO(logger_, "setting traj_duration_ here 2");
        traj_duration_ = std::max(traj_duration_, yaw_dist / yaw_v_des_ + yaw_v_des_ / yaw_a_des_);
      } else {
        // HERE IS THE PROBLEM
        RCLCPP_INFO(logger_, "setting traj_duration_ here 3");
        if (yaw_a_des_ != 0) {
          traj_duration_ = std::max(traj_duration_, 2 * std::sqrt(yaw_dist / yaw_a_des_));
        }
        RCLCPP_INFO_STREAM(logger_, "traj_duration: " << traj_duration_);
        RCLCPP_INFO_STREAM(logger_, "yaw_dist: " << yaw_dist);
        RCLCPP_INFO_STREAM(logger_, "yaw_a_des: " << yaw_a_des_);
      }
    }

    // TODO: Should consider yaw_dot_. See hover() in MAVManager

    gen_trajectory(ICs_.pos(), goal_, ICs_.vel(), Eigen::Vector3f::Zero(), ICs_.acc(), Eigen::Vector3f::Zero(),
                   ICs_.yaw(), goal_yaw_, ICs_.yaw_dot(), 0, traj_duration_, coeffs_, yaw_coeffs_);

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    if(current_goal_handle_->is_active())
    {
      RCLCPP_ERROR(logger_, "LineTrackerMinJerk::update: Action server not completed.\n");
    }

    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    // patch to track yaw while at goal
    const float current_yaw = tf2::getYaw(msg->pose.pose.orientation);
    const float target_yaw = goal_yaw_;
    const float yaw_error = std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));
    const float max_yaw_rate = 0.3f;
    const float kp_yaw = 1.0f; // TODO make config
    
    float yaw_rate = kp_yaw * yaw_error;
    yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_rate)); // clamp
    cmd->yaw = target_yaw;
    cmd->yaw_dot = yaw_rate;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  Eigen::Vector3f x(ICs_.pos()), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero()), j(Eigen::Vector3f::Zero());
  float yaw_des(ICs_.yaw()), yaw_dot_des(0);

  const float traj_time = (t_now - traj_start_).seconds();

  if(traj_time >= traj_duration_)  // Reached goal
  {
    // Send a success message and reset the length and duration variables.
    result->duration = traj_time;
    result->length = current_traj_length_;
    result->x = goal_(0);
    result->y = goal_(1);
    result->z = goal_(2);
    result->yaw = goal_yaw_;

    current_goal_handle_->succeed(result);
    result_ = result;
    current_traj_length_ = 0.0;

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Reached goal");
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
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Trajectory hasn't started yet");

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->yaw = yaw_des;
  cmd->yaw_dot = yaw_dot_des;
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  cmd->jerk.x = j(0), cmd->jerk.y = j(1), cmd->jerk.z = j(2);

  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    auto feedback = std::make_shared<LineTracker::Feedback>();
    feedback->distance_from_goal = (current_pos_ - goal_).norm();
    current_goal_handle_->publish_feedback(feedback);
  }

  return cmd;
}

rclcpp_action::GoalResponse LineTrackerMinJerk::goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LineTracker::Goal> goal)
{
  (void)uuid;
  (void)goal;

  // If another goal is already active, cancel that goal
  // and track this one instead.
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_INFO(logger_, "LineTrackerMinJerk goal (%f, %f, %f) preempted.", goal_(0), goal_(1), goal_(2));
    preempted_goal_id_ = current_goal_handle_->get_goal_id();
    preempt_requested_ = true;

    goal_ = current_pos_;
    goal_set_ = false;
    goal_reached_ = false;
  }

  if(current_goal_handle_ && preempt_requested_)
  {
    if(current_goal_handle_->get_goal_id() == preempted_goal_id_)
    {
      RCLCPP_INFO(logger_, "LineTrackerMinJerk going to goal (%2.2f, %2.2f, %2.2f) aborted.", goal_(0), goal_(1), goal_(2));
      current_goal_handle_->abort(result_);
      preempt_requested_ = false;
    }
  }
  goal_(0) = goal->x;
  goal_(1) = goal->y;
  goal_(2) = goal->z;
  goal_yaw_ = goal->yaw;
  goal_duration_ = goal->duration;
  if(goal->t_start != rclcpp::Time())
  {
    traj_start_ = goal->t_start;
    traj_start_set_ = true;
  }
  else
    traj_start_set_ = false;

  if(goal->relative)
  {
    goal_ += ICs_.pos();
    goal_yaw_ += ICs_.yaw();
    RCLCPP_INFO(logger_, "line_tracker_min_jerk using relative command");
  }

  v_des_ = (goal->v_des > 0.0) ? goal->v_des : default_v_des_;
  a_des_ = (goal->a_des > 0.0) ? goal->a_des : default_a_des_;

  RCLCPP_INFO(logger_, "line_tracker_min_jerk using v_des = %2.2f m/s and a_des = %2.2f m/s^2", v_des_, a_des_);

  goal_set_ = true;
  goal_reached_ = false;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LineTrackerMinJerk::cancel_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle)
{
  RCLCPP_INFO(logger_, "Received Cancel Request.");
  (void)goal_handle;

  goal_ = ICs_.pos();
  goal_set_ = false;
  goal_reached_ = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void LineTrackerMinJerk::handle_accepted_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);


  // Pointer to the goal received
  current_goal_handle_ = goal_handle;
  // auto msg = goal_handle->get_goal();
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

uint8_t LineTrackerMinJerk::status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return current_goal_handle_->is_active() ? static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE) : static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(LineTrackerMinJerk, kr_trackers_manager::Tracker);
