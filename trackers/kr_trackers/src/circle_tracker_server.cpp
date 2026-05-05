/*
 * Trajectory tracker for elliptical/circular motion:
 * x(t) = [Ax*cos(2*pi*t/T), Ay*sin(2*pi*t/T), 0] + offset.
 */

#include "kr_trackers/Tracker.hpp"

#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/circle_tracker.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/utils.h"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <mutex>

class CircleTracker : public kr_trackers_manager::Tracker
{
 public:
  CircleTracker() = default;

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) override;
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) override;
  void Deactivate() override;

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg) override;
  uint8_t status() override;

 private:
  using CircleTrackerAction = kr_tracker_msgs::action::CircleTracker;
  using CircleTrackerGoalHandle = rclcpp_action::ServerGoalHandle<CircleTrackerAction>;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const CircleTrackerAction::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<CircleTrackerGoalHandle> goal_handle);
  void handle_accepted_callback(const std::shared_ptr<CircleTrackerGoalHandle> goal_handle);

  rclcpp::Logger logger_{rclcpp::get_logger("trackers_manager")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp_action::Server<CircleTrackerAction>::SharedPtr tracker_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<CircleTrackerGoalHandle> current_goal_handle_;
  std::recursive_mutex mutex_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_start_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_end_;

  bool active_{false};
  bool have_odom_{false};
  bool traj_started_{false};
  bool traj_completed_{false};

  rclcpp::Time traj_start_time_;

  float current_traj_length_{0.0f};
  float ax_{0.0f};
  float ay_{0.0f};
  float period_{1.0f};
  float traj_duration_{0.0f};
  float omega_{0.0f};
  float ramp_up_time_{2.0f};
  float ramp_down_time_{2.0f};

  Eigen::Vector3f offset_pos_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f final_pos_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f current_pos_{Eigen::Vector3f::Zero()};
  float current_yaw_{0.0f};
  float constant_yaw_{0.0f};
};

void CircleTracker::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter("circle_tracker/ramp_up_time", 2.0);
  ramp_up_time_ = static_cast<float>(node->get_parameter("circle_tracker/ramp_up_time").as_double());
  ramp_down_time_ = ramp_up_time_;

  pub_start_ = node->create_publisher<std_msgs::msg::Empty>("~/circle_tracker/traj_start", 10);
  pub_end_ = node->create_publisher<std_msgs::msg::Empty>("~/circle_tracker/traj_end", 10);

  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tracker_server_ = rclcpp_action::create_server<CircleTrackerAction>(
      node,
      "~/circle_tracker/CircleTracker",
      std::bind(&CircleTracker::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CircleTracker::cancel_callback, this, std::placeholders::_1),
      std::bind(&CircleTracker::handle_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

  RCLCPP_INFO(logger_, "Initialized CircleTracker");
}

bool CircleTracker::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  (void)cmd;
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(!have_odom_)
  {
    RCLCPP_WARN(logger_, "CircleTracker::Activate failed: no odometry yet.");
    active_ = false;
    return false;
  }

  if(!current_goal_handle_ || !current_goal_handle_->is_active())
  {
    RCLCPP_WARN(logger_, "CircleTracker::Activate failed: no active goal.");
    active_ = false;
    return false;
  }

  active_ = true;
  traj_started_ = false;
  traj_completed_ = false;
  current_traj_length_ = 0.0f;

  // Center trajectory on activation pose.
  offset_pos_ = current_pos_;
  constant_yaw_ = current_yaw_;
  return true;
}

void CircleTracker::Deactivate()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<CircleTrackerAction::Result>();
    result->duration = std::max(0.0f, static_cast<float>((clock_->now() - traj_start_time_).seconds()));
    result->length = current_traj_length_;
    current_goal_handle_->abort(result);
    current_goal_handle_.reset();
  }

  active_ = false;
  traj_started_ = false;
  traj_completed_ = false;
  current_traj_length_ = 0.0f;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr CircleTracker::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const float dx =
      Eigen::Vector3f((current_pos_(0) - msg->pose.pose.position.x), (current_pos_(1) - msg->pose.pose.position.y),
                      (current_pos_(2) - msg->pose.pose.position.z))
          .norm();

  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;
  current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  have_odom_ = true;

  if(!active_)
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  current_traj_length_ += dx;

  if(!traj_started_)
  {
    traj_start_time_ = clock_->now();
    traj_started_ = true;
    traj_completed_ = false;
    current_traj_length_ = 0.0f;
  }

  const float t = std::max(0.0f, static_cast<float>((clock_->now() - traj_start_time_).seconds()));
  const float t_eval = std::min(t, traj_duration_);

  const float ramp_t_up = std::max(0.0f, std::min(ramp_up_time_, traj_duration_));
  const float ramp_t_down = std::max(0.0f, std::min(ramp_down_time_, traj_duration_));
  float s = 1.0f;
  float s_dot = 0.0f;
  float s_ddot = 0.0f;
  float s_dddot = 0.0f;

  if(ramp_t_up > 1e-4f && t_eval < ramp_t_up)
  {
    const float tau = std::max(0.0f, std::min(1.0f, t_eval / ramp_t_up));
    const float tau2 = tau * tau;
    const float tau3 = tau2 * tau;
    const float tau4 = tau3 * tau;

    // smooth ramp up
    s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau4 * tau;
    s_dot = (30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4) / ramp_t_up;
    s_ddot = (60.0f * tau - 180.0f * tau2 + 120.0f * tau3) / (ramp_t_up * ramp_t_up);
    s_dddot = (60.0f - 360.0f * tau + 360.0f * tau2) / (ramp_t_up * ramp_t_up * ramp_t_up);
  }
  else if(ramp_t_down > 1e-4f && t_eval > (traj_duration_ - ramp_t_down))
  {
    const float t_remain = traj_duration_ - t_eval;
    const float tau = std::max(0.0f, std::min(1.0f, t_remain / ramp_t_down));
    const float tau2 = tau * tau;
    const float tau3 = tau2 * tau;
    const float tau4 = tau3 * tau;

    // smooth ramp down: s goes from 1.0 -> 0.0
    s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau4 * tau;
    s_dot = -(30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4) / ramp_t_down;
    s_ddot = -(60.0f * tau - 180.0f * tau2 + 120.0f * tau3) / (ramp_t_down * ramp_t_down);
    s_dddot = -(60.0f - 360.0f * tau + 360.0f * tau2) / (ramp_t_down * ramp_t_down * ramp_t_down);
  }

  const float theta = omega_ * t_eval;
  const float sin_t = std::sin(theta);
  const float cos_t = std::cos(theta);

  float pos_x = ax_ * s * cos_t;
  float pos_y = ay_ * s * sin_t;
  float pos_z = 0.0f;

  float vel_x = ax_ * (s_dot * cos_t - s * sin_t * omega_);
  float vel_y = ay_ * (s_dot * sin_t + s * cos_t * omega_);
  float vel_z = 0.0f;

  const float omega2 = omega_ * omega_;
  float acc_x = ax_ * (s_ddot * cos_t - 2.0f * s_dot * sin_t * omega_ - s * cos_t * omega2);
  float acc_y = ay_ * (s_ddot * sin_t + 2.0f * s_dot * cos_t * omega_ - s * sin_t * omega2);
  float acc_z = 0.0f;

  const float omega3 = omega2 * omega_;
  float jerk_x =
      ax_ * (s_dddot * cos_t - 3.0f * s_ddot * sin_t * omega_ - 3.0f * s_dot * cos_t * omega2 + s * sin_t * omega3);
  float jerk_y =
      ay_ * (s_dddot * sin_t + 3.0f * s_ddot * cos_t * omega_ - 3.0f * s_dot * sin_t * omega2 - s * cos_t * omega3);
  float jerk_z = 0.0f;

  if(t >= traj_duration_)
  {
    traj_completed_ = true;
    final_pos_ = Eigen::Vector3f(pos_x, pos_y, pos_z);

    pos_x = final_pos_(0);
    pos_y = final_pos_(1);
    pos_z = final_pos_(2);
    vel_x = 0.0f;
    vel_y = 0.0f;
    vel_z = 0.0f;
    acc_x = 0.0f;
    acc_y = 0.0f;
    acc_z = 0.0f;
    jerk_x = 0.0f;
    jerk_y = 0.0f;
    jerk_z = 0.0f;

    std_msgs::msg::Empty end_msg;
    pub_end_->publish(end_msg);
  }
  else
  {
    std_msgs::msg::Empty start_msg;
    pub_start_->publish(start_msg);
  }

  auto cmd = std::make_shared<kr_mav_msgs::msg::PositionCommand>();
  cmd->header.stamp = clock_->now();
  cmd->header.frame_id = msg->header.frame_id;

  cmd->position.x = pos_x + offset_pos_(0);
  cmd->position.y = pos_y + offset_pos_(1);
  cmd->position.z = pos_z + offset_pos_(2);
  cmd->yaw = constant_yaw_;

  cmd->velocity.x = vel_x;
  cmd->velocity.y = vel_y;
  cmd->velocity.z = vel_z;
  cmd->yaw_dot = 0.0f;

  cmd->acceleration.x = acc_x;
  cmd->acceleration.y = acc_y;
  cmd->acceleration.z = acc_z;

  cmd->jerk.x = jerk_x;
  cmd->jerk.y = jerk_y;
  cmd->jerk.z = jerk_z;

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    if(traj_completed_)
    {
      auto result = std::make_shared<CircleTrackerAction::Result>();
      result->duration = t;
      result->length = current_traj_length_;
      current_goal_handle_->succeed(result);

      active_ = false;
      current_goal_handle_.reset();
      current_traj_length_ = 0.0f;
    }
    else
    {
      auto feedback = std::make_shared<CircleTrackerAction::Feedback>();
      feedback->duration = t;
      current_goal_handle_->publish_feedback(feedback);
    }
  }

  return cmd;
}

uint8_t CircleTracker::status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(active_ && current_goal_handle_ && current_goal_handle_->is_active())
  {
    return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE);
  }
  return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

rclcpp_action::GoalResponse CircleTracker::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CircleTrackerAction::Goal> goal)
{
  (void)uuid;

  if(goal->t <= 0.0 || goal->duration <= 0.0)
  {
    RCLCPP_WARN(logger_, "Rejecting CircleTracker goal with non-positive period/duration");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CircleTracker::cancel_callback(const std::shared_ptr<CircleTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ == goal_handle)
  {
    auto result = std::make_shared<CircleTrackerAction::Result>();
    result->duration = std::max(0.0f, static_cast<float>((clock_->now() - traj_start_time_).seconds()));
    result->length = current_traj_length_;
    goal_handle->canceled(result);

    active_ = false;
    traj_started_ = false;
    traj_completed_ = true;
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void CircleTracker::handle_accepted_callback(const std::shared_ptr<CircleTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<CircleTrackerAction::Result>();
    result->duration = std::max(0.0f, static_cast<float>((clock_->now() - traj_start_time_).seconds()));
    result->length = current_traj_length_;
    current_goal_handle_->abort(result);
  }

  const auto goal = goal_handle->get_goal();
  ax_ = static_cast<float>(goal->ax);
  ay_ = static_cast<float>(goal->ay);
  period_ = static_cast<float>(goal->t);
  traj_duration_ = static_cast<float>(goal->duration);
  omega_ = static_cast<float>(2.0 * M_PI / period_);

  // Use ramp_time from goal if provided (> 0), otherwise keep the parameter default
  if(goal->ramp_time > 0.0)
  {
    ramp_up_time_ = static_cast<float>(goal->ramp_time);
    ramp_down_time_ = ramp_up_time_;
  }

  current_goal_handle_ = goal_handle;
  traj_started_ = false;
  traj_completed_ = false;
  active_ = false;
}

PLUGINLIB_EXPORT_CLASS(CircleTracker, kr_trackers_manager::Tracker);
