#include "kr_trackers/Tracker.hpp"
#include "kr_trackers/initial_conditions.hpp"
#include "kr_trackers/lissajous_generator.h"

#include "geometry_msgs/msg/point.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/lissajous_tracker.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <mutex>

class LissajousTracker : public kr_trackers_manager::Tracker
{
 public:
  LissajousTracker() = default;

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) override;
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) override;
  void Deactivate() override;

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg) override;
  uint8_t status() override;

 private:
  using LissajousTrackerAction = kr_tracker_msgs::action::LissajousTracker;
  using LissajousTrackerGoalHandle = rclcpp_action::ServerGoalHandle<LissajousTrackerAction>;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const LissajousTrackerAction::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<LissajousTrackerGoalHandle> goal_handle);
  void handle_accepted_callback(const std::shared_ptr<LissajousTrackerGoalHandle> goal_handle);

  rclcpp::Logger logger_{rclcpp::get_logger("trackers_manager")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<LissajousTrackerAction>::SharedPtr tracker_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<LissajousTrackerGoalHandle> current_goal_handle_;
  std::recursive_mutex mutex_;

  InitialConditions ICs_;
  LissajousGenerator generator_;
  double distance_traveled_{0.0};
  Eigen::Vector3d position_last_{Eigen::Vector3d::Zero()};
  bool traj_start_set_{false};
  bool active_{false};
  std::string frame_id_{"odom"};
};

void LissajousTracker::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter("lissajous_tracker/frame_id", "odom");
  frame_id_ = node->get_parameter("lissajous_tracker/frame_id").as_string();

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/lissajous_tracker/lissajous_path", 1);

  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tracker_server_ = rclcpp_action::create_server<LissajousTrackerAction>(
      node,
      "~/lissajous_tracker/LissajousTracker",
      std::bind(&LissajousTracker::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LissajousTracker::cancel_callback, this, std::placeholders::_1),
      std::bind(&LissajousTracker::handle_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

  RCLCPP_INFO(logger_, "Initialized LissajousTracker");
}

bool LissajousTracker::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  (void)cmd;
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(generator_.goalIsSet() && current_goal_handle_ && current_goal_handle_->is_active())
  {
    active_ = generator_.activate();
    return active_;
  }

  active_ = false;
  return false;
}

void LissajousTracker::Deactivate()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousTrackerAction::Result>();
    result->duration = generator_.timeElapsed();
    result->length = distance_traveled_;
    current_goal_handle_->abort(result);
    current_goal_handle_.reset();
  }

  ICs_.reset();
  generator_.deactivate();
  traj_start_set_ = false;
  active_ = false;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr LissajousTracker::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(!active_ || !generator_.isActive())
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  if(!traj_start_set_)
  {
    traj_start_set_ = true;
    ICs_.set_from_odom(msg);
    position_last_ = Eigen::Vector3d(ICs_.pos()(0), ICs_.pos()(1), ICs_.pos()(2));

    geometry_msgs::msg::Point initial_pt;
    initial_pt.x = ICs_.pos()(0);
    initial_pt.y = ICs_.pos()(1);
    initial_pt.z = ICs_.pos()(2);

    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = clock_->now();
    generator_.generatePath(path, initial_pt, 0.1);
    path_pub_->publish(path);
  }

  auto cmd = generator_.getPositionCmd();
  if(!cmd)
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  cmd->header.stamp = clock_->now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->position.x += ICs_.pos()(0);
  cmd->position.y += ICs_.pos()(1);
  cmd->position.z += ICs_.pos()(2);
  cmd->yaw += ICs_.yaw();

  if(!generator_.status())
  {
    if(current_goal_handle_ && current_goal_handle_->is_active())
    {
      auto feedback = std::make_shared<LissajousTrackerAction::Feedback>();
      feedback->time_to_completion = generator_.timeRemaining();
      current_goal_handle_->publish_feedback(feedback);
    }

    const Eigen::Vector3d position_current(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    distance_traveled_ += (position_current - position_last_).norm();
    position_last_ = position_current;
  }
  else if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousTrackerAction::Result>();
    result->x = msg->pose.pose.position.x;
    result->y = msg->pose.pose.position.y;
    result->z = msg->pose.pose.position.z;
    result->yaw = ICs_.yaw();
    result->duration = generator_.timeElapsed();
    result->length = distance_traveled_;
    current_goal_handle_->succeed(result);

    generator_.deactivate();
    active_ = false;
    current_goal_handle_.reset();
  }

  return cmd;
}

uint8_t LissajousTracker::status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(active_ && current_goal_handle_ && current_goal_handle_->is_active())
  {
    return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE);
  }
  return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

rclcpp_action::GoalResponse LissajousTracker::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LissajousTrackerAction::Goal> goal)
{
  (void)uuid;
  if(goal->period <= 0.0 || goal->num_cycles <= 0.0)
  {
    RCLCPP_WARN(logger_, "Rejecting Lissajous goal with non-positive period/num_cycles");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LissajousTracker::cancel_callback(
    const std::shared_ptr<LissajousTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ == goal_handle)
  {
    auto result = std::make_shared<LissajousTrackerAction::Result>();
    result->duration = generator_.timeElapsed();
    result->length = distance_traveled_;
    goal_handle->canceled(result);

    generator_.deactivate();
    active_ = false;
    traj_start_set_ = false;
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void LissajousTracker::handle_accepted_callback(const std::shared_ptr<LissajousTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousTrackerAction::Result>();
    result->duration = generator_.timeElapsed();
    result->length = distance_traveled_;
    current_goal_handle_->abort(result);
  }

  current_goal_handle_ = goal_handle;
  generator_.setParams(goal_handle->get_goal());
  traj_start_set_ = false;
  distance_traveled_ = 0.0;
  active_ = false;
}

PLUGINLIB_EXPORT_CLASS(LissajousTracker, kr_trackers_manager::Tracker);
