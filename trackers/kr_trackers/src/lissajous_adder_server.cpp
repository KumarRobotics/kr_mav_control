#include "kr_trackers/Tracker.hpp"
#include "kr_trackers/initial_conditions.hpp"
#include "kr_trackers/lissajous_generator.h"

#include "geometry_msgs/msg/point.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/lissajous_adder.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <mutex>

class LissajousAdder : public kr_trackers_manager::Tracker
{
 public:
  LissajousAdder() = default;

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) override;
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) override;
  void Deactivate() override;

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg) override;
  uint8_t status() override;

 private:
  using LissajousAdderAction = kr_tracker_msgs::action::LissajousAdder;
  using LissajousAdderGoalHandle = rclcpp_action::ServerGoalHandle<LissajousAdderAction>;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const LissajousAdderAction::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<LissajousAdderGoalHandle> goal_handle);
  void handle_accepted_callback(const std::shared_ptr<LissajousAdderGoalHandle> goal_handle);

  rclcpp::Logger logger_{rclcpp::get_logger("trackers_manager")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<LissajousAdderAction>::SharedPtr tracker_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<LissajousAdderGoalHandle> current_goal_handle_;
  std::recursive_mutex mutex_;

  InitialConditions ICs_;
  LissajousGenerator generator_1_;
  LissajousGenerator generator_2_;
  double distance_traveled_{0.0};
  Eigen::Vector3d position_last_{Eigen::Vector3d::Zero()};
  bool traj_start_set_{false};
  bool active_{false};
  std::string frame_id_{"odom"};
};

void LissajousAdder::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter("lissajous_adder/frame_id", "odom");
  frame_id_ = node->get_parameter("lissajous_adder/frame_id").as_string();

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/lissajous_adder/lissajous_path", 1);

  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tracker_server_ = rclcpp_action::create_server<LissajousAdderAction>(
      node,
      "~/lissajous_adder/LissajousAdder",
      std::bind(&LissajousAdder::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LissajousAdder::cancel_callback, this, std::placeholders::_1),
      std::bind(&LissajousAdder::handle_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

  RCLCPP_INFO(logger_, "Initialized LissajousAdder");
}

bool LissajousAdder::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  (void)cmd;
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(generator_1_.goalIsSet() && generator_2_.goalIsSet() && current_goal_handle_ && current_goal_handle_->is_active())
  {
    active_ = generator_1_.activate() && generator_2_.activate();
    return active_;
  }

  active_ = false;
  return false;
}

void LissajousAdder::Deactivate()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousAdderAction::Result>();
    result->duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
    result->length = distance_traveled_;
    current_goal_handle_->abort(result);
    current_goal_handle_.reset();
  }

  ICs_.reset();
  generator_1_.deactivate();
  generator_2_.deactivate();
  traj_start_set_ = false;
  active_ = false;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr LissajousAdder::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(!active_ || !generator_1_.isActive() || !generator_2_.isActive())
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

    nav_msgs::msg::Path path1;
    nav_msgs::msg::Path path2;
    path1.header.frame_id = frame_id_;
    path1.header.stamp = clock_->now();

    generator_1_.generatePath(path1, initial_pt, 0.1);
    generator_2_.generatePath(path2, initial_pt, 0.1);

    const size_t count = std::min(path1.poses.size(), path2.poses.size());
    for(size_t i = 0; i < count; ++i)
    {
      path1.poses[i].pose.position.x += path2.poses[i].pose.position.x - initial_pt.x;
      path1.poses[i].pose.position.y += path2.poses[i].pose.position.y - initial_pt.y;
      path1.poses[i].pose.position.z += path2.poses[i].pose.position.z - initial_pt.z;
    }

    path_pub_->publish(path1);
  }

  auto cmd1 = generator_1_.getPositionCmd();
  auto cmd2 = generator_2_.getPositionCmd();
  if(!cmd1 || !cmd2)
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  cmd1->header.stamp = clock_->now();
  cmd1->header.frame_id = msg->header.frame_id;
  cmd1->position.x += ICs_.pos()(0) + cmd2->position.x;
  cmd1->position.y += ICs_.pos()(1) + cmd2->position.y;
  cmd1->position.z += ICs_.pos()(2) + cmd2->position.z;
  cmd1->velocity.x += cmd2->velocity.x;
  cmd1->velocity.y += cmd2->velocity.y;
  cmd1->velocity.z += cmd2->velocity.z;
  cmd1->acceleration.x += cmd2->acceleration.x;
  cmd1->acceleration.y += cmd2->acceleration.y;
  cmd1->acceleration.z += cmd2->acceleration.z;
  cmd1->jerk.x += cmd2->jerk.x;
  cmd1->jerk.y += cmd2->jerk.y;
  cmd1->jerk.z += cmd2->jerk.z;
  cmd1->yaw += ICs_.yaw() + cmd2->yaw;

  if(!generator_1_.status() || !generator_2_.status())
  {
    if(current_goal_handle_ && current_goal_handle_->is_active())
    {
      auto feedback = std::make_shared<LissajousAdderAction::Feedback>();
      feedback->time_to_completion = std::max(generator_1_.timeRemaining(), generator_2_.timeRemaining());
      current_goal_handle_->publish_feedback(feedback);
    }

    const Eigen::Vector3d position_current(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    distance_traveled_ += (position_current - position_last_).norm();
    position_last_ = position_current;
  }
  else if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousAdderAction::Result>();
    result->x = msg->pose.pose.position.x;
    result->y = msg->pose.pose.position.y;
    result->z = msg->pose.pose.position.z;
    result->yaw = ICs_.yaw();
    result->duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
    result->length = distance_traveled_;
    current_goal_handle_->succeed(result);

    generator_1_.deactivate();
    generator_2_.deactivate();
    active_ = false;
    current_goal_handle_.reset();
  }

  return cmd1;
}

uint8_t LissajousAdder::status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(active_ && current_goal_handle_ && current_goal_handle_->is_active())
  {
    return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE);
  }
  return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

rclcpp_action::GoalResponse LissajousAdder::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LissajousAdderAction::Goal> goal)
{
  (void)uuid;
  if(goal->period[0] <= 0.0 || goal->period[1] <= 0.0)
  {
    RCLCPP_WARN(logger_, "Rejecting LissajousAdder goal with non-positive period");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LissajousAdder::cancel_callback(
    const std::shared_ptr<LissajousAdderGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ == goal_handle)
  {
    auto result = std::make_shared<LissajousAdderAction::Result>();
    result->duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
    result->length = distance_traveled_;
    goal_handle->canceled(result);

    generator_1_.deactivate();
    generator_2_.deactivate();
    active_ = false;
    traj_start_set_ = false;
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void LissajousAdder::handle_accepted_callback(const std::shared_ptr<LissajousAdderGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    auto result = std::make_shared<LissajousAdderAction::Result>();
    result->duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
    result->length = distance_traveled_;
    current_goal_handle_->abort(result);
  }

  current_goal_handle_ = goal_handle;
  traj_start_set_ = false;
  distance_traveled_ = 0.0;
  generator_1_.setParams(goal_handle->get_goal(), 0);
  generator_2_.setParams(goal_handle->get_goal(), 1);
  active_ = false;
}

PLUGINLIB_EXPORT_CLASS(LissajousAdder, kr_trackers_manager::Tracker);
