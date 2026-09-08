#include "rclcpp_action/rclcpp_action.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/line_tracker.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_trackers/initial_conditions.hpp"
#include "kr_trackers/Tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Geometry>
#include <memory>

class LineTrackerDistance : public kr_trackers_manager::Tracker
{
 public:
  explicit LineTrackerDistance();

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

  rclcpp_action::Server<LineTracker>::SharedPtr tracker_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<LineTrackerGoalHandle> current_goal_handle_;
  rclcpp_action::GoalUUID preempted_goal_id_;
  std::recursive_mutex mutex_;

  bool preempt_requested_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f start_, goal_, pos_;
  float yaw_, start_yaw_;

  // Time taken to get to the goal.
  float current_traj_duration_;
  // Distance traveled to get to last goal.
  float current_traj_length_;

  std::shared_ptr<kr_tracker_msgs::action::LineTracker::Result> result_ = std::make_shared<kr_tracker_msgs::action::LineTracker::Result>();
};

LineTrackerDistance::LineTrackerDistance() 
  : preempt_requested_(false),
    pos_set_(false), 
    goal_set_(false), 
    goal_reached_(true), 
    active_(false)
{
}

void LineTrackerDistance::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter("line_tracker_distance/default_v_des", 0.5);
  node->declare_parameter("line_tracker_distance/default_a_des", 0.5);
  node->declare_parameter("line_tracker_distance/epsilon", 0.1);

  default_v_des_ = node->get_parameter("line_tracker_distance/default_v_des").as_double();
  default_a_des_ = node->get_parameter("line_tracker_distance/default_a_des").as_double();
  epsilon_ = node->get_parameter("line_tracker_distance/epsilon").as_double();

  v_des_ = static_cast<float>(default_v_des_);
  a_des_ = static_cast<float>(default_a_des_);

  // Set up the action server
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // tracker_server_ = rclcpp_action::create_server<LineTracker>
  tracker_server_ = rclcpp_action::create_server<LineTracker>(
    node,
    "~/line_tracker_distance/LineTracker",
    std::bind(&LineTrackerDistance::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LineTrackerDistance::cancel_callback, this, std::placeholders::_1),
    std::bind(&LineTrackerDistance::handle_accepted_callback, this, std::placeholders::_1), 
    rcl_action_server_get_default_options(),
    cb_group_
    );

  RCLCPP_INFO(logger_, "Initialized Line Tracker Distance");
}

bool LineTrackerDistance::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{ 
  RCLCPP_INFO(logger_, "In Activate 1");
  (void)cmd;
  RCLCPP_INFO_STREAM(logger_, "pos_set_: " << pos_set_ );
  RCLCPP_INFO_STREAM(logger_, "goal_set_: " << goal_set_ );
  if(goal_set_ && pos_set_)
  {
    RCLCPP_INFO(logger_, "In Activate 2");
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if(!current_goal_handle_ || !current_goal_handle_->is_active())
      {
        RCLCPP_WARN(logger_, "LineTrackerDistance::Activate: goal_set_ is true but action server has no active goal - not activating.");
        active_ = false;
        return active_;
      }
    }
    RCLCPP_INFO(logger_, "In Activate 3");

    // Set start and start_yaw here so that even if the goal was sent at a
    // different position, we still use the current position as start
    start_ = pos_;
    start_yaw_ = yaw_;

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    active_ = true;
  }
  RCLCPP_INFO(logger_, "In Activate 4");
  return active_;
}

void LineTrackerDistance::Deactivate(void)
{
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(current_goal_handle_ && current_goal_handle_->is_active())
    {
      RCLCPP_WARN(logger_, "LineTrackerDistance::Deactivate: deactivated tracker while still tracking the goal.");
      current_goal_handle_->abort(result_);
      current_goal_handle_.reset();
    }
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr LineTrackerDistance::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // Record distance between last position and current.
  const float dx = Eigen::Vector3f((pos_(0) - msg->pose.pose.position.x), (pos_(1) - msg->pose.pose.position.y),
                                   (pos_(2) - msg->pose.pose.position.z))
                       .norm();

  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  static rclcpp::Time t_prev = msg->header.stamp;
  const double dT = ((rclcpp::Time)msg->header.stamp - t_prev).seconds();
  t_prev = msg->header.stamp;

  if(!active_)
  {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  // Track the distance and time in the current trajectory.
  current_traj_duration_ += dT;
  current_traj_length_ += dx;

  auto result = std::make_shared<LineTracker::Result>();
  result->x = pos_(0);
  result->y = pos_(1);
  result->z = pos_(2);
  result->yaw = yaw_;
  result->duration = current_traj_duration_;
  result->length = current_traj_length_;
  result_ = result;

  if (current_goal_handle_) {
    if(current_goal_handle_ && current_goal_handle_->is_canceling())
    {
      RCLCPP_INFO(logger_, "LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) canceled.", goal_(0), goal_(1), goal_(2));
    
      current_goal_handle_->canceled(result);
      goal_ = pos_;
      goal_set_ = false;
      goal_reached_ = false;
      return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
    }
  }

  auto cmd = std::make_shared<kr_mav_msgs::msg::PositionCommand>();
  cmd->header.stamp = clock_->now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;

  if(goal_reached_)
  {
    // RCLCPP_INFO_STREAM(logger_, "current_goal_handle: "<< current_goal_handle_);
    if (current_goal_handle_) {
      if(current_goal_handle_->is_active())
      {
        RCLCPP_ERROR(logger_, "LineTrackerDistance::update: Action server not completed.\n");
      }
    }

    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    // patch to track yaw while at goal
    const float target_yaw = start_yaw_;
    const float yaw_error = std::atan2(std::sin(target_yaw - yaw_), std::cos(target_yaw - yaw_));
    const float max_yaw_rate = 0.3f;
    const float kp_yaw = 1.0f; // TODO make config

    float yaw_rate = kp_yaw * yaw_error;
    yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_rate)); //clamp
    cmd->yaw = target_yaw;
    cmd->yaw_dot = yaw_rate;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_ -  start_).normalized();
  const float total_dist = (goal_ - start_).norm();
  const float d = (pos_ - start_).dot(dir);
  const Eigen::Vector3f proj = start_ + d * dir;

  const float v_max = std::min(std::sqrt(a_des_ * total_dist), v_des_);
  const float ramp_dist = v_max * v_max / (2 * a_des_);

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());


  if((pos_ - goal_).norm() <= epsilon_) // Reached goal
  {
    // Send success message and reset the length and duration variables
    result->duration = current_traj_duration_;
    result->length = current_traj_length_;
    result->x = goal_(0);
    result->y = goal_(1);
    result->z = goal_(2);
    result->yaw = start_yaw_;
    current_goal_handle_->succeed(result);
    result_ = result;
    current_goal_handle_.reset(); 

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(d > total_dist)  // Overshoot
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Overshoot");
    a = -a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = goal_;
  }
  else if(d >= (total_dist - ramp_dist) && d <= total_dist) // Decelerate
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Decelerate");
    a = -a_des_ * dir;
    v = std::sqrt(2 * a_des_ * (total_dist - d)) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if(d > ramp_dist && d < total_dist - ramp_dist) // Constant velocity
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max * dir;
    x = proj + v * dT;
  }
  else if(d >= 0 && d <= ramp_dist) // Accelerate
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Accelerate");
    a = a_des_ * dir;
    v = std::sqrt(2 * a_des_ * d) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if(d < 0)
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "Undershoot");
    a = a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = start_ + 0.5 * a * dT * dT;
  }
  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    auto feedback = std::make_shared<LineTracker::Feedback>();
    feedback->distance_from_goal = (pos_ - goal_).norm();
    current_goal_handle_->publish_feedback(feedback);
  }

  return cmd;
}


rclcpp_action::GoalResponse LineTrackerDistance::goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LineTracker::Goal> goal)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO_STREAM(logger_, "current_goal_handle_: " << current_goal_handle_);
  
  if (current_goal_handle_ == 0) {
    RCLCPP_INFO_STREAM(logger_, "Initialization of line tracker detected");
    goal_set_ = false;
  }
  
  if(current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_INFO(logger_, "LineTrackerDistance goal (%2.2f, %2.2f, %2.2f) preempted.", goal_(0), goal_(1), goal_(2));
    preempted_goal_id_ = current_goal_handle_->get_goal_id();
    preempt_requested_ = true;
    goal_ = pos_;
    goal_set_ = false;
    goal_reached_ = false;
  }
  
  // Moving the functionality from handle_accepted_callback to here
  if(current_goal_handle_ && preempt_requested_) {
    if(current_goal_handle_->get_goal_id() == preempted_goal_id_) {
      RCLCPP_INFO(logger_, "LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) aborted.", goal_(0), goal_(1), goal_(2));
      current_goal_handle_->abort(result_);
      preempt_requested_ = false;
    }
  }
  
  // Set up the goal parameters
  goal_(0) = goal->x;
  goal_(1) = goal->y;
  goal_(2) = goal->z;
  
  if(goal->relative)
    goal_ += ICs_.pos();
  
  if(goal->v_des > 0)
    v_des_ = goal->v_des;
  else
    v_des_ = default_v_des_;
  
  if(goal->a_des > 0)
    a_des_ = goal->a_des;
  else
    a_des_ = default_a_des_;
  
  start_ = pos_;
  start_yaw_ = yaw_;
  current_traj_length_ = 0.0;
  current_traj_duration_ = 0.0;
  goal_set_ = true;
  goal_reached_ = false;
  
  RCLCPP_INFO(logger_, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LineTrackerDistance::cancel_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle)
{
  RCLCPP_INFO(logger_, "Received Cancel Request.");
  (void)goal_handle;
  goal_ = pos_;
  goal_set_ = false;
  goal_reached_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LineTrackerDistance::handle_accepted_callback(const std::shared_ptr<LineTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  current_goal_handle_ = goal_handle;
}

uint8_t LineTrackerDistance::status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (!current_goal_handle_) {
    return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
  }
  
  return current_goal_handle_->is_active() ? 
    static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE) : 
    static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(LineTrackerDistance, kr_trackers_manager::Tracker)
