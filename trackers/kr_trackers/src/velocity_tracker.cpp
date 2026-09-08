// ROS2 port of VelocityTracker

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/msg/velocity_goal.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_trackers/Tracker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>

class VelocityTracker : public kr_trackers_manager::Tracker
{
 public:
  VelocityTracker(void);

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent);
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) override;
  void Deactivate(void) override;

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg) override;
  uint8_t status() override;

 private:
  void velocity_cmd_cb(const kr_tracker_msgs::msg::VelocityGoal::SharedPtr msg);

  rclcpp::Subscription<kr_tracker_msgs::msg::VelocityGoal>::SharedPtr sub_vel_cmd_;
  kr_mav_msgs::msg::PositionCommand position_cmd_;
  bool odom_set_, active_, use_position_gains_;
  double last_t_;
  double pos_[3], cur_yaw_;
  rclcpp::Time last_cmd_time_;

  double timeout_;

  // ROS2 context
  rclcpp::Logger logger_{rclcpp::get_logger("velocity_tracker")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

VelocityTracker::VelocityTracker(void)
  : odom_set_(false), active_(false), use_position_gains_(false), last_t_(0), timeout_(0.5)
{
}

void VelocityTracker::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  if(!node) return;
  node_ = node;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // declare/read parameter
  node_->declare_parameter("timeout", 0.5);
  timeout_ = node_->get_parameter("timeout").as_double();

  // subscribe to the goal (private namespace)
  sub_vel_cmd_ = node_->create_subscription<kr_tracker_msgs::msg::VelocityGoal>(
    "~/velocity_tracker/goal", 10, std::bind(&VelocityTracker::velocity_cmd_cb, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "VelocityTracker initialized (timeout=%f)", timeout_);
}

bool VelocityTracker::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  if(cmd)
  {
    position_cmd_.position = cmd->position;
    position_cmd_.yaw = cmd->yaw;
    active_ = true;
  }
  else if(odom_set_)
  {
    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
    position_cmd_.yaw = cur_yaw_;
    active_ = true;
  }
  return active_;
}

void VelocityTracker::Deactivate(void)
{
  active_ = false;
  odom_set_ = false;
  last_t_ = 0;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr VelocityTracker::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;
  pos_[2] = msg->pose.pose.position.z;
  cur_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  odom_set_ = true;

  if(!active_)
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();

  // check timeout against last_cmd_time_
  if((node_->now().seconds() - last_cmd_time_.seconds()) > timeout_)
  {
    position_cmd_.velocity.x = 0.0;
    position_cmd_.velocity.y = 0.0;
    position_cmd_.velocity.z = 0.0;
    position_cmd_.yaw_dot = 0.0;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000.0, "VelocityTracker is active but timed out");
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000.0, "Time since last command: %f seconds",
      (node_->now().seconds() - last_cmd_time_.seconds()));

    if(use_position_gains_)
      position_cmd_.use_msg_gains_flags = kr_mav_msgs::msg::PositionCommand::USE_MSG_GAINS_NONE;

    position_cmd_.header.stamp = msg->header.stamp;
    position_cmd_.header.frame_id = msg->header.frame_id;
    last_t_ = 0;
    return std::make_shared<kr_mav_msgs::msg::PositionCommand>(position_cmd_);
  }

  if(last_t_ == 0)
    last_t_ = node_->now().seconds();

  const double t_now = node_->now().seconds();
  const double dt = t_now - last_t_;
  last_t_ = t_now;

  use_position_gains_ = true;

  if(use_position_gains_)
  {
    position_cmd_.use_msg_gains_flags = kr_mav_msgs::msg::PositionCommand::USE_MSG_GAINS_NONE;
    position_cmd_.position.x = position_cmd_.position.x + dt * position_cmd_.velocity.x;
    position_cmd_.position.y = position_cmd_.position.y + dt * position_cmd_.velocity.y;
    position_cmd_.position.z = position_cmd_.position.z + dt * position_cmd_.velocity.z;
  }
  else
  {
    position_cmd_.kx[0] = 0; position_cmd_.kx[1] = 0; position_cmd_.kx[2] = 0;
    position_cmd_.use_msg_gains_flags = kr_mav_msgs::msg::PositionCommand::USE_MSG_GAINS_POSITION_ALL;
    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
  }
  position_cmd_.yaw = position_cmd_.yaw + dt * position_cmd_.yaw_dot;

  position_cmd_.header.stamp = msg->header.stamp;
  position_cmd_.header.frame_id = msg->header.frame_id;
  
  auto position_cmd = position_cmd_;

  if (position_cmd.yaw_dot == 0.0f)
  {
    const float target_yaw = position_cmd_.yaw;
    const float yaw_error = std::atan2(std::sin(target_yaw - cur_yaw_), std::cos(target_yaw - cur_yaw_));
    const float max_yaw_rate = 0.3f;
    const float kp_yaw = 1.0f; // TODO make config

    float yaw_rate = kp_yaw * yaw_error;
    yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_rate)); // clamp
    position_cmd.yaw_dot = yaw_rate;
  }

  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000.0, "VelocityTracker dt: %f", dt);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000.0, "VelocityTracker commanded vel: (%f, %f, %f), yaw_dot=%f",
    position_cmd.velocity.x, position_cmd.velocity.y, position_cmd.velocity.z, position_cmd.yaw_dot);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000.0, "VelocityTracker position command: pos=(%f, %f, %f), yaw=%f",
    position_cmd.position.x, position_cmd.position.y, position_cmd.position.z, position_cmd.yaw);
  return std::make_shared<kr_mav_msgs::msg::PositionCommand>(position_cmd);
}

void VelocityTracker::velocity_cmd_cb(const kr_tracker_msgs::msg::VelocityGoal::SharedPtr msg)
{
  position_cmd_.velocity.x = msg->vx;
  position_cmd_.velocity.y = msg->vy;
  position_cmd_.velocity.z = msg->vz;
  position_cmd_.yaw_dot = msg->vyaw;

  use_position_gains_ = msg->use_position_gains;

  last_cmd_time_ = node_->now();
}

uint8_t VelocityTracker::status()
{
  return active_ ? static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE) :
                   static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(VelocityTracker, kr_trackers_manager::Tracker);
