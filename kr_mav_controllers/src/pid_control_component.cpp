#include <Eigen/Geometry>

#include "kr_mav_controllers/PIDControl.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_mav_msgs/msg/trpy_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define CLAMP(x, min, max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class PIDControlComponent : public rclcpp::Node
{
 public:
  PIDControlComponent(const rclcpp::NodeOptions &options);

 private:
  void publishTRPYCommand(void);
  void position_cmd_callback(const kr_mav_msgs::msg::PositionCommand::UniquePtr cmd);
  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom);
  void enable_motors_callback(const std_msgs::msg::Bool::UniquePtr msg);

  PIDControl controller_;
  rclcpp::Publisher<kr_mav_msgs::msg::TRPYCommand>::SharedPtr trpy_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<kr_mav_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, kx_, kv_, ki_;
  float des_yaw_;
  float current_yaw_;
  float ki_yaw_;
  float max_roll_pitch_;
  bool enable_motors_;
  bool use_external_yaw_;
};

void PIDControlComponent::publishTRPYCommand(void)
{
  float ki_yaw = 0;
  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  // Only enable integral terms when motors are on
  if(enable_motors_)
  {
    ki_yaw = ki_yaw_;
    ki = ki_;
  }
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, kx_, kv_, ki, ki_yaw);

  const Eigen::Vector4f &trpy = controller_.getControls();

  auto trpy_command = std::make_unique<kr_mav_msgs::msg::TRPYCommand>();
  trpy_command->header.stamp = this->now();
  trpy_command->header.frame_id = frame_id_;
  if(enable_motors_)
  {
    trpy_command->thrust = trpy(0);
    trpy_command->roll = CLAMP(trpy(1), -max_roll_pitch_, max_roll_pitch_);
    trpy_command->pitch = CLAMP(trpy(2), -max_roll_pitch_, max_roll_pitch_);
    trpy_command->yaw = trpy(3);
  }
  trpy_command->aux.current_yaw = current_yaw_;
  trpy_command->aux.enable_motors = enable_motors_;
  trpy_command->aux.use_external_yaw = use_external_yaw_;
  trpy_command_pub_->publish(std::move(trpy_command));
}

void PIDControlComponent::position_cmd_callback(const kr_mav_msgs::msg::PositionCommand::UniquePtr cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3f(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3f(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishTRPYCommand();
}

void PIDControlComponent::odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom)
{
  const Eigen::Vector3f position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

  current_yaw_ = tf2::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setYaw(current_yaw_);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishTRPYCommand();
    position_cmd_updated_ = false;
  }
}

void PIDControlComponent::enable_motors_callback(const std_msgs::msg::Bool::UniquePtr msg)
{
  if(msg->data)
    RCLCPP_INFO(this->get_logger(), "Enabling Motors");
  else
    RCLCPP_INFO(this->get_logger(), "Disabling Motors");

  enable_motors_ = msg->data;

  // Reset integrals when toggling motor state
  controller_.resetIntegrals();
}

PIDControlComponent::PIDControlComponent(const rclcpp::NodeOptions &options)
    : Node("pid_control", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_(0),
      current_yaw_(0),
      enable_motors_(false),
      use_external_yaw_(false)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "Intra-Process is "<< ( this->get_node_options().use_intra_process_comms() ?
  // "ON" : "OFF" ));
  this->declare_parameter<std::string>("quadrotor_name", std::string("quadrotor"));
  this->declare_parameter<double>("mass", 0.235);
  this->declare_parameter<bool>("use_external_yaw", true);
  this->declare_parameter<double>("max_roll_pitch", 30.0);
  this->declare_parameter<double>("gains/ki/x", 0.0);
  this->declare_parameter<double>("gains/ki/y", 0.0);
  this->declare_parameter<double>("gains/ki/z", 0.0);
  this->declare_parameter<double>("gains/ki/yaw", 0.01);

  std::string quadrotor_name;
  this->get_parameter("quadrotor_name", quadrotor_name);
  frame_id_ = "/" + quadrotor_name;

  double mass;
  this->get_parameter("mass", mass);
  controller_.setMass(mass);
  controller_.setMaxIntegral(mass * 3);

  this->get_parameter("use_external_yaw", use_external_yaw_);

  double max_roll_pitch;
  this->get_parameter("max_roll_pitch", max_roll_pitch);
  max_roll_pitch_ = max_roll_pitch;

  double ki_x, ki_y, ki_z, ki_yaw;
  this->get_parameter("gains/ki/x", ki_x);
  this->get_parameter("gains/ki/y", ki_y);
  this->get_parameter("gains/ki/z", ki_z);
  this->get_parameter("gains/ki/yaw", ki_yaw);
  ki_[0] = ki_x, ki_[1] = ki_y, ki_[2] = ki_z;
  ki_yaw_ = ki_yaw;

  trpy_command_pub_ = this->create_publisher<kr_mav_msgs::msg::TRPYCommand>("~/trpy_cmd", 10);

  // Setting QoS profile to get equivalent performance to ros::TransportHints().tcpNoDelay()
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos1 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
  auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 2), qos_profile);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", qos1, std::bind(&PIDControlComponent::odom_callback, this, std::placeholders::_1));
  position_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::PositionCommand>(
      "~/position_cmd", qos1, std::bind(&PIDControlComponent::position_cmd_callback, this, std::placeholders::_1));
  enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "~/motors", qos2, std::bind(&PIDControlComponent::enable_motors_callback, this, std::placeholders::_1));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PIDControlComponent)
