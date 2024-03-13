#include <Eigen/Geometry>

#include "kr_mav_controllers/SO3Control.hpp"
#include "kr_mav_msgs/msg/corrections.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_mav_msgs/msg/trpy_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define CLAMP(x, min, max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class SO3TRPYControlComponent : public rclcpp::Node
{
 public:
  explicit SO3TRPYControlComponent(const rclcpp::NodeOptions &options);

 private:
  void publishCommand();
  void position_cmd_callback(const kr_mav_msgs::msg::PositionCommand::UniquePtr cmd);
  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom);
  void enable_motors_callback(const std_msgs::msg::Bool::UniquePtr msg);
  void corrections_callback(const kr_mav_msgs::msg::Corrections::UniquePtr msg);
  rcl_interfaces::msg::SetParametersResult cfg_callback(std::vector<rclcpp::Parameter> parameters);

  SO3Control controller_;
  rclcpp::Publisher<kr_mav_msgs::msg::TRPYCommand>::SharedPtr trpy_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<kr_mav_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<kr_mav_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfigure_handle_;

  bool odom_set_, position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, kx_, kv_, config_kx_, config_kv_, config_ki_, config_kib_;
  float des_yaw_, des_yaw_dot_, yaw_int_;
  bool enable_motors_, use_external_yaw_;
  float kr_[3], kom_[3], corrections_[3];
  float ki_yaw_;
  float mass_;
  const float g_;
  Eigen::Quaternionf current_orientation_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Need this since we have SO3Control which needs aligned pointer
};

void SO3TRPYControlComponent::publishCommand()
{
  if(!odom_set_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Odom no set, not publishing command!");
    return;
  }

  float ki_yaw = 0;
  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f kib = Eigen::Vector3f::Zero();
  if(enable_motors_)
  {
    ki_yaw = ki_yaw_;
    ki = config_ki_;
    kib = config_kib_;
  }
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &q_des = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  const Eigen::Matrix3f R_des(q_des);
  const Eigen::Matrix3f R_cur(current_orientation_);

  const float yaw_cur = std::atan2(R_cur(1, 0), R_cur(0, 0));
  const float yaw_des = std::atan2(R_des(1, 0), R_des(0, 0));
  const float pitch_des = -std::asin(R_des(2, 0));
  const float roll_des = std::atan2(R_des(2, 1), R_des(2, 2));

  const float Psi = 0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) + R_des(2, 0) * R_cur(2, 0) +
                                    R_des(0, 1) * R_cur(0, 1) + R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                                    R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) + R_des(2, 2) * R_cur(2, 2)));

  float thrust = 0.0f;
  if(Psi < 1.0f)  // Position control stability guaranteed only when Psi < 1
    thrust = force(0) * R_cur(0, 2) + force(1) * R_cur(1, 2) + force(2) * R_cur(2, 2);

  float e_yaw = yaw_des - yaw_cur;

  const float PI = static_cast<float>(M_PI);
  if(e_yaw > PI)
    e_yaw -= 2 * PI;
  else if(e_yaw < -PI)
    e_yaw += 2 * PI;

  // Yaw integral
  yaw_int_ += ki_yaw * e_yaw;
  if(yaw_int_ > PI)
    yaw_int_ = PI;
  else if(yaw_int_ < -PI)
    yaw_int_ = -PI;

  float yaw_cmd = yaw_des + yaw_int_;
  if(yaw_cmd > PI)
    yaw_cmd -= 2 * PI;
  else if(yaw_cmd < -PI)
    yaw_cmd += 2 * PI;

  auto trpy_command = std::make_unique<kr_mav_msgs::msg::TRPYCommand>();
  trpy_command->header.stamp = this->now();
  trpy_command->header.frame_id = frame_id_;
  if(enable_motors_)
  {
    trpy_command->thrust = CLAMP(thrust, 0.01f * 9.81f, 10.0f * 9.81f);
    trpy_command->roll = roll_des;
    trpy_command->pitch = pitch_des;
    trpy_command->yaw = yaw_cmd;
    trpy_command->angular_velocity.x = ang_vel(0);
    trpy_command->angular_velocity.y = ang_vel(1);
    trpy_command->angular_velocity.z = ang_vel(2);
    for(int i = 0; i < 3; i++)
    {
      trpy_command->kr[i] = kr_[i];
      trpy_command->kom[i] = kom_[i];
    }
  }
  trpy_command->aux.current_yaw = yaw_cur;
  trpy_command->aux.kf_correction = corrections_[0];
  trpy_command->aux.angle_corrections[0] = corrections_[1];
  trpy_command->aux.angle_corrections[1] = corrections_[2];
  trpy_command->aux.enable_motors = enable_motors_;
  trpy_command->aux.use_external_yaw = use_external_yaw_;
  trpy_command_pub_->publish(std::move(trpy_command));
}

void SO3TRPYControlComponent::position_cmd_callback(const kr_mav_msgs::msg::PositionCommand::UniquePtr cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  des_jrk_ = Eigen::Vector3f(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);

  // Check use_msg_gains_flag to decide whether to use gains from the msg or config
  kx_[0] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_X) ? cmd->kx[0] : config_kx_[0];
  kx_[1] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_Y) ? cmd->kx[1] : config_kx_[1];
  kx_[2] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_Z) ? cmd->kx[2] : config_kx_[2];
  kv_[0] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_X) ? cmd->kv[0] : config_kv_[0];
  kv_[1] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_Y) ? cmd->kv[1] : config_kv_[1];
  kv_[2] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_Z) ? cmd->kv[2] : config_kv_[2];

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishCommand();
}

void SO3TRPYControlComponent::odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom)
{
  if(!odom_set_)
    odom_set_ = true;

  const Eigen::Vector3f position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

  current_orientation_ = Eigen::Quaternionf(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setCurrentOrientation(current_orientation_);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishCommand();
    position_cmd_updated_ = false;
  }
}

void SO3TRPYControlComponent::enable_motors_callback(const std_msgs::msg::Bool::UniquePtr msg)
{
  if(msg->data)
    RCLCPP_INFO(this->get_logger(), "Enabling motors");
  else
    RCLCPP_INFO(this->get_logger(), "Disabling motors");

  enable_motors_ = msg->data;

  // Reset integrals when toggling motor state
  yaw_int_ = 0;
  controller_.resetIntegrals();
}

void SO3TRPYControlComponent::corrections_callback(const kr_mav_msgs::msg::Corrections::UniquePtr msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}
rcl_interfaces::msg::SetParametersResult SO3TRPYControlComponent::cfg_callback(
    std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for(auto &parameter : parameters)
  {
    const auto &name = parameter.get_name();

    if(name == "kp_x")
    {
      config_kx_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "kp_y")
    {
      config_kx_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "kp_z")
    {
      config_kx_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "kd_x")
    {
      config_kv_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "kd_y")
    {
      config_kv_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "kd_z")
    {
      config_kv_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}",
                  config_kx_[0], config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
    }
    else if(name == "ki_x")
    {
      config_ki_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "ki_y")
    {
      config_ki_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "ki_z")
    {
      config_ki_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "kib_x")
    {
      config_kib_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "ki_y")
    {
      config_kib_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "ki_z")
    {
      config_kib_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}",
                  config_ki_[0], config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
    }
    else if(name == "rot_x")
    {
      kr_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "rot_y")
    {
      kr_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "rot_z")
    {
      kr_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "ang_x")
    {
      kom_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "ang_y")
    {
      kom_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "ang_z")
    {
      kom_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}",
                  kr_[0], kr_[1], kr_[2], kom_[0], kom_[1], kom_[2]);
    }
    else if(name == "kf_correction")
    {
      corrections_[0] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Corrections set to kf: %2.2g, roll: %2.2g, pitch: %2.2g", corrections_[0],
                  corrections_[1], corrections_[2]);
    }
    else if(name == "roll_correction")
    {
      corrections_[1] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Corrections set to kf: %2.2g, roll: %2.2g, pitch: %2.2g", corrections_[0],
                  corrections_[1], corrections_[2]);
    }
    else if(name == "pitch_correction")
    {
      corrections_[2] = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Corrections set to kf: %2.2g, roll: %2.2g, pitch: %2.2g", corrections_[0],
                  corrections_[1], corrections_[2]);
    }
    else if(name == "max_pos_int")
    {
      controller_.setMaxIntegral(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "World max integral set to: %2.2g", parameter.as_double());
    }
    else if(name == "max_pos_int_b")
    {
      controller_.setMaxIntegralBody(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Body max integral set to: %2.2g", parameter.as_double());
    }
    else if(name == "max_tilt_angle")
    {
      controller_.setMaxTiltAngle(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Max tilt angle set to: %2.2g", parameter.as_double());
    }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "kr_mav_controllers dynamic reconfigure called with invalid parameter");
      result.successful = false;
      return result;
    }
  }
  result.successful = true;
  return result;
}

SO3TRPYControlComponent::SO3TRPYControlComponent(const rclcpp::NodeOptions &options)
    : Node("so3_control", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
      odom_set_(false),
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_(0),
      des_yaw_dot_(0),
      yaw_int_(0),
      enable_motors_(false),
      use_external_yaw_(false),
      g_(9.81)
{
  controller_.resetIntegrals();

  this->declare_parameter("quadrotor_name", std::string("quadrotor"));
  this->declare_parameter("mass", 0.5f);

  std::string quadrotor_name;
  this->get_parameter("quadrotor_name", quadrotor_name);
  frame_id_ = "/" + quadrotor_name;
  this->get_parameter("mass", mass_);

  controller_.setMass(mass_);
  controller_.setGravity(g_);

  this->declare_parameter("use_external_yaw", true);
  this->get_parameter("use_external_yaw", use_external_yaw_);

  // Dynamic parameter reconfiguration definitions

  // Translation Gains: World Position Gains
  rcl_interfaces::msg::ParameterDescriptor desc_kp_x;
  rcl_interfaces::msg::FloatingPointRange r_kp_x;
  r_kp_x.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kp_x.type = 3;  // PARAMETER_DOUBLE
  desc_kp_x.description = std::string("World x position gain");
  desc_kp_x.floating_point_range = {r_kp_x};
  this->declare_parameter("kp_x", 7.4f, desc_kp_x);

  rcl_interfaces::msg::ParameterDescriptor desc_kp_y;
  rcl_interfaces::msg::FloatingPointRange r_kp_y;
  r_kp_y.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kp_y.type = 3;  // PARAMETER_DOUBLE
  desc_kp_y.description = std::string("World y position gain");
  desc_kp_y.floating_point_range = {r_kp_y};
  this->declare_parameter("kp_y", 7.4f, desc_kp_y);

  rcl_interfaces::msg::ParameterDescriptor desc_kp_z;
  rcl_interfaces::msg::FloatingPointRange r_kp_z;
  r_kp_z.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kp_z.type = 3;  // PARAMETER_DOUBLE
  desc_kp_z.description = std::string("World z position gain");
  desc_kp_z.floating_point_range = {r_kp_z};
  this->declare_parameter("kp_z", 10.4f, desc_kp_z);

  // Translation Gains: World Derivative Gains
  rcl_interfaces::msg::ParameterDescriptor desc_kd_x;
  rcl_interfaces::msg::FloatingPointRange r_kd_x;
  r_kd_x.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kd_x.type = 3;  // PARAMETER_DOUBLE
  desc_kd_x.description = std::string("World x derivative gain");
  desc_kd_x.floating_point_range = {r_kd_x};
  this->declare_parameter("kd_x", 4.8f, desc_kd_x);

  rcl_interfaces::msg::ParameterDescriptor desc_kd_y;
  rcl_interfaces::msg::FloatingPointRange r_kd_y;
  r_kd_y.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kd_y.type = 3;  // PARAMETER_DOUBLE
  desc_kd_y.description = std::string("World y derivative gain");
  desc_kd_y.floating_point_range = {r_kd_y};
  this->declare_parameter("kd_y", 4.8f, desc_kd_y);

  rcl_interfaces::msg::ParameterDescriptor desc_kd_z;
  rcl_interfaces::msg::FloatingPointRange r_kd_z;
  r_kd_z.set__from_value(0.0).set__to_value(20.0).set__step(0);
  desc_kd_z.type = 3;  // PARAMETER_DOUBLE
  desc_kd_z.description = std::string("World z derivative gain");
  desc_kd_z.floating_point_range = {r_kd_z};
  this->declare_parameter("kd_z", 6.0f, desc_kd_z);

  // Integral Gains: World Integral Gains
  rcl_interfaces::msg::ParameterDescriptor desc_ki_x;
  rcl_interfaces::msg::FloatingPointRange r_ki_x;
  r_ki_x.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_ki_x.type = 3;  // PARAMETER_DOUBLE
  desc_ki_x.description = std::string("World x integral gain");
  desc_ki_x.floating_point_range = {r_ki_x};
  this->declare_parameter("ki_x", 0.0f, desc_ki_x);

  rcl_interfaces::msg::ParameterDescriptor desc_ki_y;
  rcl_interfaces::msg::FloatingPointRange r_ki_y;
  r_ki_y.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_ki_y.type = 3;  // PARAMETER_DOUBLE
  desc_ki_y.description = std::string("World y integral gain");
  desc_ki_y.floating_point_range = {r_ki_y};
  this->declare_parameter("ki_y", 0.0f, desc_ki_y);

  rcl_interfaces::msg::ParameterDescriptor desc_ki_z;
  rcl_interfaces::msg::FloatingPointRange r_ki_z;
  r_ki_z.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_ki_z.type = 3;  // PARAMETER_DOUBLE
  desc_ki_z.description = std::string("World z integral gain");
  desc_ki_z.floating_point_range = {r_ki_z};
  this->declare_parameter("ki_z", 0.0f, desc_ki_z);

  // Integral Gains: Body Integral Gains
  rcl_interfaces::msg::ParameterDescriptor desc_kib_x;
  rcl_interfaces::msg::FloatingPointRange r_kib_x;
  r_kib_x.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_kib_x.type = 3;  // PARAMETER_DOUBLE
  desc_kib_x.description = std::string("Body x integral gain");
  desc_kib_x.floating_point_range = {r_kib_x};
  this->declare_parameter("kib_x", 0.0f, desc_kib_x);

  rcl_interfaces::msg::ParameterDescriptor desc_kib_y;
  rcl_interfaces::msg::FloatingPointRange r_kib_y;
  r_kib_y.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_kib_y.type = 3;  // PARAMETER_DOUBLE
  desc_kib_y.description = std::string("Body y integral gain");
  desc_kib_y.floating_point_range = {r_kib_y};
  this->declare_parameter("kib_y", 0.0f, desc_kib_y);

  rcl_interfaces::msg::ParameterDescriptor desc_kib_z;
  rcl_interfaces::msg::FloatingPointRange r_kib_z;
  r_kib_z.set__from_value(0.0).set__to_value(0.2).set__step(0);
  desc_kib_z.type = 3;  // PARAMETER_DOUBLE
  desc_kib_z.description = std::string("Body z integral gain");
  desc_kib_z.floating_point_range = {r_kib_z};
  this->declare_parameter("kib_z", 0.0f, desc_kib_z);

  // Attitude Gains: Rotation Gains
  rcl_interfaces::msg::ParameterDescriptor desc_rot_x;
  rcl_interfaces::msg::FloatingPointRange r_rot_x;
  r_rot_x.set__from_value(0.0).set__to_value(3.00).set__step(0);
  desc_rot_x.type = 3;  // PARAMETER_DOUBLE
  desc_rot_x.description = std::string("Rotation x gain");
  desc_rot_x.floating_point_range = {r_rot_x};
  this->declare_parameter("rot_x", 1.5f, desc_rot_x);

  rcl_interfaces::msg::ParameterDescriptor desc_rot_y;
  rcl_interfaces::msg::FloatingPointRange r_rot_y;
  r_rot_y.set__from_value(0.0).set__to_value(3.00).set__step(0);
  desc_rot_y.type = 3;  // PARAMETER_DOUBLE
  desc_rot_y.description = std::string("Rotation y gain");
  desc_rot_y.floating_point_range = {r_rot_y};
  this->declare_parameter("rot_y", 1.5f, desc_rot_y);

  rcl_interfaces::msg::ParameterDescriptor desc_rot_z;
  rcl_interfaces::msg::FloatingPointRange r_rot_z;
  r_rot_z.set__from_value(0.0).set__to_value(3.00).set__step(0);
  desc_rot_z.type = 3;  // PARAMETER_DOUBLE
  desc_rot_z.description = std::string("Rotation z gain");
  desc_rot_z.floating_point_range = {r_rot_z};
  this->declare_parameter("rot_z", 1.0f, desc_rot_z);

  // Attitude Gains: Angular Gains
  rcl_interfaces::msg::ParameterDescriptor desc_ang_x;
  rcl_interfaces::msg::FloatingPointRange r_ang_x;
  r_ang_x.set__from_value(0.0).set__to_value(1.00).set__step(0);
  desc_ang_x.type = 3;  // PARAMETER_DOUBLE
  desc_ang_x.description = std::string("Angular x gain");
  desc_ang_x.floating_point_range = {r_ang_x};
  this->declare_parameter("ang_x", 0.13f, desc_ang_x);

  rcl_interfaces::msg::ParameterDescriptor desc_ang_y;
  rcl_interfaces::msg::FloatingPointRange r_ang_y;
  r_ang_y.set__from_value(0.0).set__to_value(1.00).set__step(0);
  desc_ang_y.type = 3;  // PARAMETER_DOUBLE
  desc_ang_y.description = std::string("Angular y gain");
  desc_ang_y.floating_point_range = {r_ang_y};
  this->declare_parameter("ang_y", 0.13f, desc_ang_y);

  rcl_interfaces::msg::ParameterDescriptor desc_ang_z;
  rcl_interfaces::msg::FloatingPointRange r_ang_z;
  r_ang_z.set__from_value(0.0).set__to_value(1.00).set__step(0);
  desc_ang_z.type = 3;  // PARAMETER_DOUBLE
  desc_ang_z.description = std::string("Angular z gain");
  desc_ang_z.floating_point_range = {r_ang_z};
  this->declare_parameter("ang_z", 0.10f, desc_ang_z);

  // Low level Corrections
  rcl_interfaces::msg::ParameterDescriptor desc_kf_correction;
  desc_kf_correction.type = 3;  // PARAMETER_DOUBLE
  desc_kf_correction.description = std::string("kf");
  this->declare_parameter("kf_correction", 0.0f, desc_kf_correction);

  rcl_interfaces::msg::ParameterDescriptor desc_roll_correction;
  desc_roll_correction.type = 3;  // PARAMETER_DOUBLE
  desc_roll_correction.description = std::string("roll");
  this->declare_parameter("roll_correction", 0.0f, desc_roll_correction);

  rcl_interfaces::msg::ParameterDescriptor desc_pitch_correction;
  desc_pitch_correction.type = 3;  // PARAMETER_DOUBLE
  desc_pitch_correction.description = std::string("pitch");
  this->declare_parameter("pitch_correction", 0.0f, desc_pitch_correction);

  // Misc Parameters
  rcl_interfaces::msg::ParameterDescriptor desc_max_pos_int;
  rcl_interfaces::msg::FloatingPointRange r_max_pos_int;
  r_max_pos_int.set__from_value(0.0).set__to_value(4.00).set__step(0);
  desc_max_pos_int.type = 3;  // PARAMETER_DOUBLE
  desc_max_pos_int.description = std::string("World max integral");
  desc_max_pos_int.floating_point_range = {r_max_pos_int};
  this->declare_parameter("max_pos_int", 0.5f, desc_max_pos_int);

  rcl_interfaces::msg::ParameterDescriptor desc_max_pos_int_b;
  rcl_interfaces::msg::FloatingPointRange r_max_pos_int_b;
  r_max_pos_int_b.set__from_value(0.0).set__to_value(4.00).set__step(0);
  desc_max_pos_int_b.type = 3;  // PARAMETER_DOUBLE
  desc_max_pos_int_b.description = std::string("Body max integral");
  desc_max_pos_int_b.floating_point_range = {r_max_pos_int_b};
  this->declare_parameter("max_pos_int_b", 0.5f, desc_max_pos_int_b);

  rcl_interfaces::msg::ParameterDescriptor desc_max_tilt_angle;
  rcl_interfaces::msg::FloatingPointRange r_max_tilt_angle;
  r_max_tilt_angle.set__from_value(0.0).set__to_value(3.14).set__step(0);
  desc_max_tilt_angle.type = 3;  // PARAMETER_DOUBLE
  desc_max_tilt_angle.description = std::string("Max tilt angle");
  desc_max_tilt_angle.floating_point_range = {r_max_tilt_angle};
  this->declare_parameter("max_tilt_angle", static_cast<float>(M_PI), desc_max_tilt_angle);

  // Getting dynamic parameters
  this->get_parameter("kp_x", config_kx_[0]);
  this->get_parameter("kp_y", config_kx_[1]);
  this->get_parameter("kp_z", config_kx_[2]);
  kx_[0] = config_kx_[0];
  kx_[1] = config_kx_[1];
  kx_[2] = config_kx_[2];

  this->get_parameter("kd_x", config_kv_[0]);
  this->get_parameter("kd_y", config_kv_[1]);
  this->get_parameter("kd_z", config_kv_[2]);
  kv_[0] = config_kv_[0];
  kv_[1] = config_kv_[1];
  kv_[2] = config_kv_[2];

  this->get_parameter("ki_x", config_ki_[0]);
  this->get_parameter("ki_y", config_ki_[1]);
  this->get_parameter("ki_z", config_ki_[2]);

  this->get_parameter("kib_x", config_kib_[0]);
  this->get_parameter("kib_y", config_kib_[1]);
  this->get_parameter("kib_z", config_kib_[2]);

  this->get_parameter("rot_x", kr_[0]);
  this->get_parameter("rot_y", kr_[1]);
  this->get_parameter("rot_z", kr_[2]);

  this->get_parameter("ang_x", kom_[0]);
  this->get_parameter("ang_y", kom_[1]);
  this->get_parameter("ang_z", kom_[2]);

  this->get_parameter("kf_correction", corrections_[0]);
  this->get_parameter("roll_correction", corrections_[1]);
  this->get_parameter("pitch_correction", corrections_[2]);

  float max_pos_int, max_pos_int_b;
  this->get_parameter("max_pos_int", max_pos_int);
  this->get_parameter("max_pos_int_b", max_pos_int_b);
  controller_.setMaxIntegral(max_pos_int);
  controller_.setMaxIntegralBody(max_pos_int_b);

  float max_tilt_angle;
  this->get_parameter("max_tilt_angle", max_tilt_angle);
  controller_.setMaxTiltAngle(max_tilt_angle);

  // Initialize dynamic reconfigure callback
  reconfigure_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SO3TRPYControlComponent::cfg_callback, this, std::placeholders::_1));

  trpy_command_pub_ = this->create_publisher<kr_mav_msgs::msg::TRPYCommand>("~/trpy_cmd", 10);

  // Setting QoS profile to get equivalent performance to ros::TransportHints().tcpNoDelay()
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos1 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
  auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 2), qos_profile);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", qos1, std::bind(&SO3TRPYControlComponent::odom_callback, this, std::placeholders::_1));
  position_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::PositionCommand>(
      "~/position_cmd", qos1, std::bind(&SO3TRPYControlComponent::position_cmd_callback, this, std::placeholders::_1));
  enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "~/motors", qos2, std::bind(&SO3TRPYControlComponent::enable_motors_callback, this, std::placeholders::_1));
  corrections_sub_ = this->create_subscription<kr_mav_msgs::msg::Corrections>(
      "~/corrections", qos1, std::bind(&SO3TRPYControlComponent::corrections_callback, this, std::placeholders::_1));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SO3TRPYControlComponent)
