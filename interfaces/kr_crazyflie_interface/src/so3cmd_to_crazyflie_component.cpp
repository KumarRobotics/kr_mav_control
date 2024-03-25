// This nodelet is meant to subscribe to so3 commands and then convert them
// to geometry_msgs/Twist which is the type for cmd_vel, the crazyflie control
// topic. The format of this is as follows:
//  linear.y = roll     [-30 to 30 degrees]         (may be negative)
//  linear.x = pitch    [-30 to 30 degrees]         (may be negative)
//  linear.z = thrust   [0 to 60,000]               (motors stiction around 2000)
//  angular.z = yawrate [-200 to 200 degrees/second] (note this is not yaw!)

#include <Eigen/Geometry>

#include "geometry_msgs/msg/twist.hpp"
#include "kr_mav_msgs/msg/so3_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

// TODO: Remove CLAMP as macro
#define CLAMP(x, min, max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class SO3CmdToCrazyflie : public rclcpp::Node
{
 public:
  explicit SO3CmdToCrazyflie(const rclcpp::NodeOptions &options);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::UniquePtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom);

  bool odom_set_, so3_cmd_set_;
  Eigen::Quaterniond q_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr crazy_fast_cmd_vel_pub_, crazy_cmd_vel_pub_;

  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;

  double c1_;
  double c2_;
  double c3_;

  // TODO get rid of this for the gains coming in
  double kp_yaw_rate_;

  int thrust_pwm_min_;  // Necessary to overcome stiction
  int thrust_pwm_max_;  // Mapped to PWM

  int motor_status_;
};

void SO3CmdToCrazyflie::odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom)
{
  if(!odom_set_)
    odom_set_ = true;

  q_odom_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  if(so3_cmd_set_ && ((this->now() - last_so3_cmd_time_).seconds() >= so3_cmd_timeout_))
  {
    auto last_so3_cmd_ptr = std::make_unique<kr_mav_msgs::msg::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(std::move(last_so3_cmd_ptr));
  }
}

void SO3CmdToCrazyflie::so3_cmd_callback(kr_mav_msgs::msg::SO3Command::UniquePtr msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

  // switch on motors
  if(msg->aux.enable_motors)
  {
    // If the crazyflie motors are timed out, we need to send a zero message in order to get them to start
    if(motor_status_ < 3)
    {
      auto motors_vel_cmd = std::make_unique<geometry_msgs::msg::Twist>();
      crazy_cmd_vel_pub_->publish(std::move(motors_vel_cmd));
      last_so3_cmd_ = *msg;
      last_so3_cmd_time_ = msg->header.stamp;
      motor_status_ += 1;
      return;
    }
    // After sending zero message send min thrust
    if(motor_status_ < 10)
    {
      auto motors_vel_cmd = std::make_unique<geometry_msgs::msg::Twist>();
      motors_vel_cmd->linear.z = thrust_pwm_min_;
      crazy_cmd_vel_pub_->publish(std::move(motors_vel_cmd));
    }
    motor_status_ += 1;
  }
  else if(!msg->aux.enable_motors)
  {
    motor_status_ = 0;
    auto motors_vel_cmd = std::make_unique<geometry_msgs::msg::Twist>();
    crazy_cmd_vel_pub_->publish(std::move(motors_vel_cmd));
    last_so3_cmd_ = *msg;
    last_so3_cmd_time_ = msg->header.stamp;
    return;
  }

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(q_odom_);

  const float yaw_cur = std::atan2(R_cur(1, 0), R_cur(0, 0));
  const float yaw_des = std::atan2(R_des(1, 0), R_des(0, 0));

  // Map these into the current body frame (based on yaw)
  Eigen::Matrix3d R_des_new = R_des * Eigen::AngleAxisd(yaw_cur - yaw_des, Eigen::Vector3d::UnitZ());
  float pitch_des = -std::asin(R_des_new(2, 0));
  float roll_des = std::atan2(R_des_new(2, 1), R_des_new(2, 2));

  roll_des = roll_des * 180 / M_PI;
  pitch_des = pitch_des * 180 / M_PI;

  double thrust_f = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);  // Force in Newtons
  // RCLCPP_INFO(this->get_logger(), "thrust_f is %2.5f newtons", thrust_f);
  thrust_f = std::max(thrust_f, 0.0);

  thrust_f = thrust_f * 1000 / 9.81;  // Force in grams
  // RCLCPP_INFO(this->get_logger(), "thrust_f is %2.5f grams", thrust_f);

  // RCLCPP_INFO(this->get_logger(), "coeffs are %2.2f, %2.2f, %2.2f", c1_, c2_, c3_);
  float thrust_pwm = c1_ + c2_ * std::sqrt(c3_ + thrust_f);

  // RCLCPP_INFO(this->get_logger(), "thrust_pwm is %2.5f from 0-1", thrust_pwm);

  thrust_pwm = thrust_pwm * thrust_pwm_max_;  // thrust_pwm mapped from 0-60000
  // RCLCPP_INFO(this->get_logger(), "thrust_pwm is calculated to be %2.5f in pwm", thrust_pwm);

  auto crazy_vel_cmd = std::make_unique<geometry_msgs::msg::Twist>();

  float e_yaw = yaw_des - yaw_cur;
  if(e_yaw > M_PI)
    e_yaw -= 2 * M_PI;
  else if(e_yaw < -M_PI)
    e_yaw += 2 * M_PI;

  float yaw_rate_des = ((-msg->kr[2] * e_yaw) - msg->angular_velocity.z) * (180 / M_PI);

  // TODO change this check to be a param
  // throttle the publish rate
  // if ((this->now() - last_so3_cmd_time_).seconds() > 1.0/30.0){
  crazy_vel_cmd->linear.y = roll_des + msg->aux.angle_corrections[0];
  crazy_vel_cmd->linear.x = pitch_des + msg->aux.angle_corrections[1];
  crazy_vel_cmd->linear.z = CLAMP(thrust_pwm, thrust_pwm_min_, thrust_pwm_max_);

  // RCLCPP_INFO(this->get_logger(), "commanded thrust is %2.2f", CLAMP(thrust_pwm, thrust_pwm_min_, thrust_pwm_max_));

  crazy_vel_cmd->angular.z = yaw_rate_des;
  // RCLCPP_INFO(this->get_logger(), "commanded yaw rate is %2.2f", yaw_rate_des); //yaw_debug

  crazy_fast_cmd_vel_pub_->publish(std::move(crazy_vel_cmd));
  // save last so3_cmd
  last_so3_cmd_ = *msg;
  // last_so3_cmd_time_ = this->now();
  last_so3_cmd_time_ = msg->header.stamp;
  //}
  // else {
  //  RCLCPP_INFO_STREAM(this->get_logger(), "Commands too quick, time since is: " << (this->now() -
  //  last_so3_cmd_time_).seconds());
  //}
}

SO3CmdToCrazyflie::SO3CmdToCrazyflie(const rclcpp::NodeOptions &options)
    : Node("so3cmd_to_crazyflie", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  this->declare_parameter("kp_yaw_rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("c1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("c2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("c3", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter<double>("so3_cmd_timeout", 0.1);
  this->declare_parameter<int>("thrust_pwm_min", 10000);
  this->declare_parameter<int>("thrust_pwm_max", 60000);

  // get thrust scaling parameters
  // Note that this is ignoring a constant based on the number of props, which
  // is captured with the lin_cof_a variable later.
  //
  // TODO this is where we load thrust scaling stuff
  if(this->get_parameter("kp_yaw_rate", kp_yaw_rate_))
    RCLCPP_INFO(this->get_logger(), "kp yaw rate is %2.2f", kp_yaw_rate_);
  else
    RCLCPP_FATAL(this->get_logger(), "kp yaw rate not found");

  // get thrust scaling parameters
  if(this->get_parameter("c1", c1_) && this->get_parameter("c2", c2_) && this->get_parameter("c3", c3_))
    RCLCPP_INFO(this->get_logger(), "Using %2.2f, %2.2f, %2.2f for thrust mapping", c1_, c2_, c3_);
  else
    RCLCPP_FATAL(this->get_logger(), "Must set coefficients for thrust scaling");

  // get param for so3 command timeout duration
  so3_cmd_timeout_ = this->get_parameter("so3_cmd_timeout").as_double();

  thrust_pwm_max_ = this->get_parameter("thrust_pwm_max").as_int();
  thrust_pwm_min_ = this->get_parameter("thrust_pwm_min").as_int();

  odom_set_ = false;
  so3_cmd_set_ = false;
  motor_status_ = 0;

  // TODO make sure this is publishing to the right place
  crazy_fast_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel_fast", 10);

  crazy_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

  // Setting QoS profile to get equivalent performance to ros::TransportHints().tcpNoDelay()
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos1 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
  auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

  so3_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>(
      "~/so3_cmd", qos2, std::bind(&SO3CmdToCrazyflie::so3_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", qos1, std::bind(&SO3CmdToCrazyflie::odom_callback, this, std::placeholders::_1));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToCrazyflie)
