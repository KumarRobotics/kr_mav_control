#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kr_mav_msgs/msg/so3_command.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

static constexpr double kYawOffsetWarnThreshold = 5.0 * M_PI / 180.0;

static std::pair<double, double> solve_quadratic(double a, double b, double c)
{
  const double term1 = -b, term2 = std::sqrt(b * b - 4 * a * c);
  return std::make_pair((term1 + term2) / (2 * a), (term1 - term2) / (2 * a));
}

class SO3CmdToMavros : public rclcpp::Node
{
 public:
  explicit SO3CmdToMavros(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose);
  void publish_attitude_target(const kr_mav_msgs::msg::SO3Command &msg, bool fresh_command);

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_;
  double lin_cof_a_, lin_int_b_;
  int num_props_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub_;

  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  double so3_cmd_timeout_;
  double odom_timeout_;
  double idle_hold_max_force_;
  double vision_pose_period_;
  rclcpp::Time next_vision_pose_time_;
  rclcpp::Time last_so3_cmd_time_;
  rclcpp::Time last_odom_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;
};

void SO3CmdToMavros::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  const auto &p = odom->pose.pose.position;
  const auto &q = odom->pose.pose.orientation;

  // Reject invalid odom rather than feeding it to the controller and to EKF2 via
  // vision_pose. Matches the guard the betaflight interface already has.
  if(!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
     !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Received non-finite odom pose, ignoring");
    odom_set_ = false;
    return;
  }

  const rclcpp::Time now = this->now();

  odom_q_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  odom_set_ = true;
  last_odom_time_ = now;

  // Deadline accumulator rather than a simple "elapsed >= period" test. The latter
  // aliases: at 114 Hz odom against a 20 ms budget, the sample at 17.5 ms is
  // rejected and the next lands at 26.3 ms, so it publishes every 3rd sample and
  // yields 38 Hz, not 50. Advancing the deadline by exactly one period keeps the
  // long-run average on target (alternating 2 and 3 input samples).
  if(vision_pose_period_ > 0.0)
  {
    if(now < next_vision_pose_time_)
      return;
    next_vision_pose_time_ = next_vision_pose_time_ + rclcpp::Duration::from_seconds(vision_pose_period_);
    // Resync if we fell behind (odom stalled, clock jump) instead of bursting to
    // catch up on a backlog that no longer matters.
    if(next_vision_pose_time_ < now)
      next_vision_pose_time_ = now + rclcpp::Duration::from_seconds(vision_pose_period_);
  }

  auto odom_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  odom_pose_msg->header = odom->header;
  odom_pose_msg->pose = odom->pose.pose;
  odom_pose_pub_->publish(*odom_pose_msg);
}

void SO3CmdToMavros::imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose)
{
  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);
  imu_set_ = true;

  if(!so3_cmd_set_)
    return;

  const rclcpp::Time now = this->now();
  const double cmd_age = (now - last_so3_cmd_time_).seconds();
  if(cmd_age < so3_cmd_timeout_)
    return;  // commands are flowing normally; nothing to do

  const double odom_age = (now - last_odom_time_).seconds();
  if(!odom_set_ || odom_age > odom_timeout_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "so3_cmd stale (%.2f s) AND odom stale (%.2f s): not sending setpoints. "
                         "PX4 offboard failsafe should engage.",
                         cmd_age, odom_age);
    return;
  }
  
  const double f = std::sqrt(last_so3_cmd_.force.x * last_so3_cmd_.force.x +
                             last_so3_cmd_.force.y * last_so3_cmd_.force.y +
                             last_so3_cmd_.force.z * last_so3_cmd_.force.z);
  if(f > idle_hold_max_force_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "so3_cmd stale (%.2f s) while airborne (last commanded force %.2f N): NOT holding. "
                         "PX4 offboard failsafe should engage.",
                         cmd_age, f);
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "so3_cmd idle for %.2f s (odom healthy, force %.3f N): holding level at current yaw "
                       "to keep OFFBOARD alive.",
                       cmd_age, f);

  kr_mav_msgs::msg::SO3Command hold = last_so3_cmd_;
  const double odom_yaw = std::atan2(2.0 * (odom_q_.w() * odom_q_.z() + odom_q_.x() * odom_q_.y()),
                                     1.0 - 2.0 * (odom_q_.y() * odom_q_.y() + odom_q_.z() * odom_q_.z()));
  hold.orientation.w = std::cos(odom_yaw / 2.0);
  hold.orientation.x = 0.0;
  hold.orientation.y = 0.0;
  hold.orientation.z = std::sin(odom_yaw / 2.0);

  publish_attitude_target(hold, false);
}

void SO3CmdToMavros::so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg)
{
  // Record the genuine command first, so the idle-hold path in imu_callback has
  // something to re-send and can measure how long it has been since a real one.
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = this->now();
  so3_cmd_set_ = true;

  publish_attitude_target(*msg, true);
}

void SO3CmdToMavros::publish_attitude_target(const kr_mav_msgs::msg::SO3Command &cmd, bool fresh_command)
{
  const auto msg = &cmd;

  // both imu_q_ and odom_q_ would be uninitialized if not set. Throttled because
  // mav_services queues a burst of commands from set_motors(), which on a normal
  // startup arrives before mavros has finished connecting to the FCU.
  if(!imu_set_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Did not receive any imu messages (is mavros connected to the FCU?)");
    return;
  }

  if(!odom_set_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Did not receive any odom messages");
    return;
  }

  // transform to take into consideration the different yaw of the flight
  // controller imu and the odom
  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  // convert to tf2::Quaternion
  tf2::Quaternion imu_tf(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf2::Quaternion odom_tf(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  // extract RPY's
  double imu_roll, imu_pitch, imu_yaw;
  double odom_roll, odom_pitch, odom_yaw;
  tf2::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  tf2::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);

  // create only yaw tf2::Quaternions
  tf2::Quaternion imu_tf_yaw;
  tf2::Quaternion odom_tf_yaw;
  imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  const tf2::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // Diagnostic: how far the FCU's heading estimate has drifted from the odom heading.
  // This correction silently rotates the commanded tilt direction, so a large offset
  // means the position loop is fighting a rotated frame.
  const double yaw_offset = std::atan2(std::sin(imu_yaw - odom_yaw), std::cos(imu_yaw - odom_yaw));
  if(std::abs(yaw_offset) > kYawOffsetWarnThreshold)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "FCU yaw and odom yaw differ by %.1f deg. Check that EKF2 is fusing the "
                         "vision pose for yaw.",
                         yaw_offset * 180.0 / M_PI);
  }

  // transform!
  const Eigen::Quaterniond q_des_transformed =
      Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(), tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) * q_des;

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const float Psi = 0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) + R_des(2, 0) * R_cur(2, 0) +
                                    R_des(0, 1) * R_cur(0, 1) + R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                                    R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) + R_des(2, 2) * R_cur(2, 2)));

  if(fresh_command && Psi > 1.0f)  // Position control stability guaranteed only when Psi < 1
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Psi > 1.0, orientation error is too large!");
  }

  double thrust = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  // Scale thrust to individual rotor velocities (RPM) via quadratic thrust curve.
  const double avg_thrust = std::max(0.0, thrust) / num_props_;
  const auto rpm_solutions =
      solve_quadratic(thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_ - avg_thrust);
  const double omega_avg = std::max(rpm_solutions.first, rpm_solutions.second);

  // Scaling from rotor velocity (RPM) to att_throttle for pixhawk
  double throttle = lin_cof_a_ * omega_avg + lin_int_b_;

  // failsafe for the error in traj_gen that can lead to nan values
  //prevents throttle from being sent to 1 if it is nan.
  if (std::isnan(throttle))
  {
    throttle = 0.0;
  }

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if(!msg->aux.enable_motors)
    throttle = 0;

  // publish messages
  auto setpoint_msg = std::make_shared<mavros_msgs::msg::AttitudeTarget>();
  setpoint_msg->header = msg->header;
  // Stamp with now(), not the original command's stamp: on the idle-hold path the
  // stored command can be seconds old and PX4 should see a current setpoint.
  setpoint_msg->header.stamp = this->now();
  setpoint_msg->type_mask = 0;
  setpoint_msg->orientation.w = q_des_transformed.w();
  setpoint_msg->orientation.x = q_des_transformed.x();
  setpoint_msg->orientation.y = q_des_transformed.y();
  setpoint_msg->orientation.z = q_des_transformed.z();
  setpoint_msg->body_rate.x = msg->angular_velocity.x;
  setpoint_msg->body_rate.y = msg->angular_velocity.y;
  setpoint_msg->body_rate.z = msg->angular_velocity.z;
  setpoint_msg->thrust = throttle;

  attitude_raw_pub_->publish(*setpoint_msg);
}

SO3CmdToMavros::SO3CmdToMavros(const rclcpp::NodeOptions &options) : rclcpp::Node("so3cmd_to_mavros", options)
{
  const auto &overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  auto was_configured = [&overrides](const std::string &name) { return overrides.count(name) > 0; };

  this->declare_parameter("num_props", 4);
  this->get_parameter("num_props", num_props_);
  if(was_configured("num_props"))
    RCLCPP_INFO(this->get_logger(), "Got number of props: %d", num_props_);
  else
    RCLCPP_ERROR(this->get_logger(), "num_props not set, defaulting to %d", num_props_);

  // quadratic thrust-vs-RPM curve: thrust_per_prop = a*omega^2 + b*omega + c
  this->declare_parameter("thrust_vs_rpm_cof_a", 2.137145e-6);
  this->declare_parameter("thrust_vs_rpm_cof_b", 0.0);
  this->declare_parameter("thrust_vs_rpm_cof_c", 0.0);
  this->get_parameter("thrust_vs_rpm_cof_a", thrust_vs_rpm_cof_a_);
  this->get_parameter("thrust_vs_rpm_cof_b", thrust_vs_rpm_cof_b_);
  this->get_parameter("thrust_vs_rpm_cof_c", thrust_vs_rpm_cof_c_);

  if(was_configured("thrust_vs_rpm_cof_a") && was_configured("thrust_vs_rpm_cof_b") &&
     was_configured("thrust_vs_rpm_cof_c"))
  {
    RCLCPP_INFO(this->get_logger(), "Using thrust = %g*omega^2 + %g*omega + %g to scale force to rotor speed.",
                thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Must set thrust_vs_rpm_cof_{a,b,c} params for thrust scaling.");
  }

  if(thrust_vs_rpm_cof_a_ <= 0)
  {
    RCLCPP_FATAL(this->get_logger(), "thrust_vs_rpm_cof_a must be positive. thrust_vs_rpm_cof_a = %g", thrust_vs_rpm_cof_a_);
    throw std::invalid_argument("thrust_vs_rpm_cof_a must be positive");
  }

  // scaling from rotor velocity (RPM) to att_throttle for pixhawk
  this->declare_parameter("lin_cof_a", 0.0015);
  this->declare_parameter("lin_int_b", -1.5334);
  this->get_parameter("lin_cof_a", lin_cof_a_);
  this->get_parameter("lin_int_b", lin_int_b_);

  if(was_configured("lin_cof_a") && was_configured("lin_int_b"))
  {
    RCLCPP_INFO(this->get_logger(), "Using %g*x + %g to scale prop speed to att_throttle.", lin_cof_a_, lin_int_b_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Must set coefficients for thrust scaling (scaling from rotor velocity (RPM) to att_throttle for pixhawk)");
  }

  // get param for so3 command timeout duration
  this->declare_parameter("so3_cmd_timeout", 0.25);
  this->get_parameter("so3_cmd_timeout", so3_cmd_timeout_);

  // How stale odom may be before the idle-hold in imu_callback stops re-sending
  // the last command and lets PX4's offboard-loss failsafe take over.
  this->declare_parameter("odom_timeout", 0.2);
  this->get_parameter("odom_timeout", odom_timeout_);

  // Above this commanded force the vehicle is considered airborne and the
  // idle-hold refuses to engage. Ground idle commands ~0 N (set_motors sends
  // FLT_MIN); hover for this airframe is ~7.6 N, so 1 N separates them cleanly.
  this->declare_parameter("idle_hold_max_force", 1.0);
  this->get_parameter("idle_hold_max_force", idle_hold_max_force_);

  // Max rate for the vision_pose stream to the FCU, in Hz. 0 = unthrottled
  // (the old behaviour, which overflowed the serial TX queue at ~114 Hz).
  this->declare_parameter("vision_pose_rate", 50.0);
  double vision_pose_rate = this->get_parameter("vision_pose_rate").as_double();
  vision_pose_period_ = (vision_pose_rate > 0.0) ? (1.0 / vision_pose_rate) : 0.0;
  RCLCPP_INFO(this->get_logger(), "vision_pose to FCU limited to %.1f Hz", vision_pose_rate);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;
  last_so3_cmd_time_ = this->now();
  last_odom_time_ = this->now();
  next_vision_pose_time_ = this->now();

  attitude_raw_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("~/attitude_raw", 10);
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/odom_pose", 10);

  so3_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>(
      "~/so3_cmd", 10, std::bind(&SO3CmdToMavros::so3_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", rclcpp::SensorDataQoS(), std::bind(&SO3CmdToMavros::odom_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "~/imu", rclcpp::SensorDataQoS(), std::bind(&SO3CmdToMavros::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "SO3CmdToMavros node initialized");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToMavros)
