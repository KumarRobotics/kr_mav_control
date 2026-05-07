#include <kr_betaflight_interface/crsf/crsf_bridge.h>

#include <Eigen/Dense>
#include <kr_betaflight_interface/crsf/crsf_channel_mapping.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace crsf_bridge {

CrsfBridge::CrsfBridge(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      logger_(node->get_logger()),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
      time_last_crsf_msg_sent_(node_->now()),
      time_last_battery_voltage_received_(node_->now()),
      time_last_active_control_command_received_(),
      bridge_state_(BridgeState::KILL),
      control_mode_(ControlMode::NONE),
      arming_counter_(0),
      battery_voltage_(0.0),
      bridge_armed_(false),
      rc_was_disarmed_once_(false),
      destructor_invoked_(false) {
  serial_port_ = std::make_unique<CrsfSerialPort>();
  if (!loadParameters()) {
    RCLCPP_ERROR(logger_, "Could not load parameters.");
    rclcpp::shutdown();
    return;
  }

  if (!serial_port_->setUpCrsfSerialPort(port_name_, node->get_clock())) {
    rclcpp::shutdown();
    return;
  }

  // Set callback for received CRSF messages
  serial_port_->setMessageCallback([this](const CrsfMsg& msg) {
    this->handleReceivedCrsfMessage(msg);
  });

  try {
    watchdog_thread_ = std::thread(&CrsfBridge::watchdogThread, this);
  } catch (...) {
    RCLCPP_ERROR(
        logger_,
        "Could not successfully start watchdog thread. Exiting CrsfBridge.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(logger_, "CrsfBridge initialized");
}

CrsfBridge::~CrsfBridge() {
  destructor_invoked_ = true;
  serial_port_->stopReceiverThread();

  stop_watchdog_thread_ = true;
  watchdog_thread_.join();
  setBridgeState(BridgeState::OFF);

  // Send disarming CRSF message for safety
  CrsfMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  rclcpp::Rate loop_rate(110.0);
  for (int i = 0; i < kSmoothingFailRepetitions_; i++) {
  serial_port_->transmitSerialCrsfMessage(shut_down_message);
    loop_rate.sleep();
  }

  serial_port_->disconnectSerialPort();
}

void CrsfBridge::watchdogThread() {
  rclcpp::Rate watchdog_rate(110.0);
  while (rclcpp::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    const rclcpp::Time time_now = node_->now();
    if (bridge_state_ == BridgeState::RC_FLIGHT &&
        (time_now - time_last_rc_msg_received_).seconds() > rc_timeout_) {
      RCLCPP_WARN(
          logger_,
          "Remote control was active but no message from it was received "
          "within timeout (%f s).",
          rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }
    if (bridge_state_ == BridgeState::KILL) {
      CrsfMsg kill_msg;
      kill_msg.setArmStateDisarmed();
      sendCrsfMessageToSerialPort(kill_msg);
      if (isBridgeArmed()) {
        disarmBridge();
      }
    }
  }
}

void CrsfBridge::handleReceivedCrsfMessage(const CrsfMsg &received_crsf_msg)
{
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);

    time_last_rc_msg_received_ = node_->now();

    if (received_crsf_msg.isKillSwitch())
    {
      if (bridge_state_ != BridgeState::KILL)
      {
        setBridgeState(BridgeState::KILL);
        RCLCPP_INFO(logger_, "Kill switch ON");
      }      
    }
    else
    {
        if (bridge_state_ == BridgeState::KILL)
        {
            setBridgeState(BridgeState::OFF);
            RCLCPP_WARN(logger_, "Kill switch OFF");
        }
    }

    if (bridge_state_ == BridgeState::KILL)
    {
      // send disarm to FC to kill motors
    //   sendCrsfMessageToSerialPort(received_crsf_msg);

      return;
    }

    if (received_crsf_msg.isArmed()) 
    {
      if (!rc_was_disarmed_once_) {
        // This flag prevents that the vehicle can be armed if the RC is armed
        // on startup of the bridge
        RCLCPP_WARN_THROTTLE(
            logger_, *node_->get_clock(), 1000,
            "RC needs to be disarmed once before it can take over control");
        return;
      }

      // Immediately go into RC_FLIGHT state since RC always has priority
      if (bridge_state_ != BridgeState::RC_FLIGHT) {
        setBridgeState(BridgeState::RC_FLIGHT);
        RCLCPP_INFO(logger_, "Control authority taken over by remote control.");
      }
      sendCrsfMessageToSerialPort(received_crsf_msg);
      control_mode_ = received_crsf_msg.getControlMode();
    }
    else if (bridge_state_ == BridgeState::RC_FLIGHT)
    {
      // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the
      // state to AUTONOMOUS_FLIGHT
      // In case there are valid control commands, the bridge will stay in
      // AUTONOMOUS_FLIGHT, otherwise the watchdog will set the state to OFF      
      RCLCPP_INFO(logger_, "Control authority returned by remote control.");
      if (isBridgeArmed())
      {
        RCLCPP_INFO(logger_, "Bridge armed, setting bridge state to AUTONOMOUS_FLIGHT");
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      }
      else
      {
        // When switching the bridge state to off, our watchdog ensures that a
        // disarming off message is sent
        RCLCPP_INFO(logger_, "Bridge not armed, setting bridge state to OFF");
        setBridgeState(BridgeState::OFF);
      }
    }
    else if (!rc_was_disarmed_once_)
    {
      RCLCPP_INFO(
          logger_,
          "Received disarmed RC message but RC was not disarmed once, ignoring it");
      rc_was_disarmed_once_ = true;
    }
    else if (bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT) {
      // not killed, not armed, not autonomous flight
      sendCrsfMessageToSerialPort(received_crsf_msg);
    }

    // Main mutex is unlocked here because it goes out of scope
  }

//   received_crsf_msg_pub_.publish(received_crsf_msg.toRosMessage());
}

void CrsfBridge::controlCommandCallback(
    const kr_mav_msgs::msg::SO3Command::ConstPtr &msg,
    const Eigen::Quaterniond &odom_q) 
{
  std::lock_guard<std::mutex> main_lock(main_mutex_);
  if (destructor_invoked_)
  {
    return;
  }
  time_last_active_control_command_received_ = node_->now();
  if (!isBridgeArmed() || bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
  {    
    if (!isBridgeArmed() && msg->aux.enable_motors &&
        bridge_state_ != BridgeState::RC_FLIGHT)
    {
      RCLCPP_WARN(
          logger_,
          "Received active control command but crsf bridge is not armed. "
          "Please arm the bridge before sending control commands.");
    }
    return;
  }
  CrsfMsg crsf_msg_to_send;
  {
    crsf_msg_to_send = generateCrsfMessageFromSO3Command(msg, odom_q);
  }
  if (!msg->aux.enable_motors) 
  {
    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      RCLCPP_INFO(logger_, "Control command received, setting bridge state to OFF");
      setBridgeState(BridgeState::OFF);
    }
    crsf_msg_to_send.setArmStateDisarmed();
  }
  crsf_msg_to_send.limitAllChannelsFeasible();
  sendCrsfMessageToSerialPort(crsf_msg_to_send);
}

void CrsfBridge::sendCrsfMessageToSerialPort(const CrsfMsg& crsf_msg)
{
  CrsfMsg crsf_message_to_send = crsf_msg;

  switch (bridge_state_) {
    case BridgeState::OFF:
      // Disarm vehicle
      crsf_message_to_send.setArmStateDisarmed();
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple
      // messages before arming, we repeat the messages with minimum throttle
      // and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
        // Set thrust to minimum command to make sure FC arms
        crsf_message_to_send.setThrottleCommand(CrsfMsg::kMinCmd);
        crsf_message_to_send.setArmStateArmed();
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      crsf_message_to_send.setArmStateArmed();
      if (use_body_rates_)
      {
        crsf_message_to_send.setControlModeBodyRates();
      } else
      {
        crsf_message_to_send.setControlModeAttitude();
      }
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
    //   RCLCPP_INFO(logger_, "RC MODE: throttle_cmd %d", crsf_message_to_send.channels[crsf_bridge::crsf_channel_mapping::kThrottle]);
      crsf_message_to_send.setControlModeAttitude();
      break;

    case BridgeState::KILL:
      // Disarm vehicle
      crsf_message_to_send.setArmStateDisarmed();
      break;

    default:
      // Disarm the vehicle because this code must be terribly wrong
      crsf_message_to_send.setArmStateDisarmed();
      RCLCPP_WARN(
          logger_, "Bridge is in unknown state, vehicle will be disarmed");
      break;
  }

  if ((node_->now() - time_last_crsf_msg_sent_).seconds() <= 0.006) {
    // An CRSF message takes 3ms to be transmitted by the serial port so let's
    // not stress it too much. This should only happen in case of switching
    // between control commands and rc commands
    if (bridge_state_ == BridgeState::ARMING) {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages
      // with minimum throttle to the flight controller. Since this check
      // prevents the message from being sent out we reduce the counter that
      // was incremented above assuming the message would actually be sent.
      arming_counter_--;
    }
    return;
  }

  crsf_message_to_send.timestamp = node_->now();
  serial_port_->transmitSerialCrsfMessage(crsf_message_to_send);
  time_last_crsf_msg_sent_ = node_->now();
}

static std::pair<double, double> solve_quadratic(double a, double b, double c)
{
  const double term1 = -b, term2 = std::sqrt(b * b - 4 * a * c);
  return std::make_pair((term1 + term2) / (2 * a), (term1 - term2) / (2 * a));
}

double CrsfBridge::thrust_model_kartik(double thrust) const
{
  int num_props = 4;
  double avg_thrust = std::max(0.0, thrust) / num_props;

  // Scale thrust to individual rotor velocities (RPM)
  auto rpm_solutions =
      solve_quadratic(thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_,
                      thrust_vs_rpm_cof_c_ - avg_thrust);
  const double omega_avg = std::max(rpm_solutions.first, rpm_solutions.second);

  // Scaling from rotor velocity (RPM) to att_throttle for betaflight
  double throttle =
      (omega_avg - rpm_vs_throttle_linear_coeff_b_) / rpm_vs_throttle_linear_coeff_a_;
  return throttle;
}

CrsfMsg CrsfBridge::generateCrsfMessageFromSO3Command(const kr_mav_msgs::msg::SO3Command::ConstPtr& msg, const Eigen::Quaterniond& odom_q) const
{
  CrsfMsg crsf_msg;

  // set crsf_msg to not killed
  crsf_msg.channels[crsf_bridge::crsf_channel_mapping::kKillSwitch] = 1792;

  crsf_msg.setArmStateArmed();

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  const Eigen::Matrix3d R_cur(odom_q);

  double thrust = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  double throttle = 0.0;
  if (thrust > 1e-5)
  {
    throttle = thrust_model_kartik(thrust);
  }

  // remap throttle (1000 to 2000) to crsf (kMinCmd to kMaxCmd)
  uint16_t throttle_cmd = round(((throttle - 1000) / 1000) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
//   RCLCPP_INFO_THROTTLE(
//       logger_, *node_->get_clock(), 1000,
//       "AUTONOMOUS MODE: thrust: %f throttle: %f throttle_cmd: %d",
//       thrust, throttle, throttle_cmd);
  crsf_msg.setThrottleCommand(throttle_cmd);

  // convert quaternion to euler
  Eigen::Vector3d desired_euler_angles = q_des.toRotationMatrix().eulerAngles(0, 1, 2);

  // sanity check if larger than 3 rad, set to 0
  for (int i = 0; i < 3; i++) {
    if (desired_euler_angles(i) > 3.0 || desired_euler_angles(i) < -3.0)
    {
      desired_euler_angles(i) = 0;
    }
  }

  // parse commands
  uint16_t roll_cmd, pitch_cmd, yaw_cmd;

  if (use_body_rates_)
  {
    // get body rates
    double roll_rate = msg->angular_velocity.x;
    double pitch_rate = msg->angular_velocity.y;

    roll_rate = std::max(-max_roll_rate_, std::min(max_roll_rate_, roll_rate));
    pitch_rate = std::max(-max_pitch_rate_, std::min(max_pitch_rate_, pitch_rate));

    // additional safety check
    roll_rate = std::max(-400.0, std::min(400.0, roll_rate));
    pitch_rate = std::max(-400.0, std::min(400.0, pitch_rate));

    roll_cmd = round((roll_rate + max_roll_rate_) / (2 * max_roll_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    pitch_cmd = round((pitch_rate + max_pitch_rate_) / (2 * max_pitch_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    
    RCLCPP_INFO_THROTTLE(
        logger_, *node_->get_clock(), 1000,
        "AUTONOMOUS MODE: roll_rate: %f pitch_rate: %f roll_cmd: %d pitch_cmd: %d",
        roll_rate, pitch_rate, roll_cmd, pitch_cmd);
  }
  else
  {
    // get body angles
    double yaw, roll, pitch;
    // use tf2 to convert quaternion to euler
    tf2::Matrix3x3(tf2::Quaternion(q_des.x(), q_des.y(), q_des.z(), q_des.w())).getRPY(roll, pitch, yaw);
    // convert to degrees
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    
    // clip max roll
    roll_deg = std::max(-max_roll_angle_, std::min(max_roll_angle_, roll_deg));
    // clip max pitch
    pitch_deg = std::max(-max_pitch_angle_, std::min(max_pitch_angle_, pitch_deg));    

    roll_cmd = round((roll_deg + max_roll_angle_) / (2 * max_roll_angle_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    pitch_cmd = round((pitch_deg + max_pitch_angle_) / (2 * max_pitch_angle_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    
    RCLCPP_INFO_THROTTLE(
        logger_, *node_->get_clock(), 1000,
        "AUTONOMOUS MODE: roll_deg: %f pitch_deg: %f roll_cmd: %d pitch_cmd: %d",
        roll_deg, pitch_deg, roll_cmd, pitch_cmd);
  }

  // always use yaw rate
  double yaw_rate = -msg->angular_velocity.z; // flip sign since FC yaw is inverted (z-down)
  yaw_rate = yaw_rate * 180.0 / M_PI; // convert to degrees
  yaw_rate = std::max(-400.0, std::min(400.0, yaw_rate)); // clip max yaw rate
  yaw_cmd = round((yaw_rate + max_yaw_rate_) / (2 * max_yaw_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
  
  RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "AUTONOMOUS MODE: yaw_rate: %f yaw_cmd: %d",
      yaw_rate, yaw_cmd);

  crsf_msg.setRollCommand(roll_cmd);
  crsf_msg.setPitchCommand(pitch_cmd);
  crsf_msg.setYawCommand(yaw_cmd);
  
  return crsf_msg;
}

void CrsfBridge::setBridgeState(const BridgeState& desired_bridge_state) {
  switch (desired_bridge_state) {
    case BridgeState::OFF:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::ARMING:
      bridge_state_ = desired_bridge_state;
      arming_counter_ = 0;
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::RC_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::KILL:
      bridge_state_ = desired_bridge_state;
      break;

    default:
      RCLCPP_WARN(
          logger_, "Wanted to switch to unknown bridge state, setting to OFF");
      bridge_state_ = BridgeState::OFF;
  }
}

void CrsfBridge::armBridge() 
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = true;
}

void CrsfBridge::disarmBridge()
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = false;
}

bool CrsfBridge::isBridgeArmed() const
{
  std::lock_guard<std::mutex> arm_lock(arm_mutex_);
  return bridge_armed_;
}

bool CrsfBridge::loadParameters()
{
  node_->declare_parameter("port_name", std::string("/dev/ttyUSB0"));
  node_->declare_parameter("control_command_timeout", 0.5);
  node_->declare_parameter("rc_timeout", 0.5);
  node_->declare_parameter("disable_thrust_mapping", false);
  node_->declare_parameter("max_roll_rate", 400.0);
  node_->declare_parameter("max_pitch_rate", 400.0);
  node_->declare_parameter("max_yaw_rate", 400.0);
  node_->declare_parameter("max_roll_angle", 0.5);
  node_->declare_parameter("max_pitch_angle", 0.5);
  node_->declare_parameter("alpha_vbat_filter", 0.1);
  node_->declare_parameter("perform_thrust_voltage_compensation", true);
  node_->declare_parameter("n_lipo_cells", 3);
  node_->declare_parameter("thrust_vs_rpm_cof_a_", 0.0001);
  node_->declare_parameter("thrust_vs_rpm_cof_b_", 0.0);
  node_->declare_parameter("thrust_vs_rpm_cof_c_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_linear_coeff_a_", 0.0001);
  node_->declare_parameter("rpm_vs_throttle_linear_coeff_b_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_a_", 0.0001);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_b_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_c_", 0.0);
  node_->declare_parameter("use_body_rates", false);
  
  // Get parameters
  bool all_params_loaded = true;

  if (!node_->get_parameter("port_name", port_name_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: port_name");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("control_command_timeout", control_command_timeout_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: control_command_timeout");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rc_timeout", rc_timeout_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rc_timeout");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("disable_thrust_mapping", disable_thrust_mapping_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: disable_thrust_mapping");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("max_roll_rate", max_roll_rate_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: max_roll_rate");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("max_pitch_rate", max_pitch_rate_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: max_pitch_rate");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("max_yaw_rate", max_yaw_rate_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: max_yaw_rate");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("max_roll_angle", max_roll_angle_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: max_roll_angle");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("max_pitch_angle", max_pitch_angle_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: max_pitch_angle");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("alpha_vbat_filter", alpha_vbat_filter_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: alpha_vbat_filter");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("perform_thrust_voltage_compensation", perform_thrust_voltage_compensation_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: perform_thrust_voltage_compensation");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("n_lipo_cells", n_lipo_cells_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: n_lipo_cells");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("thrust_vs_rpm_cof_a_", thrust_vs_rpm_cof_a_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: thrust_vs_rpm_cof_a_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("thrust_vs_rpm_cof_b_", thrust_vs_rpm_cof_b_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: thrust_vs_rpm_cof_b_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("thrust_vs_rpm_cof_c_", thrust_vs_rpm_cof_c_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: thrust_vs_rpm_cof_c_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rpm_vs_throttle_linear_coeff_a_", rpm_vs_throttle_linear_coeff_a_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rpm_vs_throttle_linear_coeff_a_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rpm_vs_throttle_linear_coeff_b_", rpm_vs_throttle_linear_coeff_b_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rpm_vs_throttle_linear_coeff_b_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rpm_vs_throttle_quadratic_coeff_a_", rpm_vs_throttle_quadratic_coeff_a_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rpm_vs_throttle_quadratic_coeff_a_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rpm_vs_throttle_quadratic_coeff_b_", rpm_vs_throttle_quadratic_coeff_b_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rpm_vs_throttle_quadratic_coeff_b_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("rpm_vs_throttle_quadratic_coeff_c_", rpm_vs_throttle_quadratic_coeff_c_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: rpm_vs_throttle_quadratic_coeff_c_");
    all_params_loaded = false;
  }
  if (!node_->get_parameter("use_body_rates", use_body_rates_)) {
    RCLCPP_ERROR(logger_, "Failed to load parameter: use_body_rates");
    all_params_loaded = false;
  }

  if (!all_params_loaded) {
    RCLCPP_ERROR(logger_, "Failed to load one or more parameters.");
    return false;
  }
  return true;
}

}  // namespace crsf_bridge
