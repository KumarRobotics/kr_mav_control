// kr_mav_manager
#include <kr_mav_manager/manager.hpp>

// Standard C++
#include <math.h>
#include <map>

// ROS2 related
// equivalent to #include <tf/transform_datatypes.h>
#include "tf2/utils.h"

namespace kr_mav_manager
{
// Strings
static const std::string line_tracker_distance("kr_trackers/LineTrackerDistance");
static const std::string line_tracker_min_jerk("kr_trackers/LineTrackerMinJerk");
static const std::string velocity_tracker_str("kr_trackers/VelocityTracker");
static const std::string null_tracker_str("kr_trackers/NullTracker");
static const std::string circle_tracker_str("kr_trackers/CircleTracker");
static const std::string lissajous_tracker_str("kr_trackers/LissajousTracker");
static const std::string lissajous_adder_str("kr_trackers/LissajousAdder");


using namespace std::placeholders;

MAVManager::MAVManager()
  : Node("manager"),
    active_tracker_(""),
    status_(INIT),
    last_odom_t_(0, 0),
    last_imu_t_(0, 0),
    last_output_data_t_(0, 0),
    last_heartbeat_t_(0, 0),
    mass_(-1.0),
    odom_q_(1.0, 0.0, 0.0, 0.0),
    imu_q_(1.0, 0.0, 0.0, 0.0),
    max_attitude_angle_(45.0 / 180.0 * M_PI),
    need_imu_(false),
    need_output_data_(true),
    need_odom_(true),
    use_attitude_safety_catch_(true)
{

  // Declaring parameters
  this->declare_parameter("server_wait_timeout", 0.5f);
  this->declare_parameter("need_imu");
  this->declare_parameter("need_output_data");
  this->declare_parameter("use_attitude_safety_catch");
  this->declare_parameter("max_attitude_angle");
  this->declare_parameter("mass");
  this->declare_parameter("odom_timeout", 0.1f);
  this->declare_parameter("takeoff_height", 0.1);

  // Action Client 
  line_tracker_distance_client_ = rclcpp_action::create_client<LineTracker>(this, 
                                                                            "trackers_manager/line_tracker_distance/LineTracker");
  line_tracker_min_jerk_client_ = rclcpp_action::create_client<LineTracker>(this,
                                                                            "trackers_manager/line_tracker_min_jerk/LineTracker");
  circle_tracker_client_ = rclcpp_action::create_client<CircleTracker>(this, "trackers_manager/circle_tracker/CircleTracker");
  lissajous_tracker_client_ = rclcpp_action::create_client<LissajousTracker>(this,
                                                                             "trackers_manager/lissajous_tracker/LissajousTracker");
  lissajous_adder_client_ = rclcpp_action::create_client<LissajousAdder>(this, "trackers_manager/lissajous_adder/LissajousAdder");

  float server_wait_timout;
  server_wait_timout = this->get_parameter("server_wait_timeout").as_double();
  auto server_wait_timeout_duration = std::chrono::duration<float>(server_wait_timout);

  if(!line_tracker_distance_client_->wait_for_action_server(server_wait_timeout_duration))
  {
    RCLCPP_ERROR(this->get_logger(), "LineTrackerDistance server not found.");
  }

  if(!line_tracker_min_jerk_client_->wait_for_action_server(server_wait_timeout_duration))
  {
    RCLCPP_ERROR(this->get_logger(), "LineTrackerMinJerk server not found.");
  }

  // Optional Trackers
  if(!circle_tracker_client_->wait_for_action_server(server_wait_timeout_duration))
  {
    RCLCPP_WARN(this->get_logger(), "CircleTracker server not found.");
  }

  if(!lissajous_tracker_client_->wait_for_action_server(server_wait_timeout_duration))
  {
    RCLCPP_WARN(this->get_logger(), "LissajousTracker server not found.");
  }

  if(!lissajous_adder_client_->wait_for_action_server(server_wait_timeout_duration))
  {
    RCLCPP_WARN(this->get_logger(), "LissajousAdder server not found.");
  }

  // Publishers
  pub_motors_ = this->create_publisher<std_msgs::msg::Bool>("motors", 10);
  pub_estop_ = this->create_publisher<std_msgs::msg::Empty>("estop", 10);
  pub_so3_command_ = this->create_publisher<kr_mav_msgs::msg::SO3Command>("so3_cmd", 10);
  pub_trpy_command_ = this->create_publisher<kr_mav_msgs::msg::TRPYCommand>("trpy_cmd", 10);
  pub_position_command_ = this->create_publisher<kr_mav_msgs::msg::PositionCommand>("position_cmd", 10);
  pub_status_ = this->create_publisher<std_msgs::msg::UInt8>("status", 10);
  pub_goal_velocity_ = this->create_publisher<kr_trackers_msgs::msg::VelocityGoal>("trackers_manager/velocity_tracker/goal", 10);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&MAVManager::odometry_cb, this, _1));
  heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>("heartbeat", 10, std::bind(&MAVManager::heartbeat_cb, this, _1));
  tracker_status_sub_ = this->create_subscription<kr_trackers_msgs::msg::TrackerStatus>("trackers_manager/status",
                                                                                        10,
                                                                                        std::bind(&MAVManager::tracker_status_cb, this, _1));

  // Services
  srv_transition_ = this->create_client<kr_trackers_msgs::srv::Transition>("trackers_manager/transition");

  srv_transition_->wait_for_service();
  if(!this->transition(null_tracker_str))
  {
    RCLCPP_FATAL(this->get_logger(), "Activation of NullTracker failed.");
  }

  // Load params after we are sure that we have stuff loaded
  if(!this->get_parameter("need_imu", need_imu_))
  {
    RCLCPP_WARN(this->get_logger(), "Couldn't find need_imu param");
  }
  if(need_imu_)
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("quad_decode_msg/imu",
                                                                10,
                                                                std::bind(&MAVManager::imu_cb, this, _1));
  }

  if(!this->get_parameter("need_output_data", need_output_data_))
  {
    RCLCPP_WARN(this->get_logger(), "Couldn't find need_output_data param");
  }
  if(need_output_data_)
  {
    output_data_sub_ = this->create_subscription<kr_mav_msgs::msg::OutputData>("quad_decode_msg/output_data",
                                                                               10,
                                                                               std::bind(&MAVManager::output_data_cb, this, _1));
  }

  if(!this->get_parameter("use_attitide_safety_catch", use_attitude_safety_catch_))
  {
    RCLCPP_WARN(this->get_logger(), "Couldn't find use_attitude_safety_catch param");
  }

  double m;
  if(!this->get_parameter("mass", m))
  {
    RCLCPP_ERROR(this->get_logger(), "MAV Manager requires mass to be set as a param.");
  }
  else if(this->set_mass(m))
  {
    RCLCPP_INFO(this->get_logger(), "MAV Manager using mass = %2.2f.", mass_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Mass failed to set. Perhaps mass <= 0?");
  }

  this->get_parameter("odom_timeout", odom_timeout_);

  if(!this->set_motors(false))
  {
    RCLCPP_ERROR(this->get_logger(), "Could not disable motors");
  }
}


void MAVManager::tracker_done_callback(const LineTrackerGoalHandle::WrappedResult &result)
{
  auto status = result.code;
  RCLCPP_INFO(this->get_logger(), "Goal (%2.2f, %2.2f, %2.2f, %2.2f) finished with state %s after %2.2f s. and %2.2f m.",
              result.result->x, result.result->y, result.result->z, result.result->yaw, result_status[status].c_str(), 
              result.result->duration, result.result->length);
}

void MAVManager::circle_tracker_done_callback(const CircleTrackerGoalHandle::WrappedResult &result)
{
  RCLCPP_INFO(this->get_logger(), "Circle tracking completed after %2.2f seconds", result.result->duration);
}

void MAVManager::lissajous_tracker_done_callback(const LissajousTrackerGoalHandle::WrappedResult &result)
{
  RCLCPP_INFO(this->get_logger(), "Lissajous tracking completed. Duration %2.2f seconds, distance: %2.2f m, now located at (%2.2f, %2.2f, %2.2f, %2.2f).", 
              result.result->duration, result.result->length, result.result->x, result.result->y, 
              result.result->z, result.result->yaw);
}

void MAVManager::lissajous_adder_done_callback(const LissajousAdderGoalHandle::WrappedResult &result)
{
  RCLCPP_INFO(this->get_logger(), "Lissajous tracking completed. Duration %2.2f seconds, distance: %2.2f m, now located at (%2.2f, %2.2f, %2.2f, %2.2f).", 
              result.result->duration, result.result->length, result.result->x, result.result->y, 
              result.result->z, result.result->yaw);
}


void MAVManager::odometry_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  odom_q_ = Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z);
  
  yaw_ = tf2::getYaw(msg->pose.pose.orientation);

  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = this->now();

  this->heartbeat();
}

bool MAVManager::takeoff()
{
  if(!this->have_recent_odom())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot takeoff without odometry.");
    return false;
  }

  if(!this->setHome())
  {
    return false;
  }

  if(!this->motors() || status_ != IDLE)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot takeoff motors are idling.");
    return false;
  }

  // Only takeoff if currently under NULL_TRACKER
  if(active_tracker_.compare(null_tracker_str) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "The Null Tracker must be active before taking off");
    return false;
  }

  // Read takeoff height
  double takeoff_height;
  takeoff_height = this->get_parameter("takeoff_height").as_double();
  takeoff_height_ = takeoff_height;

  if(takeoff_height_ > 3.0f)
  {
    RCLCPP_ERROR(this->get_logger(), "Takeoff Height is Dangerously High");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Initiating launch sequence...");

  auto goal = LineTracker::Goal();
  goal.z = takeoff_height_;
  goal.relative = true;
  auto options = rclcpp_action::Client<LineTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::tracker_done_callback, this, _1);
  line_tracker_distance_client_->async_send_goal(goal, options);

  if(this->transition(line_tracker_distance))
  {
    status_ = FLYING;
    return true;
  }
  else
    return false;
}

bool MAVManager::set_mass(float m)
{
  if(m > 0)
  {
    // TODO: This should update the mass in the controller and everywhere else that is necessary
    mass_ = m;
    return true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Mass must be > 0");
    return false;
  }
}

bool MAVManager::setHome()
{
  if(this->have_recent_odom())
  {
    home_ = pos_;
    home_yaw_ = yaw_;
    home_set_ = true;
    return true;
  }
  else
    RCLCPP_WARN(this->get_logger(), "Cannot set home unless current pose is known.");

  return false;
}

bool MAVManager::goHome()
{
  if(home_set_)
    return this->goTo(home_ + Vec3(0, 0, 0.15), home_yaw_);
  else
  {
    RCLCPP_WARN(this->get_logger(), "Home not set. Cannot go home.");
    return false;
  }
}

bool MAVManager::land()
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "Not landing since the robot is not already flying.");
    return false;
  }

  auto goal = LineTracker::Goal();
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = home_(2);
  std::cout << " landing at " << goal.x << " " << goal.y << " " << goal.z << std::endl;
  auto options = rclcpp_action::Client<LineTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::tracker_done_callback, this, _1);
  line_tracker_distance_client_->async_send_goal(goal, options);

  return this->transition(line_tracker_distance);
}

bool MAVManager::goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative)
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "The robot must be flying before using the goTo method.");
    return false;
  }

  auto goal = LineTracker::Goal();
  goal.x = x;
  goal.y = y;

  goal.relative = relative;
  // Convert relative translation in body frame to global frame
  if(relative)
  {
    goal.x = x * std::cos(yaw_) - y * std::sin(yaw_);
    goal.y = x * std::sin(yaw_) + y * std::cos(yaw_);
  }

  goal.z = z;
  goal.yaw = yaw;
  goal.v_des = v_des;
  goal.a_des = a_des;

  auto options = rclcpp_action::Client<LineTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::tracker_done_callback, this, _1);

  line_tracker_min_jerk_client_->async_send_goal(goal, options);

  return this->transition(line_tracker_min_jerk);
}

bool MAVManager::goToTimed(float x, float y, float z, float yaw, float v_des, float a_des, bool relative,
                           rclcpp::Duration duration, rclcpp::Time t_start)
{
  auto goal = LineTracker::Goal();
  goal.x = x;
  goal.y = y;

  goal.relative = relative;
  // Convert relative translation in body frame to global frame
  if(relative)
  {
    goal.x = x * std::cos(yaw_) - y * std::sin(yaw_);
    goal.y = x * std::sin(yaw_) + y * std::cos(yaw_);
  }

  goal.z = z;
  goal.yaw = yaw;
  goal.duration = duration;
  goal.t_start = t_start;
  goal.v_des = v_des;
  goal.a_des = a_des;

  auto options = rclcpp_action::Client<LineTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::tracker_done_callback, this, _1);

  line_tracker_min_jerk_client_->async_send_goal(goal, options);

  RCLCPP_INFO(this->get_logger(), "Going to {%2.2f, %2.2f, %2.2f, %2.2f}%s with duration %2.2f", x, y, z, yaw,
              (relative ? " relative to the current position." : ""), duration.seconds());

  return this->transition(line_tracker_min_jerk);
}

bool MAVManager::goTo(Vec4 xyz_yaw, Vec2 v_and_a_des)
{
  return this->goTo(xyz_yaw(0), xyz_yaw(1), xyz_yaw(2), xyz_yaw(3), v_and_a_des(0), v_and_a_des(1));
}

bool MAVManager::goTo(Vec3 xyz, float yaw, Vec2 v_and_a_des)
{
  return this->goTo(xyz(0), xyz(1), xyz(2), yaw, v_and_a_des(0), v_and_a_des(1));
}

bool MAVManager::goTo(Vec3 xyz, Vec2 v_and_a_des)
{
  return this->goTo(xyz(0), xyz(1), xyz(2), yaw_, v_and_a_des(0), v_and_a_des(1));
}

bool MAVManager::goToYaw(float yaw)
{
  return this->goTo(pos_(0), pos_(1), pos_(2), yaw);
}

bool MAVManager::circle(float Ax, float Ay, float T, float duration)
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "The robot must be flying before using the circle method.");
    return false;
  }

  auto goal = CircleTracker::Goal();
  goal.ax = Ax;
  goal.ay = Ay;
  goal.t = T;
  goal.duration = duration;

  auto options = rclcpp_action::Client<CircleTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::circle_tracker_done_callback, this, _1);

  circle_tracker_client_->async_send_goal(goal, options);

  return this->transition(circle_tracker_str);
}

bool MAVManager::lissajous(float x_amp, float y_amp, float z_amp, float yaw_amp, float x_num_periods,
                           float y_num_periods, float z_num_periods, float yaw_num_periods, float period,
                           float num_cycles, float ramp_time)
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "The robot must be flying to execute Lissajous");
    return false;
  }

  auto goal = LissajousTracker::Goal();
  goal.x_amp = x_amp;
  goal.y_amp = y_amp;
  goal.z_amp = z_amp;
  goal.yaw_amp = yaw_amp;
  goal.x_num_periods = x_num_periods;
  goal.y_num_periods = y_num_periods;
  goal.z_num_periods = z_num_periods;
  goal.yaw_num_periods = yaw_num_periods;
  goal.period = period;
  goal.num_cycles = num_cycles;
  goal.ramp_time = ramp_time;

  auto options = rclcpp_action::Client<LissajousTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::lissajous_tracker_done_callback, this, _1);

  lissajous_tracker_client_->async_send_goal(goal, options);

  return this->transition(lissajous_tracker_str);
}

bool MAVManager::compound_lissajous(float x_amp[2], float y_amp[2], float z_amp[2], float yaw_amp[2],
                                    float x_num_periods[2], float y_num_periods[2], float z_num_periods[2],
                                    float yaw_num_periods[2], float period[2], float num_cycles[2], float ramp_time[2])
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "The robot must be flying to execute Lissajous.");
    return false;
  }

  auto goal = LissajousAdder::Goal();
  goal.x_amp[0] = x_amp[0];
  goal.x_amp[1] = x_amp[1];
  goal.y_amp[0] = y_amp[0];
  goal.y_amp[1] = y_amp[1];
  goal.z_amp[0] = z_amp[0];
  goal.z_amp[1] = z_amp[1];
  goal.yaw_amp[0] = yaw_amp[0];
  goal.yaw_amp[1] = yaw_amp[1];
  goal.x_num_periods[0] = x_num_periods[0];
  goal.x_num_periods[1] = x_num_periods[1];
  goal.y_num_periods[0] = y_num_periods[0];
  goal.y_num_periods[1] = y_num_periods[1];
  goal.z_num_periods[0] = z_num_periods[0];
  goal.z_num_periods[1] = z_num_periods[1];
  goal.yaw_num_periods[0] = yaw_num_periods[0];
  goal.yaw_num_periods[1] = yaw_num_periods[1];
  goal.period[0] = period[0];
  goal.period[1] = period[1];
  goal.num_cycles[0] = num_cycles[0];
  goal.num_cycles[1] = num_cycles[1];
  goal.ramp_time[0] = ramp_time[0];
  goal.ramp_time[1] = ramp_time[1];

  auto options = rclcpp_action::Client<LissajousAdder>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::lissajous_adder_done_callback, this, _1);

  lissajous_adder_client_->async_send_goal(goal, options);

  return this->transition(lissajous_adder_str);
}

// World Velocity Commands
bool MAVManager::setDesVelInWorldFrame(float x, float y, float z, float yaw, bool use_positio_feedback)
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "The robot must be flying with motors on before setting desired velocity.");
    return false;
  }

  auto goal = kr_trackers_msgs::msg::VelocityGoal();
  goal.header.stamp = this->now();
  goal.vx = x;
  goal.vy = y;
  goal.vz = z;
  goal.vyaw = yaw;
  goal.use_position_gains = use_positio_feedback;

  pub_goal_velocity_->publish(goal);

  RCLCPP_INFO(this->get_logger(), "Desired World velocity: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.vx, goal.vy, goal.vz, goal.vyaw);

  // Since this could be called quite often,
  // only try to transition if it is not the active tracker.
  if(active_tracker_.compare(velocity_tracker_str) != 0)
  {
    return this->transition(velocity_tracker_str);
  }

  return true;
}

// Body Velocity Commands
bool MAVManager::setDesVelInBodyFrame(float x, float y, float z, float yaw, bool use_position_feedback)
{
  Vec3 vel(x, y, z);
  vel = odom_q_ * vel;
  return this->setDesVelInWorldFrame(vel(0), vel(1), vel(2), yaw, use_position_feedback);
}

bool MAVManager::setPositionCommand(const kr_mav_msgs::msg::PositionCommand &msg)
{
  // TODO: Need to keep publishing a position command if there is no update
  // Otherwise, no so3_command will be published

  if(this->motors() && status_ != ESTOP)
  {
    bool flag(true);

    // Since this could be called quite often,
    // only try to transition if it is not the active tracker.
    if(active_tracker_.compare(null_tracker_str) != 0)
      flag = this->transition(null_tracker_str);

    if(flag)
      pub_position_command_->publish(msg);

    return flag;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Refusing to set PositionCommand since motors are off or the robot is not flying.");
    return false;
  }
}

bool MAVManager::setSO3Command(const kr_mav_msgs::msg::SO3Command &msg)
{
  // Note: To enable motors, the motors method must be used
  if(!this->motors())
  {
    RCLCPP_WARN(this->get_logger(), "Refusing to publish an SO3Command until motors have been enabled using the motors method.");
    return false;
  }

  // Since this could be called quite often,
  // only try to transition if it is not the active tracker.
  bool flag(true);
  if(active_tracker_.compare(null_tracker_str) != 0)
    flag = this->transition(null_tracker_str);

  if(flag)
    pub_so3_command_->publish(msg);

  return flag;
}

bool MAVManager::setTRPYCommand(const kr_mav_msgs::msg::TRPYCommand &msg)
{
  // Note: To enable motors, the motors method must be used
  if(!this->motors())
  {
    RCLCPP_WARN(this->get_logger(), "Refusing to publish an SO3Command until motors have been enabled using the motors method.");
    return false;
  }

  // Since this could be called quite often,
  // only try to transition if it is not the active tracker.
  bool flag(true);
  if(active_tracker_.compare(null_tracker_str) != 0)
    flag = this->transition(null_tracker_str);

  if(flag)
    pub_trpy_command_->publish(msg);

  return flag;
}

bool MAVManager::useNullTracker()
{
  if(active_tracker_.compare(null_tracker_str) != 0)
    return this->transition(null_tracker_str);

  return true;
}

bool MAVManager::set_motors(bool motors)
{
  // Do nothing if we ask for motors to be turned on when they already are on
  if(motors && this->motors())
    return true;

  bool null_tkr = this->transition(null_tracker_str);

  // Make sure null_tracker is active before starting motors. If turning motors
  // off, continue anyway.
  if(motors && !null_tkr)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transition to null_tracker before starting motors");
    return false;
  }

  // Enable/Diable motors
  if(motors)
    RCLCPP_INFO(this->get_logger(), "Motors starting...");
  else
    RCLCPP_INFO(this->get_logger(), "Motors stopping...");

  auto motors_cmd = std_msgs::msg::Bool();
  motors_cmd.data = motors;
  pub_motors_->publish(motors_cmd);

  // Publish a couple so3_commands to ensure motors are or are not spinning
  auto so3_cmd = kr_mav_msgs::msg::SO3Command();
  so3_cmd.header.stamp = this->now();
  so3_cmd.force.z = FLT_MIN;
  so3_cmd.orientation.w = 1.0;
  so3_cmd.aux.enable_motors = motors;

  auto trpy_cmd = kr_mav_msgs::msg::TRPYCommand();
  trpy_cmd.thrust = FLT_MIN;
  trpy_cmd.aux.enable_motors = motors;

  // Queue a few to make sure the signal gets through.
  // Also, the crazyflie interface throttles commands to 30 Hz, so this needs
  // to have sufficient duration.
  rclcpp::Rate loop_rate(100.0);  // confirm the loop rate of 100Hz
  for(int i = 0; i < 10; i++)
  {
    pub_so3_command_->publish(so3_cmd);
    pub_trpy_command_->publish(trpy_cmd);
    loop_rate.sleep();
  }

  motors_ = motors;
  status_ = motors_ ? IDLE : MOTORS_OFF;
  return true;
}

void MAVManager::imu_cb(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  last_imu_t_ = this->now();

  imu_q_ = Quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  this->heartbeat();
}

void MAVManager::output_data_cb(kr_mav_msgs::msg::OutputData::ConstSharedPtr msg)
{
  last_output_data_t_ = this->now();
  last_imu_t_ = this->now();

  imu_q_ = Quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  voltage_ = msg->voltage;
  pressure_dheight_ = msg->pressure_dheight;
  pressure_height_ = msg->pressure_height;
  magnetic_field_[0] = msg->magnetic_field.x;
  magnetic_field_[1] = msg->magnetic_field.y;
  magnetic_field_[2] = msg->magnetic_field.z;
  for(uint8_t i = 0; i < radio_.size(); i++)
    radio_[i] = msg->radio_channel[i];

  this->heartbeat();
}

void MAVManager::tracker_status_cb(kr_trackers_msgs::msg::TrackerStatus::ConstSharedPtr msg)
{
  active_tracker_ = msg->tracker;
}

void MAVManager::heartbeat_cb(std_msgs::msg::Empty::ConstSharedPtr msg)
{
  (void)msg;
  this->heartbeat();
}

// TODO: This should be done in a separate thread
void MAVManager::heartbeat()
{
  const float freq = 10; // Hz

  // Only need to do monitoring at the specified frequency
  rclcpp::Time t = this->now();
  float dt = (t - last_heartbeat_t_).seconds();
  if(dt < 1 / freq)
    return;
  else
    last_heartbeat_t_ = t;

  // Publish the status
  auto status_msg = std_msgs::msg::UInt8();
  status_msg.data = status_;
  pub_status_->publish(status_msg);

  // Checking for odom
  if(this->motors() && need_odom_ && !this->have_recent_odom())
  {
    RCLCPP_WARN(this->get_logger(), "No recent odometry!");
    this->eland();
  }

  // Checking for imu
  if(this->motors() && need_imu_ && !this->have_recent_imu())
  {
    RCLCPP_WARN(this->get_logger(), "No recent imu!");
    this->eland();
  }

  if(use_attitude_safety_catch_)
  {
    // TODO: Currently this can be overridden if client is continually updating
    // position commands. Maybe put a timeout, but it could be dangerous? Maybe
    // require a call to hover before exiting a safety catch mode?

    // Convert quaternions to tf so we can compute Euler angles, etc
    tf2::Quaternion imu_q(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
    tf2::Quaternion odom_q(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

    // determine a geodesic angle from hover at the same yaw
    double yaw, pitch, roll;
    tf2::Matrix3x3 R;

    // TODO: Don't check mocap odom if we have imu feedback

    // If we don't have IMU feedback, imu_q_ will be the identity rotation
    tf2::Matrix3x3(imu_q).getEulerYPR(yaw, pitch, roll);
    R.setEulerYPR(0, pitch, roll);
    float imu_geodesic = std::fabs(std::acos(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1)));

    tf2::Matrix3x3(odom_q).getEulerYPR(yaw, pitch, roll);
    R.setEulerYPR(0, pitch, roll);
    float odom_geodesic = std::fabs(std::acos(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1)));
    
    float geodesic = std::max(imu_geodesic, odom_geodesic);

    static float attitude_limit_timer = 0;
    if(geodesic > max_attitude_angle_)
      attitude_limit_timer += dt;
    else
      attitude_limit_timer = 0;
    
    if(attitude_limit_timer > 0.5f)
    {
      // Reset the timer so we don't keep calling ehover
      attitude_limit_timer = 0;
      RCLCPP_WARN(this->get_logger(), "Attitude exceeded threshold of %2.2f deg! Geodesic = %2.2f deg. Entering emergency hover.",
                  max_attitude_angle_ * 180.0f / M_PI, geodesic * 180.f/ M_PI);
      this->ehover();
    }
  }

  if(this->have_recent_output_data())
  {
    if(voltage_ < 10.0f)  // Note: Asctec firmware uses 9V
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Battery voltage = %2.2f V", voltage_); //time in milliseconds
    }
  }

  // TODO: Incorporate bounding box constraints. Something along the lines of the following.
  // We may want to use a timer in case the boundary is crossed slowly.
  //
  //
  // bool flag = in_bounding_box(this->pos());
  // static bool lastflag = flag;
  // if (!flag && flag != lastflag)
  // {
  //   this->ehover();
  // }
  // lastflag = flag;

  // TODO: Also something along the following
  // // TF broadcaster to broadcast the quadrotor frame
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(pos_.x, pos_.y, pos_.z) );
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/simulator", "/quadrotor"));
}

bool MAVManager::eland()
{
  // TODO: This should also check a height threshold or something along those
  // lines. For example, if the rotors are idle and the robot hasn't even 
  // left the ground, we don't want them to spin up faster.
  if(this->motors() && (status_  == FLYING || status_ == ELAND))
  {
    RCLCPP_WARN(this->get_logger(), "Emergency Land");

    auto goal = kr_mav_msgs::msg::PositionCommand();
    goal.acceleration.z = -0.45f;
    goal.yaw = yaw_;

    if(this->setPositionCommand(goal))
    {
      status_ = ELAND;
      return true;
    }
    else
      return false;
  }
  else
    return this->set_motors(false);
}

bool MAVManager::estop()
{
  RCLCPP_WARN(this->get_logger(), "E-STOP");
  auto estop_cmd = std_msgs::msg::Empty();
  pub_estop_->publish(estop_cmd);

  // Disarm motors
  if(this->set_motors(false))
  {
    status_ = ESTOP;
    return true;
  }
  else 
    return false;
}

bool MAVManager::hover()
{
  const float a_des(0.8);   //, yaw_a_des(0.1);

  const float v_norm = vel_.norm();

  if(v_norm > 1e-2)
  {
    Vec3 dir = vel_ / v_norm;

    // Acceleration should be opposite the velocity component
    const Vec3 acc = -dir * a_des;

    // acc(3) = -copysign(yaw_a_des, yaw_dot_);

    // vf = vo + at -> t = (vf - vo) / a
    const float t = v_norm / a_des;
    // float t_yaw = - yaw_dot_ / yaw_a_des;

    // xf = xo + vo * t + 1/2 * a * t^2
    Vec4 goal(pos_(0) + vel_(0) * t + 0.5f * acc(0) * t * t,
              pos_(1) + vel_(1) * t + 0.5f * acc(1) * t * t,
              pos_(2) + vel_(2) * t + 0.5f * acc(2) * t * t,
              yaw_); // + yaw_dot_ * t_yaw + 0.5 * yaw_a_des * t_yaw * t_yaw);

    Vec2 v_and_a_des(std::sqrt(vel_.dot(vel_)), a_des);

    RCLCPP_DEBUG(this->get_logger(), "Coasting to hover...");
    return this->goTo(goal, v_and_a_des);
  }

  RCLCPP_DEBUG(this->get_logger(), "Hovering in place...");
  return this->goTo(pos_(0), pos_(1), pos_(2), yaw_);
}

bool MAVManager::ehover()
{
  if(!this->motors() || status_ != FLYING)
  {
    RCLCPP_WARN(this->get_logger(), "Will not call emergency hover unless the robot is already flying");
    return false;
  }

  auto goal = LineTracker::Goal();
  goal.x = pos_(0);
  goal.y = pos_(1);
  goal.z = pos_(2);

  auto options = rclcpp_action::Client<LineTracker>::SendGoalOptions();
  options.result_callback = std::bind(&MAVManager::tracker_done_callback, this, _1);

  line_tracker_distance_client_->async_send_goal(goal, options);

  return this->transition(line_tracker_distance);
}

bool MAVManager::transition(const std::string& tracker_str)
{
  //usleep(100000);

  auto transition_cmd = std::make_shared<kr_trackers_msgs::srv::Transition::Request>();
  transition_cmd->tracker = tracker_str;

  auto future = srv_transition_->async_send_request(transition_cmd);

  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if(response->success)
    {
      active_tracker_ = tracker_str;
      RCLCPP_INFO(this->get_logger(), "Current tracker: %s", tracker_str.c_str());
      return true;
    }
  }

  return false;
}


bool MAVManager::have_recent_odom()
{
  return (this->now() - last_odom_t_).seconds() < odom_timeout_;
}

bool MAVManager::have_recent_imu()
{
  return (this->now() - last_imu_t_).seconds() < 0.1;
}

bool MAVManager::have_recent_output_data()
{
  return (this->now() - last_output_data_t_).seconds() < 0.1;
}

}   // namespace kr_mav_manager