#pragma once

// Standard C++ Libraries
#include <Eigen/Geometry>
#include <array>
#include <string>

// ROS2 related
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

// kr_mav_control
#include "kr_mav_msgs/msg/output_data.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_mav_msgs/msg/so3_command.hpp"
#include "kr_mav_msgs/msg/trpy_command.hpp"
#include "kr_trackers_msgs/action/circle_tracker.hpp"
#include "kr_trackers_msgs/action/line_tracker.hpp"
#include "kr_trackers_msgs/action/lissajous_adder.hpp"
#include "kr_trackers_msgs/action/lissajous_tracker.hpp"
#include "kr_trackers_msgs/msg/tracker_status.hpp"
#include "kr_trackers_msgs/msg/velocity_goal.hpp"
#include "kr_trackers_msgs/srv/transition.hpp"

namespace kr_mav_manager
{

using LineTracker = kr_trackers_msgs::action::LineTracker;
using LineTrackerGoalHandle = rclcpp_action::ClientGoalHandle<LineTracker>;
using CircleTracker = kr_trackers_msgs::action::CircleTracker;
using CircleTrackerGoalHandle = rclcpp_action::ClientGoalHandle<CircleTracker>;
using LissajousTracker = kr_trackers_msgs::action::LissajousTracker;
using LissajousTrackerGoalHandle = rclcpp_action::ClientGoalHandle<LissajousTracker>;
using LissajousAdder = kr_trackers_msgs::action::LissajousAdder;
using LissajousAdderGoalHandle = rclcpp_action::ClientGoalHandle<LissajousAdder>;

class MAVManager : public rclcpp::Node
{
public:

  //Typedefs
  typedef Eigen::Vector2f Vec2;
  typedef Eigen::Vector3f Vec3;
  typedef Eigen::Vector4f Vec4;
  typedef Eigen::Quaternionf Quat;

  enum Status
  {
    INIT,
    MOTORS_OFF,
    IDLE,
    ELAND,
    ESTOP,
    FLYING
  };

  MAVManager();
  
  // Accessors
  Vec3 pos() { return pos_; }
  Vec3 vel() { return vel_; }
  Vec3 home() { return home_; }
  float yaw() { return yaw_; }
  float home_yaw() { return home_yaw_; }
  float mass() { return mass_; }
  std::string active_tracker() { return active_tracker_; }
  bool need_imu() { return need_imu_; }
  bool need_odom() { return need_odom_; }
  Status status() { return status_; }

  // Mutators
  bool set_mass(float m);
  void set_need_imu(bool flag) { need_imu_ = flag; }      // TODO: Consider not allowing this to be toggled after takeoff
  void set_need_odom(bool flag) { need_odom_ = flag; }    // TODO: Consider not allowing this to be toggled after takeoff
  void set_use_attitude_safety_catch(bool flag) { use_attitude_safety_catch_ = flag; }

  // Home Methods
  bool setHome(); // Uses the current position and yaw
  bool goHome();
  bool land();

  // Movement
  bool takeoff();

  bool goTo(float x, float y, float z, float yaw, float v_des = 0.0f, float a_des = 0.0f, bool relative = false);
  bool goTo(Vec4 xyz_yaw, Vec2 v_and_a_des = Vec2::Zero());
  bool goTo(Vec3 xyz,  float yaw, Vec2 v_and_a_des = Vec2::Zero());
  bool goTo(Vec3 xyz, Vec2 v_and_a_des = Vec2::Zero());   // Uses current yaw

  bool goToTimed(float x, float y, float z, float yaw, float v_des = 0.0f, float a_des = 0.0f, bool relative = false,
                 rclcpp::Duration duration = rclcpp::Duration(0), rclcpp::Time start_time = rclcpp::Time(0)); //check this->now()
                //  {start_time = this->now();}

  bool setDesVelInWorldFrame(float x, float y, float z, float yaw, bool use_position_feedback = false);
  bool setDesVelInBodyFrame(float x, float y, float z, float yaw, bool use_position_feedback = false);

  // Yaw Control
  bool goToYaw(float);

  bool circle(float Ax, float Ay, float T, float duration);

  // Lissajous Control
  bool lissajous(float x_amp, float y_amp, float z_amp, float yaw_amp, float x_num_periods, float y_num_periods,
                 float z_num_periods, float yaw_num_periods, float period, float num_cycles, float ramp_time);

  // Compound Lissajous Control
  bool compound_lissajous(float x_amp[2], float y_amp[2], float z_amp[2], float yaw_amp[2], float x_num_periods[2],
                          float y_num_periods[2], float z_num_periods[2], float yaw_num_periods[2], float period[2],
                          float num_cycles[2], float ramp_time[2]);

  // Direct low-level control
  bool setPositionCommand(const kr_mav_msgs::msg::PositionCommand &msg);
  bool setSO3Command(const kr_mav_msgs::msg::SO3Command &msg);
  bool setTRPYCommand(const kr_mav_msgs::msg::TRPYCommand &msg);
  bool useNullTracker();

  // Monitoring
  bool have_recent_odom();
  bool have_recent_imu();
  bool have_recent_output_data();
  float voltage() { return voltage_; }
  float pressure_height() { return pressure_height_; }
  float pressure_dheight() { return pressure_dheight_; }
  std::array<float, 3> magnetic_field() { return magnetic_field_; }
  std::array<uint8_t, 8> radio() { return radio_; }

  // Safety
  bool hover();
  bool ehover();
  bool set_motors(bool);
  bool motors() { return motors_; }
  bool eland();
  bool estop();

  bool transition(const std::string& tracker_str);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  typedef rclcpp_action::Client<kr_trackers_msgs::action::LineTracker>::SharedPtr ClientType;
  typedef rclcpp_action::Client<kr_trackers_msgs::action::CircleTracker>::SharedPtr CircleClientType;
  typedef rclcpp_action::Client<kr_trackers_msgs::action::LissajousTracker>::SharedPtr LissajousClientType;
  typedef rclcpp_action::Client<kr_trackers_msgs::action::LissajousAdder>::SharedPtr CompoundLissajousClientType;

  // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("manager");
  // check if you need this node handle

  void tracker_done_callback(const LineTrackerGoalHandle::WrappedResult &result);
  void circle_tracker_done_callback(const CircleTrackerGoalHandle::WrappedResult &result);
  void lissajous_tracker_done_callback(const LissajousTrackerGoalHandle::WrappedResult &result);
  void lissajous_adder_done_callback(const LissajousAdderGoalHandle::WrappedResult &result);

  void odometry_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imu_cb(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void output_data_cb(kr_mav_msgs::msg::OutputData::ConstSharedPtr msg);
  void heartbeat_cb(std_msgs::msg::Empty::ConstSharedPtr msg);
  void tracker_status_cb(kr_trackers_msgs::msg::TrackerStatus::ConstSharedPtr msg);
  void heartbeat();

  std::string active_tracker_;

  Status status_;

  rclcpp::Time last_odom_t_, last_imu_t_, last_output_data_t_, last_heartbeat_t_;

  Vec3 pos_, vel_;
  float mass_;
  Quat odom_q_, imu_q_;
  float yaw_, yaw_dot_;
  float takeoff_height_;
  float max_attitude_angle_;
  float odom_timeout_;

  Vec3 home_, goal_;
  float home_yaw_;

  bool need_imu_, need_output_data_, need_odom_, use_attitude_safety_catch_;
  bool home_set_, motors_;
  float voltage_, pressure_height_, pressure_dheight_;
  std::array<float, 3> magnetic_field_;
  std::array<uint8_t, 8> radio_;

  // Action Clients
  ClientType line_tracker_distance_client_;
  ClientType line_tracker_min_jerk_client_;
  CircleClientType circle_tracker_client_;
  LissajousClientType lissajous_tracker_client_;
  CompoundLissajousClientType lissajous_adder_client_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motors_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_estop_;
  rclcpp::Publisher<kr_mav_msgs::msg::SO3Command>::SharedPtr pub_so3_command_;
  rclcpp::Publisher<kr_mav_msgs::msg::TRPYCommand>::SharedPtr pub_trpy_command_;
  rclcpp::Publisher<kr_mav_msgs::msg::PositionCommand>::SharedPtr pub_position_command_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_status_;
  rclcpp::Publisher<kr_trackers_msgs::msg::VelocityGoal>::SharedPtr pub_goal_velocity_;
  // pub_goal_yaw_ and pub_pwm_command_ defined in ros1 package but not being used

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //odometry_cb
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_; //heartbeat_cb
  rclcpp::Subscription<kr_trackers_msgs::msg::TrackerStatus>::SharedPtr tracker_status_sub_; //tracker_status_cb
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; //imu_cb
  rclcpp::Subscription<kr_mav_msgs::msg::OutputData>::SharedPtr output_data_sub_;

  //Services
  rclcpp::Client<kr_trackers_msgs::srv::Transition>::SharedPtr srv_transition_;

  // Helper variable
  std::map<rclcpp_action::ResultCode, std::string> result_status = {{rclcpp_action::ResultCode::SUCCEEDED, "SUCCEEDED"},
                                                                    {rclcpp_action::ResultCode::ABORTED, "ABORTED"},
                                                                    {rclcpp_action::ResultCode::CANCELED, "CANCELED"},
                                                                    {rclcpp_action::ResultCode::UNKNOWN, "UNKNOWN"}};

};

} // namespace kr_mav_manager