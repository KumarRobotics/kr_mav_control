#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
#include <Eigen/Geometry>
#include <array>
#include <string>

// ROS related
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

// kr_mav_control
#include <kr_mav_msgs/OutputData.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <kr_mav_msgs/TRPYCommand.h>
#include <kr_tracker_msgs/CircleTrackerAction.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/LissajousAdderAction.h>
#include <kr_tracker_msgs/LissajousTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>

namespace kr_mav_manager
{
class MAVManager
{
 public:
  // Typedefs
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

  MAVManager(std::string ns = "");

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
  void set_need_imu(bool flag) { need_imu_ = flag; }    // TODO: Consider not allowing this to be toggled after takeoff
  void set_need_odom(bool flag) { need_odom_ = flag; }  // TODO: Consider not allowing this to be toggled after takeoff
  void set_use_attitude_safety_catch(bool flag) { use_attitude_safety_catch_ = flag; }

  // Home Methods
  bool setHome();  // Uses the current position and yaw
  bool goHome();
  bool land();

  // Movement
  bool takeoff();

  bool goTo(float x, float y, float z, float yaw, float v_des = 0.0f, float a_des = 0.0f, bool relative = false);
  bool goTo(Vec4 xyz_yaw, Vec2 v_and_a_des = Vec2::Zero());
  bool goTo(Vec3 xyz, float yaw, Vec2 v_and_a_des = Vec2::Zero());
  bool goTo(Vec3 xyz, Vec2 v_and_a_des = Vec2::Zero());  // Uses Current yaw

  bool goToTimed(float x, float y, float z, float yaw, float v_des = 0.0f, float a_des = 0.0f, bool relative = false,
                 ros::Duration duration = ros::Duration(0), ros::Time start_time = ros::Time::now());

  bool setDesVelInWorldFrame(float x, float y, float z, float yaw, bool use_position_feedback = false);
  bool setDesVelInBodyFrame(float x, float y, float z, float yaw, bool use_position_feedback = false);

  // Yaw control
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
  bool setPositionCommand(const kr_mav_msgs::PositionCommand &cmd);
  bool setSO3Command(const kr_mav_msgs::SO3Command &cmd);
  bool setTRPYCommand(const kr_mav_msgs::TRPYCommand &cmd);
  bool useNullTracker();

  // Monitoring
  bool have_recent_odom(), have_recent_imu(), have_recent_output_data();
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

  bool transition(const std::string &tracker_str);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> ClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::CircleTrackerAction> CircleClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LissajousTrackerAction> LissajousClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LissajousAdderAction> CompoundLissajousClientType;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  void tracker_done_callback(const actionlib::SimpleClientGoalState &state,
                             const kr_tracker_msgs::LineTrackerResultConstPtr &result);
  void circle_tracker_done_callback(const actionlib::SimpleClientGoalState &state,
                                    const kr_tracker_msgs::CircleTrackerResultConstPtr &result);
  void lissajous_tracker_done_callback(const actionlib::SimpleClientGoalState &state,
                                       const kr_tracker_msgs::LissajousTrackerResultConstPtr &result);
  void lissajous_adder_done_callback(const actionlib::SimpleClientGoalState &state,
                                     const kr_tracker_msgs::LissajousAdderResultConstPtr &result);

  void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
  void output_data_cb(const kr_mav_msgs::OutputData::ConstPtr &msg);
  void heartbeat_cb(const std_msgs::Empty::ConstPtr &msg);
  void tracker_status_cb(const kr_tracker_msgs::TrackerStatus::ConstPtr &msg);
  void heartbeat();

  std::string active_tracker_;

  Status status_;

  ros::Time last_odom_t_, last_imu_t_, last_output_data_t_, last_heartbeat_t_;

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

  // Actionlibs
  ClientType line_tracker_distance_client_;
  ClientType line_tracker_min_jerk_client_;
  CircleClientType circle_tracker_client_;
  LissajousClientType lissajous_tracker_client_;
  CompoundLissajousClientType lissajous_adder_client_;

  // Publishers
  ros::Publisher pub_motors_, pub_estop_, pub_goal_yaw_, pub_goal_velocity_, pub_so3_command_, pub_trpy_command_,
      pub_position_command_, pub_status_, pub_pwm_command_;

  // Subscribers
  ros::Subscriber odom_sub_, imu_sub_, output_data_sub_, heartbeat_sub_, tracker_status_sub_;

  // Services
  ros::ServiceClient srv_transition_;
};

}  // namespace kr_mav_manager
#endif /* MANAGER_H */
