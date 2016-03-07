#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
#include <string>
#include <Eigen/Geometry>
#include <array> 

// ROS related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

// quadrotor_control
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/OutputData.h>

class MAVManager
{
  public:

    // Typedefs
    typedef Eigen::Vector2f    Vec2;
    typedef Eigen::Vector3f    Vec3;
    typedef Eigen::Vector4f    Vec4;
    typedef Eigen::Quaternionf Quat;

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
    uint8_t tracker_status() { return tracker_status_; }

    // Mutators
    bool set_mass(float m);
    void set_need_imu(bool flag)  {need_imu_ = flag;} // TODO: Consider not allowing this to be toggled after takeoff
    void set_need_odom(bool flag) {need_odom_ = flag;} // TODO: Consider not allowing this to be toggled after takeoff
    void set_use_attitude_safety_catch(bool flag) {use_attitude_safety_catch_ = flag;}

    // Home Methods
    bool setHome();                 // Uses the current position and yaw
    bool goHome();
    bool land();

    // Movement
    bool takeoff();

    bool goTo(float x, float y, float z, float yaw, float v_des = 0, float a_des = 0);
    bool goTo(Vec4 xyz_yaw, Vec2 v_and_a_des = Vec2::Zero());
    bool goTo(Vec3 xyz, float yaw, Vec2 v_and_a_des = Vec2::Zero());
    bool goTo(Vec3 xyz, Vec2 v_and_a_des = Vec2::Zero());  // Uses Current yaw

    bool setDesVelInWorldFrame(float x, float y, float z, float yaw);
    bool setDesVelInBodyFrame(float x, float y, float z, float yaw);
    // Yaw control
    bool goToYaw(float);

    // Waypoints
    void clearWaypoints();
    void addWaypoint();

    bool setPositionCommand(const quadrotor_msgs::PositionCommand &cmd);
    bool setSO3Command(const quadrotor_msgs::SO3Command &cmd);
    bool useNullTracker();

    // Monitoring
    bool have_recent_odom(), have_recent_imu(), have_recent_output_data();
    float voltage() {return voltage_;}
    float pressure_height() {return pressure_height_;}
    float pressure_dheight() {return pressure_dheight_;}
    float* magnetic_field() {return magnetic_field_;}
    std::array<uint8_t,8> radio() {return radio_;}

    // Try this
    bool transition(const std::string &tracker_str);

    // Safety
    bool hover();
    bool ehover();
    bool set_motors(bool);
    bool motors() {return motors_;}
    bool eland();
    bool estop();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg);
    void heartbeat_cb(const std_msgs::Empty::ConstPtr &msg);
    void tracker_status_cb(const quadrotor_msgs::TrackerStatus::ConstPtr &msg);
    void heartbeat();

    std::string active_tracker_;
    uint8_t tracker_status_;
    //bool transition(const std::string &tracker_str);

    ros::Time last_odom_t_, last_imu_t_, last_output_data_t_, last_heartbeat_t_;

    Vec3 pos_, vel_;
    float mass_;
    const float kGravity_;
    Quat odom_q_, imu_q_;
    float yaw_, yaw_dot_;
    float takeoff_height_;
    float max_attitude_angle_;

    Vec3 home_, goal_;
    float goal_yaw_, home_yaw_;

    bool need_imu_, need_output_data_, need_odom_, use_attitude_safety_catch_;
    bool home_set_, serial_, motors_;
    float voltage_, pressure_height_, pressure_dheight_, magnetic_field_[3];
    std::array<uint8_t, 8> radio_;

    // Publishers
    ros::Publisher
      pub_goal_min_jerk_,
      pub_goal_line_tracker_distance_,
      pub_goal_velocity_,
      pub_goal_position_velocity_,
      pub_motors_,
      pub_estop_,
      pub_goal_yaw_,
      pub_so3_command_,
      pub_position_command_,
      pub_pwm_command_;

    // Subscribers
    ros::Subscriber
      odom_sub_,
      imu_sub_,
      output_data_sub_,
      heartbeat_sub_,
      tracker_status_sub_;

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
