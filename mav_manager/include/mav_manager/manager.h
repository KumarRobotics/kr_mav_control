#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
#include <string>
#include <Eigen/Geometry>

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
    typedef Eigen::Vector2d    Vec2;
    typedef Eigen::Vector3d    Vec3;
    typedef Eigen::Vector4d    Vec4;
    typedef Eigen::Quaterniond Quat;

    MAVManager();

    // Accessors
    Vec3 pos() { return pos_; }
    Vec3 vel() { return vel_; }
    Vec3 home() { return home_; }
    double yaw() { return yaw_; }
    double home_yaw() { return home_yaw_; }
    double mass() { return mass_; }
    std::string active_tracker() { return active_tracker_; }
    bool need_imu() { return need_imu_; }
    bool need_odom() { return need_odom_; }
    short tracker_status() { return tracker_status_; }

    // Mutators
    bool set_mass(double m);
    void set_need_imu(bool flag)  {need_imu_ = flag;} // TODO: Consider not allowing this to be toggled after takeoff
    void set_need_odom(bool flag) {need_odom_ = flag;} // TODO: Consider not allowing this to be toggled after takeoff
    void set_use_attitude_safety_catch(bool flag) {use_attitude_safety_catch_ = flag;}

    // Home Methods
    bool setHome();                 // Uses the current position and yaw
    bool goHome();

    // Movement
    bool takeoff();

    bool goTo(double x, double y, double z, double yaw,
              double v_des = 0, double a_des = 0);
    bool goTo(Vec4 xyz_yaw, Vec2 v_and_a_des = Vec2::Zero());
    bool goTo(Vec3 xyz, double yaw, Vec2 v_and_a_des = Vec2::Zero());
    bool goTo(Vec3 xyz, Vec2 v_and_a_des = Vec2::Zero());  // Uses Current yaw

    bool setDesVelWorld(double x, double y, double z, double yaw);
    bool setDesVelWorld(Vec4 xyz_yaw);
    bool setDesVelWorld(Vec3 xyz);
    bool setDesVelWorld(Vec3 xyz, double yaw);
    bool setDesVelWorld(double x, double y, double z);

    bool setDesVelBody(Vec3 xyz, double yaw);
    bool setDesVelBody(Vec4 xyz_yaw);
    bool setDesVelBody(Vec3 xyz);
    bool setDesVelBody(double x, double y, double z);
    bool setDesVelBody(double x, double y, double z, double yaw);

    // Yaw control
    bool goToYaw(double);

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
    unsigned char* radio() {return radio_;}

    // Safety
    bool hover();
    bool ehover();
    bool motors(bool);
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
    short tracker_status_;
    bool transition(const std::string &tracker_str);

    ros::Time last_odom_t_, last_imu_t_, last_output_data_t_, last_heartbeat_t_;

    Vec3 pos_, vel_;
    double mass_;
    const double kGravity_;
    Quat odom_q_, imu_q_;
    double yaw_, yaw_dot_;
    double takeoff_height_;
    double max_attitude_angle_;

    Vec3 home_;
    Vec3 goal_;
    double goal_yaw_, home_yaw_;

    bool need_imu_, need_output_data_, need_odom_, use_attitude_safety_catch_;
    bool home_set_, serial_, motors_;
    float voltage_, pressure_height_, pressure_dheight_, magnetic_field_[3];
    unsigned char radio_[8];

    // Publishers
    ros::Publisher pub_goal_min_jerk_;
    ros::Publisher pub_goal_line_tracker_distance_;
    ros::Publisher pub_goal_velocity_;
    ros::Publisher pub_motors_;
    ros::Publisher pub_estop_;
    ros::Publisher pub_goal_yaw_;
    ros::Publisher pub_so3_command_;
    ros::Publisher pub_position_command_;
    ros::Publisher pub_pwm_command_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber output_data_sub_;
    ros::Subscriber heartbeat_sub_;
    ros::Subscriber tracker_status_sub_;

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
