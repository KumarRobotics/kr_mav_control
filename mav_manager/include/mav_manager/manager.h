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

    // Mutators
    void set_mass(double m)   {mass_ = m;}

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

    bool setPositionCommand(const quadrotor_msgs::PositionCommand cmd);
    bool useNullTracker();

    // Monitoring
    bool have_recent_odom();
    bool have_recent_imu();

    // Safety
    bool hover();
    bool ehover();
    bool motors(bool);
    bool eland();
    bool estop();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void heartbeat_cb(const std_msgs::Empty::ConstPtr &msg);
    void heartbeat();

    std::string active_tracker_;
    bool transition(const std::string &tracker_str);

    ros::Time last_odom_t_, last_imu_t_, last_heartbeat_t_;

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

    bool home_set_, serial_, motors_;

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
    ros::Subscriber heartbeat_sub_;

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
