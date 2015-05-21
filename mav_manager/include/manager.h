#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
#include <string>
#include <Eigen/Geometry>

// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

// quadrotor_control
#include <quadrotor_msgs/OutputData.h>

// Service includes
#include <mav_manager/Bool.h>
#include <mav_manager/Trigger.h>
#include <mav_manager/Vec4.h>

class MAVManager
{
  public:

    // Typedefs
    typedef Eigen::Vector3d    Vec3;
    typedef Eigen::Vector4d    Vec4;
    typedef Eigen::Quaterniond Quat;

    MAVManager();

    // Accessors
    Vec3 pos() { return pos_; }
    Vec3 home() { return home_; }
    double yaw() { return yaw_; }
    double home_yaw() { return home_yaw_; }
    double mass() { return mass_; }
   
    // Mutators
    void set_mass(double m)   {mass_ = m;}

    // Home Methods
    bool setHome();                 // Uses the current position and yaw
    bool goHome();

    // Movement
    bool takeoff();

    bool goTo(Vec4 xyz_yaw);
    bool goTo(Vec3 xyz);                                 // Uses current yaw
    bool goTo(Vec3 xyz, double yaw);
    bool goTo(double x, double y, double z);             // Uses current yaw
    bool goTo(double x, double y, double z, double yaw);

    bool setDesVelWorld(Vec4);                           // (xyz(yaw))
    bool setDesVelWorld(Vec3);                           // (xyz)
    bool setDesVelWorld(Vec3, double);                   // (xyz, yaw)
    bool setDesVelWorld(double, double, double);         // (x, y, z)
    bool setDesVelWorld(double, double, double, double); // (x, y, z, yaw)

    bool setDesVelBody(Vec4);                           // (xyz(yaw))
    bool setDesVelBody(Vec3);                           // (xyz)
    bool setDesVelBody(Vec3, double);                   // (xyz, yaw)
    bool setDesVelBody(double, double, double);         // (x, y, z)
    bool setDesVelBody(double, double, double, double); // (x, y, z, yaw)

    // Yaw control
    bool goToYaw(double);

    // Waypoints
    void clearWaypoints();
    void addWaypoint();

    // Use radio as velocity tracker
    // TODO: This will need to be monitored in the output_data callback if toggled
    bool useRadioForVelocity();
    // Monitoring
    bool have_odom();
    bool have_imu();
    bool have_output_data();

    // Safety
    bool hover();
    bool ehover();
    void motors(bool);
    bool eland();
    void estop();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void heartbeat_cb(const std_msgs::Empty::ConstPtr &msg);
    void heartbeat();

    ros::Time last_odom_t_, last_output_data_t_, last_imu_t_, last_heartbeat_t_;

    std::string active_tracker_;
    bool transition(const std::string &tracker_str);

    Vec3 pos_, vel_;
    Vec4 offsets_;
    double mass_;
    const double kGravity_;
    Quat odom_q_;
    double yaw_, yaw_dot_;

    geometry_msgs::Quaternion imu_q_;

    uint8_t radio_channel_[8];

    Vec3 home_;
    Vec3 goal_;
    double goal_yaw_, home_yaw_;

    bool home_set_, home_yaw_set_, serial_, motors_;

    // Publishers
    ros::Publisher pub_goal_min_jerk_;
    ros::Publisher pub_goal_line_tracker_distance_;
    ros::Publisher pub_goal_velocity_;
    ros::Publisher pub_motors_;
    ros::Publisher pub_estop_;
    ros::Publisher pub_goal_yaw_;
    ros::Publisher so3_command_pub_;
    ros::Publisher pub_pwm_command_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber output_data_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber heartbeat_sub_;

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
