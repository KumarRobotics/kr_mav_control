#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
// #include <iostream>
#include <string>
#include <Eigen/Geometry>

// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// quadrotor_control
#include <quadrotor_msgs/OutputData.h>

// Service includes
#include <mav_manager/Bool.h>
#include <mav_manager/Empty.h>
#include <mav_manager/Vec4.h>

class MAVManager
{
  public:

    enum enum_trackers
    {
      INIT,
      LINE_TRACKER_DISTANCE,
      LINE_TRACKER_MIN_JERK,
      VELOCITY_TRACKER,
      NULL_TRACKER
    };

    // enum states
    // {
    //   MISC,
    //   RADIO_VELOCITY_CONTROL
    // };

    // Typedefs
    typedef Eigen::Vector3d    vec3;
    typedef Eigen::Vector4d    vec4;
    typedef Eigen::Quaterniond quat;

    MAVManager();

    vec3 getPosition() {return pos_;}
    vec3 getHome()     {return home_;}

    double getYaw()      {return yaw_;}
    double getHomeYaw()  {return home_yaw_;}

    // Home Methods
    bool setHome();                 // Uses the current position and yaw
    bool goHome();

    // Movement
    bool takeoff();

    bool goTo(vec4 xyz_yaw);
    bool goTo(vec3 xyz);                                 // Uses current yaw
    bool goTo(vec3 xyz, double yaw);
    bool goTo(double x, double y, double z);             // Uses current yaw
    bool goTo(double x, double y, double z, double yaw);

    bool setDesVelWorld(vec4);                           // (xyz(yaw))
    bool setDesVelWorld(vec3);                           // (xyz)
    bool setDesVelWorld(vec3, double);                   // (xyz, yaw)
    bool setDesVelWorld(double, double, double);         // (x, y, z)
    bool setDesVelWorld(double, double, double, double); // (x, y, z, yaw)

    bool setDesVelBody(vec4);                           // (xyz(yaw))
    bool setDesVelBody(vec3);                           // (xyz)
    bool setDesVelBody(vec3, double);                   // (xyz, yaw)
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

    // Safety
    bool hover();
    bool ehover();
    void motors(bool);
    void estop();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);

    enum_trackers active_tracker_;
    vec3 pos_, vel_;
    vec4 offsets_;
    quat odom_q_;
    double yaw_, yaw_dot_;

    geometry_msgs::Quaternion imu_q_;

    uint8_t radio_channel_[8];

    vec3 home_;
    vec3 goal_;
    double goal_yaw_, home_yaw_;

    bool have_odom_, home_set_, home_yaw_set_, serial_, motors_;

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

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
