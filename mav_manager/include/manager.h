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
#include <mav_manager/Vec4.h>
#include <mav_manager/Empty.h>

class MAVManager 
{
  public:

    enum enum_controllers
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

    void goTo(vec4 xyz_yaw);
    void goTo(vec3 xyz);                                 // Uses current yaw
    void goTo(vec3 xyz, double yaw);
    void goTo(double x, double y, double z);             // Uses current yaw
    void goTo(double x, double y, double z, double yaw);

    void setDesVelWorld(vec4);                           // (xyz(yaw))
    void setDesVelWorld(vec3);                           // (xyz)
    void setDesVelWorld(vec3, double);                   // (xyz, yaw)
    void setDesVelWorld(double, double, double);         // (x, y, z)
    void setDesVelWorld(double, double, double, double); // (x, y, z, yaw)

    void setDesVelBody(vec4);                           // (xyz(yaw))
    void setDesVelBody(vec3);                           // (xyz)
    void setDesVelBody(vec3, double);                   // (xyz, yaw)
    void setDesVelBody(double, double, double);         // (x, y, z)
    void setDesVelBody(double, double, double, double); // (x, y, z, yaw)

    // Yaw control
    void goToYaw(double);

    // Waypoints
    void clearWaypoints();
    void addWaypoint();

    // Use radio as velocity controller
    // TODO: This will need to be monitored in the output_data callback if toggled 
    void useRadioForVelocity();

    // Safety
    void hover();
    void ehover();
    void motors(bool);
    void estop();

  private:
    
    ros::NodeHandle nh_;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg);

    enum_controllers active_controller_;
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
    ros::Publisher pub_goal_distance_;
    ros::Publisher pub_goal_velocity_;
    ros::Publisher pub_motors_;
    ros::Publisher pub_estop_;
    ros::Publisher pub_goal_yaw_;
    ros::Publisher so3_command_pub_;
    ros::Publisher pub_pwm_command_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber output_data_sub_;

    // Services
    ros::ServiceClient srv_transition_;
};

#endif /* MANAGER_H */
