#ifndef MANAGER_H
#define MANAGER_H

// Standard C++
#include <math.h>
#include <iostream>
#include <string>

// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

// TF
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
// #include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

// quadrotor_control
#include <controllers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PWMCommand.h>
#include <quadrotor_msgs/OutputData.h>

typedef Eigen::Vector3d    vec3;
typedef Eigen::Vector4d    vec4;
typedef Eigen::Quaterniond quat;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker_min_jerk("line_tracker/LineTrackerMinJerk");
// static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");
static const std::string null_tracker_str("null_tracker/NullTracker");

enum enum_controllers
{
  INIT,
  LINE_TRACKER_MIN_JERK,
  LINE_TRACKER_DISTANCE,
  VELOCITY_TRACKER,
  NULL_TRACKER,
  LAND,
  NONE,
};

class MAVManager {

  private:
    ros::NodeHandle nh_;

    enum_controllers active_controller_;

    vec3 pos_;
    vec4 offsets_;
    quat odom_q_;
    double yaw_;

    // geometry_msgs::Point pos_;
    geometry_msgs::Vector3 vel_;
    geometry_msgs::Quaternion imu_q_;
    
    // Odometry stuff
    void updateOdom(const nav_msgs::Odometry::ConstPtr &msg);

    // Output Data
    void updateOutputData(const quadrotor_msgs::OutputData::ConstPtr &msg);
    uint8_t radio_channel_[8];

    vec3 home_, goal_;
    double goal_yaw_, home_yaw_;

    bool have_odom_, home_set_, home_yaw_set_, serial_, motors_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber output_data_sub_;

    // Services
    ros::ServiceClient srv_transition_;

    // Publishers
    ros::Publisher pub_goal_min_jerk_;
    ros::Publisher pub_goal_distance_;
    ros::Publisher pub_goal_velocity_;
    ros::Publisher pub_motors_;
    ros::Publisher pub_estop_;
    ros::Publisher pub_goal_yaw_;
    // ros::Publisher pub_info_bool_;
    ros::Publisher so3_command_pub_;
    // ros::Publisher pub_position_cmd_;
    ros::Publisher pub_pwm_command_;

  public:

    MAVManager();

    void takeoff();

    // Home Methods
    bool setHome();
    bool setHome(vec3);
    bool setHome(vec3, double);
    vec3 getHomePosition() {return home_;}
    double getHomeYaw() {return home_yaw_;}
    void goHome();

    // Movement
    void goTo(vec4);                                  // (xyz(yaw))
    void goTo(vec3);                                  // (xyz)
    void goTo(vec3, double);                          // (xyz, yaw)
    void goTo(double, double, double);                // (x, y, z)
    void goTo(double, double, double, double);        // (x, y, z, yaw)

    void worldVelocity(vec4);                           // (xyz(yaw))
    void worldVelocity(vec3);                           // (xyz)
    void worldVelocity(vec3, double);                   // (xyz, yaw)
    void worldVelocity(double, double, double);         // (x, y, z)
    void worldVelocity(double, double, double, double); // (x, y, z, yaw)

    void bodyVelocity(vec4);                           // (xyz(yaw))
    void bodyVelocity(vec3);                           // (xyz)
    void bodyVelocity(vec3, double);                   // (xyz, yaw)
    void bodyVelocity(double, double, double);         // (x, y, z)
    void bodyVelocity(double, double, double, double); // (x, y, z, yaw)

    // Yaw control
    void setYaw(double), setYawVelocity(double);

    // Waypoints
    void clearWaypoints();
    void addWaypoint();

    // Use radio as velocity controller
    void useRadioForVelocity();

    // Safety
    void hover();
    void ehover();
    void motors(bool);
    void estop();
};


MAVManager::MAVManager(): nh_("~"), have_odom_(false), active_controller_(INIT), offsets_(0,0,0,0)
{
  // Subscribers
  odom_sub_ = nh_.subscribe("odom", 1, &MAVManager::updateOdom, this);
  output_data_sub_ = nh_.subscribe("output_data", 1, &MAVManager::updateOutputData, this);

  // Services
  srv_transition_ = nh_.serviceClient<controllers_manager::Transition>("controllers_manager/transition");

  // Publishers
  pub_goal_min_jerk_ = nh_.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_min_jerk/goal", 1);
  pub_goal_velocity_ = nh_.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/velocity_tracker/vel_cmd_with_yaw", 1);
  pub_motors_ = nh_.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = nh_.advertise<std_msgs::Empty>("estop", 1);
  so3_command_pub_ = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);
  
  // Disable motors
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);
  
  ROS_DEBUG("Starting NullTracker");
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);
  active_controller_ = NULL_TRACKER;

  // Publish so3_command to stop motors
  quadrotor_msgs::SO3Command so3_cmd;
  so3_cmd.aux.enable_motors = false;
  so3_command_pub_.publish(so3_cmd);
}


void MAVManager::updateOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  odom_q_ = quat(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

  yaw_ = tf::getYaw(msg->pose.pose.orientation);
}

void MAVManager::takeoff()
{
  if (have_odom_)
  {
    if (this->setHome())
    {
      ROS_INFO("Initiating launch sequence...");

      geometry_msgs::Point goal;
      goal.x = pos_(0);
      goal.y = pos_(1);
      goal.z = pos_(2) + 0.2;
      pub_goal_distance_.publish(goal);
     
      // Only takeoff if currently under NULL_TRACKER
      if (active_controller_ == NULL_TRACKER)
      {
        usleep(100000);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = line_tracker_distance;
        srv_transition_.call(transition_cmd);
        active_controller_ = LINE_TRACKER_DISTANCE;
      }
      else
        ROS_WARN("active_controller_ must be INIT before taking off");
    }
}
  else
    ROS_WARN("Cannot takeoff without odometry.");
}

bool MAVManager::setHome()
{
  if (have_odom_)
  {
    home_ = pos_;
    home_set_ = true;

    home_yaw_ = yaw_;
    home_yaw_set_ = true;
  }
  else
    ROS_WARN("Cannot set home unless current pose is set or an argument is provided.");
}

bool MAVManager::setHome(vec3 home)
{
  home_ = home;
  home_set_ = true;
  home_yaw_set_ = false;
}

bool MAVManager::setHome(vec3 home, double yaw)
{
  home_ = home;
  home_set_ = true;
  home_yaw_ = yaw;
}

void MAVManager::goTo(vec4 target) // (xyz(psi))
{
  ROS_INFO("Engaging controller: LINE_TRACKER_MIN_JERK");
  quadrotor_msgs::FlatOutputs goal;
  goal.x   = target(0) + offsets_(0);
  goal.y   = target(1) + offsets_(1);
  goal.z   = target(2) + offsets_(2);
  goal.yaw = target(3) + offsets_(3);
  pub_goal_min_jerk_.publish(goal);

  // Only 
  if (active_controller_ != LINE_TRACKER_MIN_JERK)
  {
    usleep(100000);
    controllers_manager::Transition transition_cmd;
    transition_cmd.request.controller = line_tracker_min_jerk;
    srv_transition_.call(transition_cmd);
    active_controller_ = LINE_TRACKER_MIN_JERK;
  }
}
void MAVManager::goTo(vec3 xyz)
{
  vec4 goal(xyz(0), xyz(1), xyz(2), yaw_);
  this->goTo(goal);
}
void MAVManager::goTo(vec3 xyz, double yaw)
{
  vec4 goal(xyz(0), xyz(1), xyz(2), yaw);
  this->goTo(goal);
}
void MAVManager::goTo(double x, double y, double z)
{
  vec3 goal(x,y,z);
  this->goTo(goal);
}
void MAVManager::goTo(double x, double y, double z, double yaw)
{
  vec4 goal(x,y,z,yaw);
  this->goTo(goal);
}

// World Velocity commands
void MAVManager::worldVelocity(vec4 vel)
{
  quadrotor_msgs::FlatOutputs goal;
  goal.x = vel(0);
  goal.y = vel(1);
  goal.z = vel(2);
  goal.yaw = vel(3);
  pub_goal_velocity_.publish(goal);
  ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw);

  if (active_controller_ != VELOCITY_TRACKER)
  {
    usleep(100000);
    ROS_INFO("Engaging controller: VELOCITY_TRACKER");
    controllers_manager::Transition transition_cmd;
    transition_cmd.request.controller = velocity_tracker_str;
    srv_transition_.call(transition_cmd);
    active_controller_ = VELOCITY_TRACKER;
  }
}
void MAVManager::worldVelocity(vec3 xyz)
{
  vec4 goal(xyz(0), xyz(1), xyz(2), 0);
  this->worldVelocity(goal);
}
void MAVManager::worldVelocity(vec3 xyz, double yaw)
{
  vec4 goal(xyz(0), xyz(1), xyz(2), yaw);
  this->worldVelocity(goal);
}
void MAVManager::worldVelocity(double x, double y, double z)
{
  vec4 goal(x,y,z,0);
  this->worldVelocity(goal);
}
void MAVManager::worldVelocity(double x, double y, double z, double yaw)
{
  vec4 goal(x,y,z,yaw);
  this->worldVelocity(goal);
}

// Body Velocity commands
void MAVManager::bodyVelocity(vec3 xyz, double yaw)
{
  vec3 vel(odom_q_ * xyz);
  this->worldVelocity(vel, yaw);
}
void MAVManager::bodyVelocity(vec4 vel)
{
  vec3 v(vel(0), vel(1), vel(2));
  this->bodyVelocity(v, vel(3));
}
void MAVManager::bodyVelocity(vec3 xyz)
{
  this->bodyVelocity(xyz, 0);
}
void MAVManager::bodyVelocity(double x, double y, double z)
{
  vec3 vel(x,y,z);
  this->bodyVelocity(vel, 0);
}
void MAVManager::bodyVelocity(double x, double y, double z, double yaw)
{
  vec3 vel(x,y,z);
  this->bodyVelocity(vel, yaw);
}


void MAVManager::motors(bool flag)
{
  // Enable/Disable motors
  if (flag)
    ROS_WARN("Enabling Motors...");
  else
    ROS_WARN("Disabling Motors...");

  std_msgs::Bool motors_cmd;
  motors_cmd.data = flag;
  pub_motors_.publish(motors_cmd);

  motors_ = flag;
}

void MAVManager::updateOutputData(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  for (unsigned int i = 0; i < 8; i++)
    radio_channel_[i] = msg->radio_channel[i];

	ROS_DEBUG("First 4 radio channels: {%u, %u, %u, %u}",
	    radio_channel_[0], radio_channel_[1], radio_channel_[2], radio_channel_[3]);

  serial_ = radio_channel_[4] > 0;
}

void MAVManager::useRadioForVelocity()
{
  // constants
  double scale    = 255.0 / 2.0;
  double rc_max_v = 1.0;
  double rc_max_w = 15.0*M_PI/180.0;

  // scale radio
  vec4 vel;
  vel(0) = -((double)radio_channel_[0] - scale) / scale * rc_max_v;
  vel(1) = -((double)radio_channel_[1] - scale) / scale * rc_max_v;

  // Only consider z velocity if a switch is toggled
  if(radio_channel_[5] > scale)
    vel(2) = ((double)radio_channel_[2] - scale) / scale * rc_max_v;
  else
    vel(2) = 0;

  vel(3) = -((double)radio_channel_[3] - scale) / scale * rc_max_w;

  this->worldVelocity(vel);
}

void MAVManager::estop()
{
  // Publish the E-Stop command
  ROS_WARN("E-STOP");
  std_msgs::Empty estop_cmd;
  pub_estop_.publish(estop_cmd);

  // Disable motors
  ROS_WARN("Disarming motors...");
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);
}

#endif /* MANAGER_H */
