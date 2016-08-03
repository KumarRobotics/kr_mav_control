// This nodelet is meant to subscribe to so3 commands and then convert them
// to geometry_msgs/Twist which is the type for cmd_vel, the crazyflie control
// topic. The format of this is as follows:
//  linear.y = roll     [-30 to 30 degrees]         (may be negative)
//  linear.x = pitch    [-30 to 30 degrees]         (may be negative)
//  linear.z = thrust   [0 to 60,000]               (motors stiction around 2000) 
//  angular.z = yawrate [-200 to 200 degrees/second] (note this is not yaw!)

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

// TODO: Remove CLAMP as macro
#define CLAMP(x,min,max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class SO3CmdToCrazyflie : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

  bool odom_set_, so3_cmd_set_;
  Eigen::Quaterniond q_odom_;
  double kf_, lin_cof_a_, lin_int_b_;

  ros::Publisher attitude_raw_pub_;
  ros::Publisher odom_pose_pub_; // For sending PoseStamped to firmware.
  ros::Publisher crazy_cmd_vel_pub_;

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  quadrotor_msgs::SO3Command last_so3_cmd_;

  double c1_;
  double c2_;
  double c3_;

  // TODO get rid of this for the gains coming in
  double kp_yaw_rate_;

  // double thrust_pwm_min_ = 10000; // Necessary to overcome stiction
  int thrust_pwm_max_; // Mapped to PWM

};

void SO3CmdToCrazyflie::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  q_odom_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  // Publish PoseStamped for mavros vision_pose plugin
  auto odom_pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();
  odom_pose_msg->header = odom->header;
  odom_pose_msg->pose = odom->pose.pose;
  odom_pose_pub_.publish(odom_pose_msg);
}

void SO3CmdToCrazyflie::so3_cmd_callback(
    const quadrotor_msgs::SO3Command::ConstPtr &msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(q_odom_);

  const float yaw_cur = std::atan2(R_cur(1,0), R_cur(0,0));
  const float yaw_des = std::atan2(R_des(1,0), R_des(0,0));
  float pitch_des = -std::asin(R_des(2,0));
  float roll_des = std::atan2(R_des(2,1), R_des(2,2));

  roll_des = roll_des * 180 / M_PI;
  pitch_des = pitch_des * 180 / M_PI;

  double thrust_f = f_des(0)*R_cur(0,2) + f_des(1)*R_cur(1,2) + f_des(2)*R_cur(2,2); // Force in Newtons
  //ROS_INFO("thrust_f is %2.5f newtons", thrust_f);
  thrust_f = std::max(thrust_f, 0.0);
  
  thrust_f = thrust_f*1000/9.8; // Force in grams
  //ROS_INFO("thrust_f is %2.5f grams", thrust_f);

  //ROS_INFO("coeffs are %2.2f, %2.2f, %2.2f", c1_, c2_, c3_);
  float thrust_pwm = c1_ + c2_*std::sqrt(c3_ + thrust_f);

  //ROS_INFO("thrust_pwm is %2.5f from 0-1", thrust_pwm);

  thrust_pwm = thrust_pwm * thrust_pwm_max_; // thrust_pwm mapped from 0-60000
  //ROS_INFO("thrust_pwm is calculated to be %2.5f in pwm", thrust_pwm);
  //

  geometry_msgs::Twist::Ptr crazy_vel_cmd(new geometry_msgs::Twist);

  if(!msg->aux.enable_motors) // If motors are not activated, set thrust to zero
  {
    thrust_pwm = 0;
    ROS_INFO("Motors disabled");
    crazy_cmd_vel_pub_.publish(crazy_vel_cmd);
    return;
  } 

  float e_yaw = yaw_des - yaw_cur;
  if(e_yaw > M_PI)
    e_yaw -= 2*M_PI;
  else if(e_yaw < -M_PI)
    e_yaw += 2*M_PI;

  float yaw_rate_des = msg->kR[2] * e_yaw;


  // If the crazyflie motors are timed out, we need to send a zero message in order to get them to start
  if((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_){
    crazy_cmd_vel_pub_.publish(crazy_vel_cmd);
    // Keep in mind that this publishes two messages in a row, this may cause bandwith problems
  }

  // TODO change this check to be a param
  // throttle the publish rate
  //if ((ros::Time::now() - last_so3_cmd_time_).toSec() > 1.0/30.0){
    crazy_vel_cmd->linear.y = roll_des  + msg->aux.angle_corrections[0];
    crazy_vel_cmd->linear.x = pitch_des + msg->aux.angle_corrections[1];
    crazy_vel_cmd->linear.z = CLAMP(thrust_pwm, 0, thrust_pwm_max_);

    // ROS_INFO("commanded thrust is %2.2f", CLAMP(thrust_pwm, 0, thrust_pwm_max_));

    crazy_vel_cmd->angular.z = yaw_rate_des;

    crazy_cmd_vel_pub_.publish(crazy_vel_cmd);
    // save last so3_cmd
    last_so3_cmd_ = *msg;
    last_so3_cmd_time_ = ros::Time::now();
  //}
  //else {
  //  ROS_INFO_STREAM("Commands too quick, time since is: " << (ros::Time::now() - last_so3_cmd_time_).toSec());
  //}
}

void SO3CmdToCrazyflie::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get thrust scaling parameters
  // Note that this is ignoring a constant based on the number of props, which
  // is captured with the lin_cof_a variable later.
  //
  // TODO this is where we load thrust scaling stuff
  if(priv_nh.getParam("kp_yaw_rate_", kp_yaw_rate_))
    ROS_INFO("kp yaw rate is %2.2f", kp_yaw_rate_);
  else
    ROS_FATAL("kp yaw rate not found");

  // get thrust scaling parameters
  if(priv_nh.getParam("c1_", c1_) &&
     priv_nh.getParam("c2_", c2_) && 
     priv_nh.getParam("c3_", c3_))
    ROS_INFO("Using %2.2f, %2.2f, %2.2f for thrust mapping", c1_, c2_, c3_);
  else
    ROS_FATAL("Must set coefficients for thrust scaling");

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.5);

  priv_nh.param("thrust_pwm_max_", thrust_pwm_max_, 60000);

  odom_set_ = false;
  so3_cmd_set_ = false;

  // TODO make sure this is publishing to the right place
  crazy_cmd_vel_pub_ =
      priv_nh.advertise<geometry_msgs::Twist>("cmd_vel_fast", 10);

  odom_pose_pub_ =
      priv_nh.advertise<geometry_msgs::PoseStamped>("odom_pose", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToCrazyflie::so3_cmd_callback, this,
                        ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToCrazyflie::odom_callback,
                                this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToCrazyflie, nodelet::Nodelet);
