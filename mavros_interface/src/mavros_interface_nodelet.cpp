#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>

class MavrosInterface : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;

  ros::Publisher attitude_raw_pub_;
  ros::Publisher odom_pose_pub_; // For sending PoseStamped to firmware.
  ros::Publisher odom_pub_;      // For conversion to our convention

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  quadrotor_msgs::SO3Command last_so3_cmd_;
  tf::TransformBroadcaster tf_broadcaster_;
};

void MavrosInterface::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);

  // Publish PoseStamped for mavros vision_pose plugin
  auto odom_pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();
  odom_pose_msg->header = odom->header;
  odom_pose_msg->pose = odom->pose.pose;
  odom_pose_pub_.publish(odom_pose_msg);


  // tf broadcaster
  geometry_msgs::TransformStamped ts;

  ts.header.stamp = odom->header.stamp;
  ts.header.frame_id = odom->header.frame_id;
  ts.child_frame_id = odom->child_frame_id;

  ts.transform.translation.x = odom->pose.pose.position.x;
  ts.transform.translation.y = odom->pose.pose.position.y;
  ts.transform.translation.z = odom->pose.pose.position.z;

  ts.transform.rotation = odom->pose.pose.orientation;

  tf_broadcaster_.sendTransform(ts);


  // Publish Odometry but with velocity in the world frame
  auto new_odom = boost::make_shared<nav_msgs::Odometry>(*odom);

  Eigen::Vector3d body_frame_linear_vel, world_frame_linear_vel;
  tf::vectorMsgToEigen(odom->twist.twist.linear, body_frame_linear_vel);
  world_frame_linear_vel = odom_q_.inverse() * body_frame_linear_vel;
  tf::vectorEigenToMsg(world_frame_linear_vel, new_odom->twist.twist.linear);

  odom_pub_.publish(new_odom);
};

void MavrosInterface::imu_callback(const sensor_msgs::Imu::ConstPtr &pose)
{
  if(!imu_set_)
    imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x,
                              pose->orientation.y, pose->orientation.z);

  if(so3_cmd_set_ &&
     ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_INFO("so3_cmd timeout. %f seconds since last command",
             (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr =
        boost::make_shared<quadrotor_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void MavrosInterface::so3_cmd_callback(
    const quadrotor_msgs::SO3Command::ConstPtr &msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);

  // convert to tf::Quaternion
  tf::Quaternion imu_tf =
      tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf::Quaternion odom_tf =
      tf::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  // extract RPY's
  double imu_roll, imu_pitch, imu_yaw;
  double odom_roll, odom_pitch, odom_yaw;
  tf::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  tf::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);

  // create only yaw tf:Quaternions
  tf::Quaternion imu_tf_yaw;
  tf::Quaternion odom_tf_yaw;
  imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  const tf::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // transform!
  Eigen::Quaterniond q_des_transformed =
      Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(),
                         tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) *
      q_des;

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const float Psi =
      0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) +
                      R_des(2, 0) * R_cur(2, 0) + R_des(0, 1) * R_cur(0, 1) +
                      R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                      R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) +
                      R_des(2, 2) * R_cur(2, 2)));

  if(Psi >= 1.0f) // Position control stability guaranteed only when Psi < 1
    ROS_WARN_THROTTLE(1, "psi = %2.2f >= 1.0, attitude controller may not be stable.", Psi);

  double throttle = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  // Scale force to be proportional to rotor velocity (rad/s).
  // Note: This ignores the number of propellers.
  throttle = std::sqrt(throttle / kf_);

  // Scaling from proportional rotor velocity (rad/s) to att_throttle for pixhawk
  throttle = lin_cof_a_ * throttle + lin_int_b_;

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if(!msg->aux.enable_motors)
    throttle = 0;

  // publish messages
  auto setpoint_msg = boost::make_shared<mavros_msgs::AttitudeTarget>();
  setpoint_msg->header = msg->header;
  setpoint_msg->type_mask = 0;
  setpoint_msg->orientation.w = q_des_transformed.w();
  setpoint_msg->orientation.x = q_des_transformed.x();
  setpoint_msg->orientation.y = q_des_transformed.y();
  setpoint_msg->orientation.z = q_des_transformed.z();
  setpoint_msg->body_rate.x = msg->angular_velocity.x;
  setpoint_msg->body_rate.y = msg->angular_velocity.y;
  setpoint_msg->body_rate.z = msg->angular_velocity.z;
  setpoint_msg->thrust = throttle;

  attitude_raw_pub_.publish(setpoint_msg);

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = ros::Time::now();
}

void MavrosInterface::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get thrust scaling parameters
  // Note that this is ignoring a constant based on the number of props, which
  // is captured with the lin_cof_a variable later.
  if(priv_nh.getParam("kf", kf_))
    ROS_INFO("Using kf=%F so that prop speed = sqrt(f / kf) to scale force to speed.", kf_);
  else
    ROS_ERROR("Must set kf param for thrust scaling. Motor speed = sqrt(thrust / kf)");

  ROS_ASSERT_MSG(kf_ > 0, "kf must be positive. kf = %d", kf_);

  // get thrust scaling parameters
  if(priv_nh.getParam("lin_cof_a", lin_cof_a_) &&
     priv_nh.getParam("lin_int_b", lin_int_b_))
    ROS_INFO("Using %F*x + %F to scale prop speed to att_throttle.", lin_cof_a_,
             lin_int_b_);
  else
    ROS_ERROR("Must set coefficients for thrust scaling (scaling from rotor "
              "velocity (rad/s) to att_throttle for pixhawk)");

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.25);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;

  attitude_raw_pub_ =
    priv_nh.advertise<mavros_msgs::AttitudeTarget>("attitude_raw", 10);

  odom_pose_pub_ =
    priv_nh.advertise<geometry_msgs::PoseStamped>("odom_pose", 10);

  odom_pub_ =
    priv_nh.advertise<nav_msgs::Odometry>("odom", 10);

  so3_cmd_sub_ =
    priv_nh.subscribe("so3_cmd", 1, &MavrosInterface::so3_cmd_callback, this,
    ros::TransportHints().tcpNoDelay());

  odom_sub_ =
    priv_nh.subscribe("mavros/local_position/odom", 1, &MavrosInterface::odom_callback, this,
    ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 1, &MavrosInterface::imu_callback, this,
    ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MavrosInterface, nodelet::Nodelet);
