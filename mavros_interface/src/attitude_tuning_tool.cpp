#include <mavros_interface/attitude_tuning_tool.h>

#include <Eigen/Geometry>

// ROS Related
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <mavros_msgs/AttitudeTarget.h>

AttitudeTuningTool::AttitudeTuningTool()
    : priv_nh_("~"),
      sp_q_(1.0, 0.0 ,0.0 ,0.0)
{
  // Publishers
  pub_sp_as_pose_msg_ = priv_nh_.advertise<geometry_msgs::PoseStamped>("setpoint_raw_attitude_as_pose", 10);
  pub_thrust_vector_error_ = priv_nh_.advertise<std_msgs::Float32>("b3_geodesic_error", 10);

  // Subscribers
  pose_sub_ = priv_nh_.subscribe("pose", 10, &AttitudeTuningTool::pose_cb, this, ros::TransportHints().tcpNoDelay());
  setpoint_sub_ = priv_nh_.subscribe("setpoint_raw/attitude", 10, &AttitudeTuningTool::setpoint_cb, this, ros::TransportHints().tcpNoDelay());
}

void AttitudeTuningTool::setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  sp_q_ = Eigen::Quaternionf(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void AttitudeTuningTool::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped sp_pose_msg;
  sp_pose_msg.header.frame_id = msg->header.frame_id;
  sp_pose_msg.pose.position.x = msg->pose.position.x;
  sp_pose_msg.pose.position.y = msg->pose.position.y;
  sp_pose_msg.pose.position.z = msg->pose.position.z;
  sp_pose_msg.pose.orientation.w = sp_q_.w();
  sp_pose_msg.pose.orientation.x = sp_q_.x();
  sp_pose_msg.pose.orientation.y = sp_q_.y();
  sp_pose_msg.pose.orientation.z = sp_q_.z();
  pub_sp_as_pose_msg_.publish(sp_pose_msg);

  Eigen::Quaternionf odom_q = Eigen::Quaternionf(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);

  Eigen::Vector3f b3 = odom_q.toRotationMatrix() * Eigen::Vector3f::UnitZ();
  Eigen::Vector3f b3_des = sp_q_.toRotationMatrix() * Eigen::Vector3f::UnitZ();

  std_msgs::Float32 error_msg;
  error_msg.data = 180 / M_PI * std::acos(b3.dot(b3_des));
  pub_thrust_vector_error_.publish(error_msg);

  // TODO: Consider yaw?
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_tuning_tool");

  AttitudeTuningTool att;

  ros::spin();
  return 0;
}
