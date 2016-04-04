#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <topic_tools/shape_shifter.h>
#include <tf/transform_datatypes.h>

static std::string g_new_frame_id;
static ros::Publisher g_pub_pose;

static void publishPoseStamped(const std::string &frame_id,
                               const geometry_msgs::Pose &pose)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = (g_new_frame_id == "") ? frame_id : g_new_frame_id;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose = pose;
  g_pub_pose.publish(pose_msg);
}

static void any_callback(const topic_tools::ShapeShifter::ConstPtr &msg)
{
  // ROS_INFO_STREAM("Type: " << msg->getDataType());

  if(msg->getDataType() == "geometry_msgs/PoseStamped")
  {
    auto pose_msg = msg->instantiate<geometry_msgs::PoseStamped>();
    publishPoseStamped(pose_msg->header.frame_id, pose_msg->pose);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseWithCovarianceStamped")
  {
    auto pose_msg =
        msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    publishPoseStamped(pose_msg->header.frame_id, pose_msg->pose.pose);
  }
  else if(msg->getDataType() == "nav_msgs/Odometry")
  {
    auto odom_msg = msg->instantiate<nav_msgs::Odometry>();
    publishPoseStamped(odom_msg->header.frame_id, odom_msg->pose.pose);
  }
  else if(msg->getDataType() == "quadrotor_msgs/PositionCommand")
  {
    auto cmd_msg = msg->instantiate<quadrotor_msgs::PositionCommand>();
    geometry_msgs::Pose cmd_pose;
    cmd_pose.position = cmd_msg->position;
    cmd_pose.orientation = tf::createQuaternionMsgFromYaw(cmd_msg->yaw);
    publishPoseStamped(cmd_msg->header.frame_id, cmd_pose);
  }
  else
  {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << " got unsupported message type " << msg->getDataType());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_visualization");

  ros::NodeHandle nh("~");

  nh.param("new_frame_id", g_new_frame_id, std::string(""));

  ros::Subscriber any_sub = nh.subscribe("input", 10, &any_callback,
                                         ros::TransportHints().tcpNoDelay());
  g_pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose_viz", 10);

  ros::spin();

  return 0;
}
