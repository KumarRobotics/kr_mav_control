#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <topic_tools/shape_shifter.h>

static std::string g_new_frame_id;
static ros::Publisher g_pub_pose;

static void publishPoseStamped(const std_msgs::Header &header,
                               const geometry_msgs::Pose &pose)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = header;
  if(g_new_frame_id != "")
    pose_msg.header.frame_id = g_new_frame_id;
  pose_msg.pose = pose;
  g_pub_pose.publish(pose_msg);
}

static void any_callback(const topic_tools::ShapeShifter::ConstPtr &msg)
{
  // ROS_INFO_STREAM("Type: " << msg->getDataType());

  if(msg->getDataType() == "geometry_msgs/PoseStamped")
  {
    auto pose_msg = msg->instantiate<geometry_msgs::PoseStamped>();
    publishPoseStamped(pose_msg->header, pose_msg->pose);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseWithCovarianceStamped")
  {
    auto pose_msg =
        msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    publishPoseStamped(pose_msg->header, pose_msg->pose.pose);
  }
  else if(msg->getDataType() == "nav_msgs/Odometry")
  {
    auto odom_msg = msg->instantiate<nav_msgs::Odometry>();
    publishPoseStamped(odom_msg->header, odom_msg->pose.pose);
  }
  else if(msg->getDataType() == "quadrotor_msgs/PositionCommand")
  {
    auto cmd_msg = msg->instantiate<quadrotor_msgs::PositionCommand>();
    geometry_msgs::Pose cmd_pose;
    cmd_pose.position = cmd_msg->position;
    cmd_pose.orientation.x = 0;
    cmd_pose.orientation.y = 0;
    cmd_pose.orientation.z = std::sin(cmd_msg->yaw / 2);
    cmd_pose.orientation.w = std::cos(cmd_msg->yaw / 2);
    publishPoseStamped(cmd_msg->header, cmd_pose);
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
