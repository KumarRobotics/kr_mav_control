#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <visualization_msgs/Marker.h>

static std::string mesh_resource, new_frame_id;
static ros::Publisher pub_vis;
static double color_r, color_g, color_b, color_a;
static double scale_x, scale_y, scale_z;

static void publishMarker(const std::string &frame_id, const geometry_msgs::Pose &pose)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = (new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time();  // time 0 so that the marker will be
                                      // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;
  marker.color.a = color_a;
  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.mesh_resource = mesh_resource;
  pub_vis.publish(marker);
}

static void any_callback(const topic_tools::ShapeShifter::ConstPtr &msg)
{
  // ROS_INFO_STREAM("Type: " << msg->getDataType());

  if(msg->getDataType() == "geometry_msgs/PoseStamped")
  {
    auto pose_msg = msg->instantiate<geometry_msgs::PoseStamped>();
    publishMarker(pose_msg->header.frame_id, pose_msg->pose);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseWithCovarianceStamped")
  {
    auto pose_msg = msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    publishMarker(pose_msg->header.frame_id, pose_msg->pose.pose);
  }
  else if(msg->getDataType() == "nav_msgs/Odometry")
  {
    auto odom_msg = msg->instantiate<nav_msgs::Odometry>();
    publishMarker(odom_msg->header.frame_id, odom_msg->pose.pose);
  }
  else
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " got unsupported message type " << msg->getDataType());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kr_mesh_visualization");

  ros::NodeHandle nh("~");

  nh.param("mesh_resource", mesh_resource, std::string("package://kr_mesh_visualization/mesh/hummingbird.mesh"));
  nh.param("color/r", color_r, 1.0);
  nh.param("color/g", color_g, 0.0);
  nh.param("color/b", color_b, 0.0);
  nh.param("color/a", color_a, 1.0);
  nh.param("scale/x", scale_x, 1.0);
  nh.param("scale/y", scale_y, 1.0);
  nh.param("scale/z", scale_z, 1.0);

  nh.param("new_frame_id", new_frame_id, std::string(""));

  pub_vis = nh.advertise<visualization_msgs::Marker>("robot", 10);

  ros::Subscriber any_sub = nh.subscribe("input", 10, &any_callback, ros::TransportHints().tcpNoDelay());
  ros::spin();

  return 0;
}
