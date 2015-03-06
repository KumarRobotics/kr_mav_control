#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

static std::string mesh_resource;
static ros::Publisher pub_vis;
static double color_r, color_g, color_b, color_a;
static double scale_x, scale_y, scale_z;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = msg->header.frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be displayed regardless of the current time
  marker.ns = "mesh_visualization";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = msg->pose.pose.position.x;
  marker.pose.position.y = msg->pose.pose.position.y;
  marker.pose.position.z = msg->pose.pose.position.z;
  marker.pose.orientation.x = msg->pose.pose.orientation.x;
  marker.pose.orientation.y = msg->pose.pose.orientation.y;
  marker.pose.orientation.z = msg->pose.pose.orientation.z;
  marker.pose.orientation.w = msg->pose.pose.orientation.w;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mesh_visualization");

  ros::NodeHandle nh("~");

  nh.param("mesh_resource", mesh_resource, std::string("package://mesh_visualization/mesh/hummingbird.mesh"));
  nh.param("color/r", color_r, 1.0);
  nh.param("color/g", color_g, 0.0);
  nh.param("color/b", color_b, 0.0);
  nh.param("color/a", color_a, 1.0);
  nh.param("scale/x", scale_x, 1.0);
  nh.param("scale/y", scale_y, 1.0);
  nh.param("scale/z", scale_z, 1.0);

  ros::Subscriber odom_sub = nh.subscribe("odom", 10, &odom_callback, ros::TransportHints().tcpNoDelay());
  pub_vis = nh.advertise<visualization_msgs::Marker>("robot", 10);

  ros::spin();

  return 0;
}
