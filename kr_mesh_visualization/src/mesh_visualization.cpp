#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MeshVisualizationNode : public rclcpp::Node
{
 public:
  MeshVisualizationNode() : Node("kr_mesh_visualization")
  {
    this->declare_parameter<std::string>("mesh_resource",
                                         std::string("package://kr_mesh_visualization/mesh/hummingbird.mesh"));
    this->declare_parameter<double>("color/r", 1.0);
    this->declare_parameter<double>("color/g", 0.0);
    this->declare_parameter<double>("color/b", 0.0);
    this->declare_parameter<double>("color/a", 1.0);
    this->declare_parameter<double>("scale/x", 1.0);
    this->declare_parameter<double>("scale/y", 1.0);
    this->declare_parameter<double>("scale/z", 1.0);
    this->declare_parameter<std::string>("new_frame_id", std::string(""));
    this->declare_parameter<std::string>("msg_type", std::string("nav_msgs/msg/Odometry"));

    this->get_parameter("mesh_resource", mesh_resource);
    this->get_parameter("color/r", color_r);
    this->get_parameter("color/g", color_g);
    this->get_parameter("color/b", color_b);
    this->get_parameter("color/a", color_a);
    this->get_parameter("scale/x", scale_x);
    this->get_parameter("scale/y", scale_y);
    this->get_parameter("scale/z", scale_z);
    this->get_parameter("new_frame_id", new_frame_id);
    this->get_parameter("msg_type", msg_type);

    using namespace std::placeholders;
    pub_vis_ = this->create_publisher<visualization_msgs::msg::Marker>("~/robot", 10);
    
    if(msg_type == "nav_msgs/msg/Odometry")
    {
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("~/input", 10,
                  std::bind(&MeshVisualizationNode::odom_callback, this, _1));
    }
    else if(msg_type == "geometry_msgs/msg/PoseStamped")
    {
      sub_ps_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "~/input", 10, std::bind(&MeshVisualizationNode::callback_pose_stamped, this, _1));
    }
    else if(msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped")
    {
      sub_pwcs_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "~/input", 10, std::bind(&MeshVisualizationNode::callback_pose_with_covariance_stamped, this, _1));
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), std::string(this->get_name()) << " got unsupported message type " << msg_type << "\n");
    }

  }

 protected:
  std::string mesh_resource, new_frame_id;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vis_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ps_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pwcs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  double color_r, color_g, color_b, color_a;
  double scale_x, scale_y, scale_z;
  std::string msg_type;

  void publishMarker(const std::string &frame_id, const geometry_msgs::msg::Pose &pose)
  {
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = (new_frame_id == "") ? frame_id : new_frame_id;
    marker.header.stamp = rclcpp::Time(0, 0);
    marker.ns = this->get_name();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = color_a;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.mesh_resource = mesh_resource;
    pub_vis_->publish(marker);
  }

  void callback_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    publishMarker(msg->header.frame_id, msg->pose);
  }

  void callback_pose_with_covariance_stamped(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    publishMarker(msg->header.frame_id, msg->pose.pose);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    publishMarker(msg->header.frame_id, msg->pose.pose);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MeshVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
