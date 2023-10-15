#ifndef COMPOSITION__TRACKERS_MNAGER_ROS2_CPP
#define COMPOSITION__TRACKERS_MNAGER_ROS2_CPP

// #include "composition/visibility_control.h"
#include <rclcpp/rclcpp.hpp>

#include <kr_trackers_msgs/msg/tracker_status.hpp>
#include <kr_trackers_msgs/srv/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <kr_trackers_manager/tracker.hpp>
#include <pluginlib/class_loader.h>

//namespace composition {

class TrackersManager : public rclcpp::Node
{
 public:
  TrackersManager(const rclcpp::NodeOptions & options);
  ~TrackersManager();

  void OnInit(void);

 private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // bool transition_callback(
  //   const std::shared_ptr<rmw_request_id_t> request_header,
  //   const std::shared_ptr<kr_tracker_msgs::srv::Transition::Request> request,
  //   std::shared_ptr<kr_tracker_msgs::srv::Transition::Response> response
  // );

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  //rclcpp::Publisher<kr_mav_msgs::msg::PositionCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<kr_trackers_msgs::msg::TrackerStatus>::SharedPtr pub_status_;

  // rclcpp::Service<kr_tracker_msgs::srv::Transition>::SharedPtr srv_tracker_;

  std::unique_ptr<pluginlib::ClassLoader<kr_trackers_manager::Tracker>> tracker_loader_;
  // kr_trackers_manager::Tracker::SharedPtr active_tracker_;
  // std::map<std::string, kr_trackers_manager::Tracker::SharedPtr> tracker_map_;
  // kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd_;
};

TrackersManager::TrackersManager(const rclcpp::NodeOptions & options)
: Node("tracker", options), 
  tracker_loader_("kr_trackers_manager", "kr_trackers_manager::Tracker") 
//  active_tracker_(NULL)
//  : Node("trackers_manager"), tracker_loader_("kr_trackers_manager", "kr_trackers_manager::Tracker")
{
  // pub_cmd_ = create_publisher<kr_mav_msgs::msg::PositionCommand>("cmd", 10);
  pub_status_ = create_publisher<kr_trackers_msgs::msg::TrackerStatus>("status", 10);

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, 
      std::bind(&TrackersManager::odom_callback, this, std::placeholders::_1));
}

TrackersManager::~TrackersManager()
{
  // for(std::map<std::string, kr_trackers_manager::Tracker *>::iterator it = tracker_map_.begin();
  //     it != tracker_map_.end(); it++)
  // {
  //   delete it->second;
  //   try
  //   {
  //     tracker_loader_.unloadLibraryForClass(it->first);
  //   }
  //   catch(pluginlib::LibraryUnloadException &e)
  //   {
  //     RCLCPP_ERROR_STREAM("Could not unload library for the tracker " << it->first << ": " << e.what());
  //   }
  // }
}


void TrackersManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Your odom_callback implementation here.
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received odom!");
}

// bool TrackersManager::transition_callback(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<kr_tracker_msgs::srv::Transition::Request> request,
//   std::shared_ptr<kr_tracker_msgs::srv::Transition::Response> response)
// {
//   // Your transition_callback implementation here.
// }

//} // namespace
  
#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(TrackersManager)
#endif
