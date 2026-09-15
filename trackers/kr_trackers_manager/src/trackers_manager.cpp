#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_tracker_msgs/srv/transition.hpp"
#include "kr_trackers/Tracker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include <pluginlib/class_loader.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrackersManager : public rclcpp_lifecycle::LifecycleNode
{
 public:
  TrackersManager(const rclcpp::NodeOptions &options);
  ~TrackersManager(void);

  // std::shared_ptr<kr_trackers_manager::Tracker> ptr_;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

 protected:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void transition_callback(const kr_tracker_msgs::srv::Transition::Request::SharedPtr req,
                           const kr_tracker_msgs::srv::Transition::Response::SharedPtr res);
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<kr_mav_msgs::msg::PositionCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<kr_tracker_msgs::msg::TrackerStatus>::SharedPtr pub_status_;
  rclcpp::Service<kr_tracker_msgs::srv::Transition>::SharedPtr srv_tracker_;
  pluginlib::ClassLoader<kr_trackers_manager::Tracker> tracker_loader_;
  // kr_trackers_manager::Tracker *active_tracker; // switched this to use smart pointers
  std::shared_ptr<kr_trackers_manager::Tracker> active_tracker_;
  // std::map<std::string, kr_trackers_manager::Tracker *> tracker_map_; // switched this to use smart pointers
  std::map<std::string, std::shared_ptr<kr_trackers_manager::Tracker>> tracker_map_;
  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd_;
};

TrackersManager::TrackersManager(const rclcpp::NodeOptions &options)
    : LifecycleNode("trackers_manager", options),
      tracker_loader_("kr_trackers", "kr_trackers_manager::Tracker")
{
  RCLCPP_INFO(this->get_logger(), "IN TRACKERS MANAGER CONSTRUCTOR");
}

TrackersManager::~TrackersManager(void)
{
}

LifecycleCallbackReturn TrackersManager::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;
  
  RCLCPP_INFO(this->get_logger(), "Configuring TrackersManager and Trackers.");

  auto node = shared_from_this();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_node = node;

  this->declare_parameter<std::vector<std::string>>("trackers", {"LineTrackerDistance"});
  std::vector<std::string> trackers_array;
  this->get_parameter("trackers", trackers_array);
  auto param_names = this->list_parameters({}, 10).names;

  // Print each parameter name and its value
  for (const auto &name : param_names) {
      rclcpp::Parameter param;
      if (this->get_parameter(name, param)) {
          RCLCPP_INFO(this->get_logger(), "Parameter: %s = %s", 
                      name.c_str(), param.value_to_string().c_str());
      }
  }

  for(auto name: trackers_array)
  {
    try
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Adding " << name.c_str() << " to tracker_map_");
      auto ptr = tracker_loader_.createSharedInstance(name);
      ptr->Initialize(weak_node);
      RCLCPP_INFO(this->get_logger(), "Loaded successfully");
      tracker_map_.insert(std::make_pair(name, ptr));
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Library Load Exception: %s", e.what());
    }
    catch (const pluginlib::CreateClassException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Create Class Exception: %s", e.what());
    }
  }

  pub_cmd_ = this->create_publisher<kr_mav_msgs::msg::PositionCommand>("~/cmd", 10);
  pub_status_ = this->create_publisher<kr_tracker_msgs::msg::TrackerStatus>("~/status", 10);

  // Setting QoS profile to get equivalent performance to ros::TransportHints().tcpNoDelay()
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("control_odom", qos, std::bind(&TrackersManager::odom_callback, this, std::placeholders::_1));

  srv_tracker_ = this->create_service<kr_tracker_msgs::srv::Transition>("~/transition", std::bind(&TrackersManager::transition_callback, this, std::placeholders::_1, std::placeholders::_2));

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TrackersManager::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating TrackersManager.");
  rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
  return LifecycleCallbackReturn::SUCCESS;
}

void TrackersManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::map<std::string, std::shared_ptr<kr_trackers_manager::Tracker>>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    if(it->second == active_tracker_)
    {
      cmd_ = it->second->update(msg);
      if(cmd_ != NULL)
      {
        // TODO: make cmd_ a unique pointer
        pub_cmd_->publish(*cmd_);
      }

      auto status_msg = std::make_unique<kr_tracker_msgs::msg::TrackerStatus>();
      status_msg->header.stamp = msg->header.stamp;
      status_msg->tracker = it->first;
      status_msg->status = it->second->status();
      pub_status_->publish(std::move(status_msg));
    }
    else
    {
      it->second->update(msg);
    }
  }
}

void TrackersManager::transition_callback(const kr_tracker_msgs::srv::Transition::Request::SharedPtr req, const kr_tracker_msgs::srv::Transition::Response::SharedPtr res)
{
  RCLCPP_INFO(this->get_logger(), "TRANSITION CALLBACK:");
  RCLCPP_INFO(this->get_logger(),  req->tracker.c_str());
  const std::map<std::string, std::shared_ptr<kr_trackers_manager::Tracker>>::iterator it = tracker_map_.find(req->tracker);
  if(it == tracker_map_.end())
  {
    res->success = false;
    res->message = std::string("Cannot find tracker ") + req->tracker + std::string(", cannot transition");
    RCLCPP_WARN_STREAM(this->get_logger(), res->message);
    return;
  }
  if(active_tracker_ == it->second)
  {
    res->success = true;
    res->message = std::string("Tracker ") + req->tracker + std::string(" already active");
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    return;
  }

  // TODO: change arguments of Activate function to take in a ConstSharedPtr
  if(!it->second->Activate(cmd_))
  {
    res->success = false;
    res->message = std::string("Failed to activate tracker ") + req->tracker + std::string(", cannot transition");
    RCLCPP_WARN_STREAM(this->get_logger(), res->message);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "TRANSITION CALLBACK 2:");

  if(active_tracker_ != NULL)
  {
    active_tracker_->Deactivate();
  }

  RCLCPP_INFO(this->get_logger(), "TRANSITION CALLBACK 3:");
  active_tracker_ = it->second;
  res->success = true;
  res->message = std::string("Successfully activated tracker ") + req->tracker;
}

LifecycleCallbackReturn TrackersManager::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(this->get_logger(), "Shutting down Trackers Manager");
  return LifecycleCallbackReturn::SUCCESS;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TrackersManager)
