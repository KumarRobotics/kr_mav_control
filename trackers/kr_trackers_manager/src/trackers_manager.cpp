#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/Transition.h>
#include <kr_trackers_manager/Tracker.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

class TrackersManager : public nodelet::Nodelet
{
 public:
  TrackersManager(void);
  ~TrackersManager(void);

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  bool transition_callback(kr_tracker_msgs::Transition::Request &req, kr_tracker_msgs::Transition::Response &res);

  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_, pub_status_;
  ros::ServiceServer srv_tracker_;
  pluginlib::ClassLoader<kr_trackers_manager::Tracker> tracker_loader_;
  kr_trackers_manager::Tracker *active_tracker_;
  std::map<std::string, kr_trackers_manager::Tracker *> tracker_map_;
  kr_mav_msgs::PositionCommand::ConstPtr cmd_;
};

TrackersManager::TrackersManager(void)
    : tracker_loader_("kr_trackers_manager", "kr_trackers_manager::Tracker"), active_tracker_(NULL)
{
}

TrackersManager::~TrackersManager(void)
{
  for(std::map<std::string, kr_trackers_manager::Tracker *>::iterator it = tracker_map_.begin();
      it != tracker_map_.end(); it++)
  {
    delete it->second;
#if ROS_VERSION_MINIMUM(1, 8, 0)
    try
    {
      tracker_loader_.unloadLibraryForClass(it->first);
    }
    catch(pluginlib::LibraryUnloadException &e)
    {
      NODELET_ERROR_STREAM("Could not unload library for the tracker " << it->first << ": " << e.what());
    }
#endif
  }
}

void TrackersManager::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  XmlRpc::XmlRpcValue tracker_list;
  priv_nh.getParam("trackers", tracker_list);
  ROS_ASSERT(tracker_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < tracker_list.size(); i++)
  {
    ROS_ASSERT(tracker_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    const std::string tracker_name = static_cast<const std::string>(tracker_list[i]);
    try
    {
#if ROS_VERSION_MINIMUM(1, 8, 0)
      kr_trackers_manager::Tracker *c = tracker_loader_.createUnmanagedInstance(tracker_name);
#else
      kr_trackers_manager::Tracker *c = tracker_loader_.createClassInstance(tracker_name);
#endif
      c->Initialize(priv_nh);
      tracker_map_.insert(std::make_pair(tracker_name, c));
    }
    catch(pluginlib::LibraryLoadException &e)
    {
      NODELET_ERROR_STREAM("Could not load library for the tracker " << tracker_name << ": " << e.what());
    }
    catch(pluginlib::CreateClassException &e)
    {
      NODELET_ERROR_STREAM("Could not create an instance of the tracker " << tracker_name << ": " << e.what());
    }
  }

  pub_cmd_ = priv_nh.advertise<kr_mav_msgs::PositionCommand>("cmd", 10);
  pub_status_ = priv_nh.advertise<kr_tracker_msgs::TrackerStatus>("status", 10);

  sub_odom_ = priv_nh.subscribe("odom", 10, &TrackersManager::odom_callback, this, ros::TransportHints().tcpNoDelay());

  srv_tracker_ = priv_nh.advertiseService("transition", &TrackersManager::transition_callback, this);
}

void TrackersManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::map<std::string, kr_trackers_manager::Tracker *>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    if(it->second == active_tracker_)
    {
      cmd_ = it->second->update(msg);
      if(cmd_ != NULL)
        pub_cmd_.publish(cmd_);

      kr_tracker_msgs::TrackerStatus::Ptr status_msg(new kr_tracker_msgs::TrackerStatus);
      status_msg->header.stamp = msg->header.stamp;
      status_msg->tracker = it->first;
      status_msg->status = it->second->status();
      pub_status_.publish(status_msg);
    }
    else
    {
      it->second->update(msg);
    }
  }
}

bool TrackersManager::transition_callback(kr_tracker_msgs::Transition::Request &req,
                                          kr_tracker_msgs::Transition::Response &res)
{
  const std::map<std::string, kr_trackers_manager::Tracker *>::iterator it = tracker_map_.find(req.tracker);
  if(it == tracker_map_.end())
  {
    res.success = false;
    res.message = std::string("Cannot find tracker ") + req.tracker + std::string(", cannot transition");
    NODELET_WARN_STREAM(res.message);
    return true;
  }
  if(active_tracker_ == it->second)
  {
    res.success = true;
    res.message = std::string("Tracker ") + req.tracker + std::string(" already active");
    NODELET_INFO_STREAM(res.message);
    return true;
  }

  if(!it->second->Activate(cmd_))
  {
    res.success = false;
    res.message = std::string("Failed to activate tracker ") + req.tracker + std::string(", cannot transition");
    NODELET_WARN_STREAM(res.message);
    return true;
  }

  if(active_tracker_ != NULL)
  {
    active_tracker_->Deactivate();
  }

  active_tracker_ = it->second;
  res.success = true;
  res.message = std::string("Successfully activated tracker ") + req.tracker;
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrackersManager, nodelet::Nodelet);
