#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_loader.h>
#include <trackers_manager/Tracker.h>
#include <trackers_manager/Transition.h>

class TrackersManager : public nodelet::Nodelet
{
 public:
  TrackersManager(void);
  ~TrackersManager(void);

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  bool transition_callback(trackers_manager::Transition::Request &req,
                           trackers_manager::Transition::Response &res);

  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_cmd_odom_;
  ros::Publisher pub_status_;
  ros::ServiceServer srv_tracker_;
  pluginlib::ClassLoader<trackers_manager::Tracker> *tracker_loader_;
  trackers_manager::Tracker *active_tracker_;
  std::map<std::string, trackers_manager::Tracker*> tracker_map_;
  quadrotor_msgs::PositionCommand::ConstPtr cmd_;
};

TrackersManager::TrackersManager(void) :
    active_tracker_(NULL)
{
  tracker_loader_ = new pluginlib::ClassLoader<trackers_manager::Tracker>("trackers_manager",
                                                                                   "trackers_manager::Tracker");
}

TrackersManager::~TrackersManager(void)
{
  std::map<std::string, trackers_manager::Tracker*>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    delete it->second;
#if ROS_VERSION_MINIMUM(1,8,0)
    tracker_loader_->unloadLibraryForClass(it->first);
#endif
  }
  delete tracker_loader_;
}

void TrackersManager::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  ros::NodeHandle nh(getNodeHandle());

  XmlRpc::XmlRpcValue tracker_list;
  priv_nh.getParam("trackers", tracker_list);
  ROS_ASSERT(tracker_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < tracker_list.size(); i++)
  {
    ROS_ASSERT(tracker_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    const std::string tracker_name = static_cast<const std::string>(tracker_list[i]);
    try
    {
#if ROS_VERSION_MINIMUM(1,8,0)
      trackers_manager::Tracker *c = tracker_loader_->createUnmanagedInstance(tracker_name);
#else
      trackers_manager::Tracker *c = tracker_loader_->createClassInstance(tracker_name);
#endif
      c->Initialize(priv_nh, nh);
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

  sub_odom_ = priv_nh.subscribe("odom", 10, &TrackersManager::odom_callback, this,
                                ros::TransportHints().tcpNoDelay());

  pub_cmd_ = priv_nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);
  pub_cmd_odom_ = priv_nh.advertise<nav_msgs::Odometry>("cmd_odom", 10);
  pub_status_ = priv_nh.advertise<quadrotor_msgs::TrackerStatus>("status", 10);

  srv_tracker_ = priv_nh.advertiseService("transition", &TrackersManager::transition_callback, this);
}

void TrackersManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::map<std::string, trackers_manager::Tracker*>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    if(it->second == active_tracker_)
    {
      cmd_ = it->second->update(msg);
      if(cmd_ != NULL)
      {
        pub_cmd_.publish(cmd_);

        nav_msgs::Odometry cmd_odom_msg;
        cmd_odom_msg.header.stamp = cmd_->header.stamp;
        cmd_odom_msg.header.frame_id = "local_origin";
        cmd_odom_msg.pose.pose.position = cmd_->position;
        tf::Quaternion q;
        q.setEuler(0,0,cmd_->yaw);
        cmd_odom_msg.pose.pose.orientation.x = q.x();
        cmd_odom_msg.pose.pose.orientation.y = q.y();
        cmd_odom_msg.pose.pose.orientation.z = q.z();
        cmd_odom_msg.pose.pose.orientation.w = q.w();

        pub_cmd_odom_.publish(cmd_odom_msg);
      }

      const quadrotor_msgs::TrackerStatus::Ptr status = it->second->status();
      if(status != NULL)
      {
        status->stamp = ros::Time::now();
        status->tracker = it->first;
        pub_status_.publish(status);
      }
    }
    else
    {
      it->second->update(msg);
    }
  }
}

bool TrackersManager::transition_callback(trackers_manager::Transition::Request &req,
                                             trackers_manager::Transition::Response &res)
{
  const std::map<std::string, trackers_manager::Tracker*>::iterator it = tracker_map_.find(req.tracker);
  if(it == tracker_map_.end())
  {
    NODELET_WARN_STREAM("Cannot find tracker " << req.tracker << ", cannot transition");
    return false;
  }
  if(active_tracker_ == it->second)
  {
    NODELET_INFO_STREAM("Tracker " << req.tracker << " already active");
    return true;
  }

  if(!it->second->Activate(cmd_))
  {
    NODELET_WARN_STREAM("Failed to activate tracker " << req.tracker << ", cannot transition");
    return false;
  }

  if(active_tracker_ != NULL)
    active_tracker_->Deactivate();

  active_tracker_ = it->second;
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrackersManager, nodelet::Nodelet);
