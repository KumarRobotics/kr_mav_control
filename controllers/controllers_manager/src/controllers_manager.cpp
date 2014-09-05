#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <pluginlib/class_loader.h>
#include <controllers_manager/Controller.h>
#include <controllers_manager/Transition.h>

class ControllersManager : public nodelet::Nodelet
{
 public:
  ControllersManager(void);
  ~ControllersManager(void);

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  bool transition_callback(controllers_manager::Transition::Request &req,
                           controllers_manager::Transition::Response &res);

  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_;
  ros::ServiceServer srv_controller_;
  pluginlib::ClassLoader<controllers_manager::Controller> *controller_loader_;
  controllers_manager::Controller *active_controller_;
  std::map<std::string, controllers_manager::Controller*> controller_map_;
};

ControllersManager::ControllersManager(void) :
    active_controller_(NULL)
{
  controller_loader_ = new pluginlib::ClassLoader<controllers_manager::Controller>("controllers_manager",
                                                                                   "controllers_manager::Controller");
}

ControllersManager::~ControllersManager(void)
{
  std::map<std::string, controllers_manager::Controller*>::iterator it;
  for(it = controller_map_.begin(); it != controller_map_.end(); it++)
  {
    delete it->second;
#if ROS_VERSION_MINIMUM(1,8,0)
    controller_loader_->unloadLibraryForClass(it->first);
#endif
  }
  delete controller_loader_;
}

void ControllersManager::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  XmlRpc::XmlRpcValue controller_list;
  priv_nh.getParam("controllers", controller_list);
  ROS_ASSERT(controller_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < controller_list.size(); i++)
  {
    ROS_ASSERT(controller_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    const std::string controller_name = static_cast<const std::string>(controller_list[i]);
    try
    {
#if ROS_VERSION_MINIMUM(1,8,0)
      controllers_manager::Controller *c = controller_loader_->createUnmanagedInstance(controller_name);
#else
      controllers_manager::Controller *c = controller_loader_->createClassInstance(controller_name);
#endif
      c->Initialize(priv_nh);
      controller_map_.insert(std::make_pair(controller_name, c));
    }
    catch(pluginlib::LibraryLoadException &e)
    {
      NODELET_ERROR_STREAM("Could not load library for the controller " << controller_name);
    }
    catch(pluginlib::CreateClassException &e)
    {
      NODELET_ERROR_STREAM("Could not create an instance of the controller " << controller_name);
    }
  }

  sub_odom_ = priv_nh.subscribe("odom", 10, &ControllersManager::odom_callback, this,
                                ros::TransportHints().tcpNoDelay());

  pub_cmd_ = priv_nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);

  srv_controller_ = priv_nh.advertiseService("transition", &ControllersManager::transition_callback, this);
}

void ControllersManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::map<std::string, controllers_manager::Controller*>::iterator it;
  for(it = controller_map_.begin(); it != controller_map_.end(); it++)
  {
    if(it->second == active_controller_)
    {
      const quadrotor_msgs::PositionCommand::ConstPtr cmd = it->second->update(msg);
      if(cmd != NULL)
        pub_cmd_.publish(cmd);
    }
    else
    {
      it->second->update(msg);
    }
  }
}

bool ControllersManager::transition_callback(controllers_manager::Transition::Request &req,
                                             controllers_manager::Transition::Response &res)
{
  const std::map<std::string, controllers_manager::Controller*>::iterator it = controller_map_.find(req.controller);
  if(it == controller_map_.end())
  {
    NODELET_WARN_STREAM("Cannot find controller " << req.controller << ", cannot transition");
    return false;
  }
  if(active_controller_ == it->second)
  {
    NODELET_INFO_STREAM("Controller " << req.controller << " already active");
    return true;
  }

  if(!it->second->Activate())
  {
    NODELET_WARN_STREAM("Failed to activate controller " << req.controller << ", cannot transition");
    return false;
  }

  if(active_controller_ != NULL)
    active_controller_->Deactivate();

  active_controller_ = it->second;
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ControllersManager, nodelet::Nodelet);
