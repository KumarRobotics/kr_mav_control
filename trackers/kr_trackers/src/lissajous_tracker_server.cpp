#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LissajousTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers/lissajous_generator.h>
#include <kr_trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

class LissajousTracker : public kr_trackers_manager::Tracker
{
 public:
  LissajousTracker(void);
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

 private:
  void goal_callback(void);
  void preempt_callback(void);

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::LissajousTrackerAction> ServerType;
  std::shared_ptr<ServerType> tracker_server_;
  ros::Publisher path_pub_;

  InitialConditions ICs_;
  LissajousGenerator generator_;
  double distance_traveled_;
  Eigen::Vector3d position_last_;
  bool traj_start_set_;
  std::string frame_id_;
};

LissajousTracker::LissajousTracker(void) : traj_start_set_(false) {}

void LissajousTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "lissajous_tracker");
  priv_nh.param<std::string>("frame_id", frame_id_, "world");
  path_pub_ = priv_nh.advertise<nav_msgs::Path>("lissajous_path", 1);

  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LissajousTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&LissajousTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LissajousTracker::preempt_callback, this));
  tracker_server_->start();
}

bool LissajousTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(generator_.goalIsSet())
  {
    if(!tracker_server_->isActive())
    {
      ROS_WARN("LissajousTracker::Activate: goal_set is true but action server has no active goal - not activating.");
      return false;
    }
    return generator_.activate();
  }
  return false;
}

void LissajousTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("LissajousTracker deactivated tracker prior to reaching goal");
    tracker_server_->setAborted();
  }
  ICs_.reset();
  generator_.deactivate();
  traj_start_set_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr LissajousTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!generator_.isActive())
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  if(!traj_start_set_)
  {
    traj_start_set_ = true;
    ICs_.set_from_odom(msg);
    position_last_ = Eigen::Vector3d(ICs_.pos()(0), ICs_.pos()(1), ICs_.pos()(2));

    // Generate path for visualizing
    geometry_msgs::Point initial_pt;
    initial_pt.x = ICs_.pos()(0);
    initial_pt.y = ICs_.pos()(1);
    initial_pt.z = ICs_.pos()(2);
    double dt = 0.1;
    nav_msgs::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    generator_.generatePath(path, initial_pt, dt);
    path_pub_.publish(path);
  }

  // Set gains
  kr_mav_msgs::PositionCommand::Ptr cmd = generator_.getPositionCmd();
  if(cmd == NULL)
  {
    return cmd;
  }
  else
  {
    cmd->header.stamp = ros::Time::now();
    cmd->header.frame_id = msg->header.frame_id;
    cmd->position.x += ICs_.pos()(0);
    cmd->position.y += ICs_.pos()(1);
    cmd->position.z += ICs_.pos()(2);
    cmd->yaw += ICs_.yaw();

    // Publish feedback and compute distance traveled
    if(!generator_.status())
    {
      kr_tracker_msgs::LissajousTrackerFeedback feedback;
      feedback.time_to_completion = generator_.timeRemaining();
      tracker_server_->publishFeedback(feedback);

      Eigen::Vector3d position_current =
          Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      distance_traveled_ += (position_current - position_last_).norm();
      position_last_ = position_current;
    }
    else if(tracker_server_->isActive())
    {
      kr_tracker_msgs::LissajousTrackerResult result;
      result.x = msg->pose.pose.position.x;
      result.y = msg->pose.pose.position.y;
      result.z = msg->pose.pose.position.z;
      result.yaw = ICs_.yaw();  // TODO: Change this to the yaw from msg
      result.duration = generator_.timeElapsed();
      result.length = distance_traveled_;
      tracker_server_->setSucceeded(result);
    }
    return cmd;
  }
}

uint8_t LissajousTracker::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

void LissajousTracker::goal_callback(void)
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("Previous LissajousTracker goal aborted.");
    tracker_server_->setAborted();
    generator_.deactivate();
  }

  kr_tracker_msgs::LissajousTrackerGoal::ConstPtr msg = tracker_server_->acceptNewGoal();

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("LissajousTracker going to goal preempted.");
    tracker_server_->setPreempted();
    return;
  }

  generator_.setParams(msg);

  traj_start_set_ = false;
  distance_traveled_ = 0;
  generator_.activate();
}

void LissajousTracker::preempt_callback(void)
{
  ICs_.reset();
  generator_.deactivate();
  traj_start_set_ = false;

  if(tracker_server_->isActive())
  {
    tracker_server_->setAborted();
  }
  else
  {
    tracker_server_->setPreempted();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousTracker, kr_trackers_manager::Tracker);
