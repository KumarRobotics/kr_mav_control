#include <memory>
#include <lissajous_generator.h>
#include <iostream>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>
#include <trackers_manager/TrackerStatus.h>
#include <actionlib/server/simple_action_server.h>
#include <std_trackers/LissajousTrackerAction.h>
#include <Eigen/Geometry>
#include <initial_conditions.h>
#include <cmath>

class LissajousTrackerAction : public trackers_manager::Tracker
{
  public:
    LissajousTrackerAction(void);

    void Initialize(const ros::NodeHandle &nh);
    bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
    void Deactivate(void);

    bool velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
    uint8_t status() const;

  private:
    void goal_callback(void);
    void preempt_callback(void);

    typedef actionlib::SimpleActionServer<std_trackers::LissajousTrackerAction> ServerType;
    std::shared_ptr<ServerType> tracker_server_;

    ros::ServiceServer vel_control_srv_;
    double kx_[3], kv_[3];
    InitialConditions ICs_;
    LissajousGenerator generator_;
};

LissajousTrackerAction::LissajousTrackerAction(void) 
{
}

void LissajousTrackerAction::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "lissajous_tracker");
  vel_control_srv_ = priv_nh.advertiseService("vel_control", &LissajousTrackerAction::velControlCallback, this);

  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LissajousTrackerAction", false));
  tracker_server_->registerGoalCallback(boost::bind(&LissajousTrackerAction::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LissajousTrackerAction::preempt_callback, this));

  std::cout << "calling from tracker" << std::endl;
}

bool LissajousTrackerAction::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if(!tracker_server_->isActive())
  {
    ROS_WARN("No goal set, not activating");
    return false;
  }
  return generator_.activate();
}

void LissajousTrackerAction::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("Deactivated tracker prior to reaching goal");
    tracker_server_->setAborted();
  }
  ICs_.reset();
  generator_.deactivate();
}

bool LissajousTrackerAction::velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  kx_[0] = 0;
  kx_[1] = 0;
  kx_[2] = 0;
  res.success = true;
  return true;
}

quadrotor_msgs::PositionCommand::ConstPtr LissajousTrackerAction::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!generator_.isActive())
  {
    ICs_.set_from_odom(msg);
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd = generator_.getPositionCmd();
  if(cmd == NULL)
  {
    return cmd;
  }
  else
  {
    cmd->header.stamp = ros::Time::now();
    cmd->header.frame_id = msg->header.frame_id;
    cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
    cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];
    cmd->position.x += ICs_.pos()(0);
    cmd->position.y += ICs_.pos()(1);
    cmd->position.z += ICs_.pos()(2);
    cmd->yaw += ICs_.yaw(); 

    // Publish feedback
    if(!generator_.status())
    {
      std_trackers::LissajousTrackerFeedback feedback;
      feedback.time_to_completion = generator_.timeRemaining();
      tracker_server_->publishFeedback(feedback);
    } 
    else if(tracker_server_->isActive())
    {
      tracker_server_->setSucceeded();
    }

    return cmd;
  }
}

uint8_t LissajousTrackerAction::status() const
{
  return tracker_server_->isActive() ?
             static_cast<uint8_t>(trackers_manager::TrackerStatus::ACTIVE) :
             static_cast<uint8_t>(trackers_manager::TrackerStatus::SUCCEEDED);
}

void LissajousTrackerAction::goal_callback(void)
{
  if (tracker_server_->isActive()) 
  {
    tracker_server_->setAborted();
  }

  if (tracker_server_->isPreemptRequested()) 
  {
    tracker_server_->setPreempted();
    return;
  }

  std_trackers::LissajousTrackerGoal::ConstPtr msg = tracker_server_->acceptNewGoal();
  generator_.setParams(msg);
}

void LissajousTrackerAction::preempt_callback(void)
{
  if (tracker_server_->isActive()) 
  {
    tracker_server_->setAborted();
  }
  else 
  {
    tracker_server_->setPreempted();
  }
  generator_.deactivate(); 
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousTrackerAction, trackers_manager::Tracker);
