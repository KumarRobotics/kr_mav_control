#include <memory>
#include <lissajous_generator.h>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>
#include <tracker_msgs/TrackerStatus.h>
#include <actionlib/server/simple_action_server.h>
#include <tracker_msgs/LissajousTrackerAction.h>
#include <Eigen/Geometry>
#include <initial_conditions.h>
#include <cmath>

class LissajousTrackerAction : public trackers_manager::Tracker
{
  public:
    void Initialize(const ros::NodeHandle &nh);
    bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
    void Deactivate(void);

    bool velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
    uint8_t status() const;

  private:
    void goal_callback(void);
    void preempt_callback(void);

    typedef actionlib::SimpleActionServer<tracker_msgs::LissajousTrackerAction> ServerType;
    std::shared_ptr<ServerType> tracker_server_;

    ros::ServiceServer vel_control_srv_;
    double kx_[3], kv_[3];
    InitialConditions ICs_;
    LissajousGenerator generator_;
    double distance_traveled_;
    Eigen::Vector3d position_last_;
};

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
  tracker_server_->start();
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
    position_last_ = Eigen::Vector3d(ICs_.pos()(0), ICs_.pos()(1), ICs_.pos()(2));
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

    // Publish feedback and compute distance traveled
    if(!generator_.status())
    {
      tracker_msgs::LissajousTrackerFeedback feedback;
      feedback.time_to_completion = generator_.timeRemaining();
      tracker_server_->publishFeedback(feedback);

      Eigen::Vector3d position_current = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      distance_traveled_ += (position_current - position_last_).norm();
      position_last_ = position_current;
    }
    else if(tracker_server_->isActive())
    {
      tracker_msgs::LissajousTrackerResult result;
      result.x = msg->pose.pose.position.x;
      result.y = msg->pose.pose.position.y;
      result.z = msg->pose.pose.position.z;
      result.yaw = ICs_.yaw(); // TODO: Change this to the yaw from msg
      result.duration = generator_.timeElapsed();
      result.length = distance_traveled_;
      tracker_server_->setSucceeded(result);
    }
    return cmd;
  }
}

uint8_t LissajousTrackerAction::status() const
{
  return tracker_server_->isActive() ?
             static_cast<uint8_t>(tracker_msgs::TrackerStatus::ACTIVE) :
             static_cast<uint8_t>(tracker_msgs::TrackerStatus::SUCCEEDED);
}

void LissajousTrackerAction::goal_callback(void)
{
  if (generator_.goalIsSet())
  {
    return;
  }

  tracker_msgs::LissajousTrackerGoal::ConstPtr msg = tracker_server_->acceptNewGoal();

  if (tracker_server_->isPreemptRequested())
  {
    tracker_server_->setPreempted();
    return;
  }

  generator_.setParams(msg);
  distance_traveled_ = 0;
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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousTrackerAction, trackers_manager::Tracker);
