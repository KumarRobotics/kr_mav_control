#include <memory>
#include <lissajous_generator.h>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>
#include <trackers_msgs/TrackerStatus.h>
#include <actionlib/server/simple_action_server.h>
#include <trackers_msgs/LissajousAdderAction.h>
#include <Eigen/Geometry>
#include <initial_conditions.h>
#include <cmath>

class LissajousAdderAction : public trackers_manager::Tracker
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

    typedef actionlib::SimpleActionServer<trackers_msgs::LissajousAdderAction> ServerType;
    std::shared_ptr<ServerType> tracker_server_;

    ros::ServiceServer vel_control_srv_;
    double kx_[3], kv_[3];
    InitialConditions ICs_;
    LissajousGenerator generator_1_, generator_2_;
    double distance_traveled_;
    Eigen::Vector3d position_last_;
};

void LissajousAdderAction::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "lissajous_adder");
  vel_control_srv_ = priv_nh.advertiseService("vel_control", &LissajousAdderAction::velControlCallback, this);

  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LissajousAdderAction", false));
  tracker_server_->registerGoalCallback(boost::bind(&LissajousAdderAction::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LissajousAdderAction::preempt_callback, this));
  tracker_server_->start();
}

bool LissajousAdderAction::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if(!tracker_server_->isActive())
  {
    ROS_WARN("No goal set, not activating");
    return false;
  }
  return (generator_1_.activate() && generator_2_.activate());
}

void LissajousAdderAction::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("Deactivated tracker prior to reaching goal");
    tracker_server_->setAborted();
  }
  ICs_.reset();
  generator_1_.deactivate();
  generator_2_.deactivate();
}

bool LissajousAdderAction::velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  kx_[0] = 0;
  kx_[1] = 0;
  kx_[2] = 0;
  res.success = true;
  return true;
}

quadrotor_msgs::PositionCommand::ConstPtr LissajousAdderAction::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!(generator_1_.isActive() & generator_2_.isActive()))
  {
    ICs_.set_from_odom(msg);
    position_last_ = Eigen::Vector3d(ICs_.pos()(0), ICs_.pos()(1), ICs_.pos()(2));
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd1 = generator_1_.getPositionCmd();
  quadrotor_msgs::PositionCommand::Ptr cmd2 = generator_2_.getPositionCmd();
  if(cmd1 == NULL && cmd2 == NULL)
  {
    return cmd1;
  }
  else
  {
    cmd1->header.stamp = ros::Time::now();
    cmd1->header.frame_id = msg->header.frame_id;
    cmd1->kx[0] = kx_[0], cmd1->kx[1] = kx_[1], cmd1->kx[2] = kx_[2];
    cmd1->kv[0] = kv_[0], cmd1->kv[1] = kv_[1], cmd1->kv[2] = kv_[2];
    cmd1->position.x += ICs_.pos()(0) + cmd2->position.x;
    cmd1->position.y += ICs_.pos()(1) + cmd2->position.y;
    cmd1->position.z += ICs_.pos()(2) + cmd2->position.z;
    cmd1->velocity.x += cmd2->velocity.x;
    cmd1->velocity.y += cmd2->velocity.y;
    cmd1->velocity.z += cmd2->velocity.z;
    cmd1->acceleration.x += cmd2->acceleration.x;
    cmd1->acceleration.y += cmd2->acceleration.y;
    cmd1->acceleration.z += cmd2->acceleration.z;
    cmd1->jerk.x += cmd2->jerk.x;
    cmd1->jerk.y += cmd2->jerk.y;
    cmd1->jerk.z += cmd2->jerk.z;
    cmd1->yaw += ICs_.yaw() + cmd2->yaw;

    // Publish feedback and compute distance traveled
    if(!generator_1_.status() || !generator_2_.status())
    {
      trackers_msgs::LissajousAdderFeedback feedback;
      feedback.time_to_completion = std::max(generator_1_.timeRemaining(), generator_2_.timeRemaining());
      tracker_server_->publishFeedback(feedback);

      Eigen::Vector3d position_current = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      distance_traveled_ += (position_current - position_last_).norm();
      position_last_ = position_current;
    }
    else if(tracker_server_->isActive())
    {
      trackers_msgs::LissajousAdderResult result;
      result.x = msg->pose.pose.position.x;
      result.y = msg->pose.pose.position.y;
      result.z = msg->pose.pose.position.z;
      result.yaw = ICs_.yaw(); // TODO: Change this to the yaw from msg
      result.duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
      result.length = distance_traveled_;
      tracker_server_->setSucceeded(result);
    }
    return cmd1;
  }
}

uint8_t LissajousAdderAction::status() const
{
  return tracker_server_->isActive() ?
             static_cast<uint8_t>(trackers_msgs::TrackerStatus::ACTIVE) :
             static_cast<uint8_t>(trackers_msgs::TrackerStatus::SUCCEEDED);
}

void LissajousAdderAction::goal_callback(void)
{
  if (generator_1_.goalIsSet() || generator_2_.goalIsSet())
  {
    return;
  }

  trackers_msgs::LissajousAdderGoal::ConstPtr msg = tracker_server_->acceptNewGoal();

  if (tracker_server_->isPreemptRequested())
  {
    tracker_server_->setPreempted();
    return;
  }

  generator_1_.setParams(msg,0);
  generator_2_.setParams(msg,1);
  distance_traveled_ = 0;
}

void LissajousAdderAction::preempt_callback(void)
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
PLUGINLIB_EXPORT_CLASS(LissajousAdderAction, trackers_manager::Tracker);
