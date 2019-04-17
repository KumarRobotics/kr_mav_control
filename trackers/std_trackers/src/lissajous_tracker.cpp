#include <std_trackers/LissajousGenerator.h>
#include <iostream>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/LissajousTrackerGoal.h>
#include <Eigen/Geometry>
#include <initial_conditions.h>
#include <cmath>

class LissajousTracker : public trackers_manager::Tracker
{
  public:
    LissajousTracker(void);

    void Initialize(const ros::NodeHandle &nh);
    bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
    void Deactivate(void);

    bool velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
    uint8_t status() const;

  private:
    void goal_callback(const quadrotor_msgs::LissajousTrackerGoal::ConstPtr &msg);

    ros::Subscriber sub_goal_;
    ros::ServiceServer vel_control_srv_;
    double kx_[3], kv_[3];
    InitialConditions ICs_;
    LissajousGenerator generator_;
};

LissajousTracker::LissajousTracker(void) 
{
}

void LissajousTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "lissajous_tracker");
  sub_goal_ = priv_nh.subscribe("goal", 10, &LissajousTracker::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
  vel_control_srv_ = priv_nh.advertiseService("vel_control", &LissajousTracker::velControlCallback, this);
  std::cout << "calling from tracker" << std::endl;
}

bool LissajousTracker::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  return generator_.activate();
}

void LissajousTracker::Deactivate(void)
{
  ICs_.reset();
  generator_.deactivate();
}

bool LissajousTracker::velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  kx_[0] = 0;
  kx_[1] = 0;
  kx_[2] = 0;
  res.success = true;
  return true;
}

quadrotor_msgs::PositionCommand::ConstPtr LissajousTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!generator_.isActive())
  {
    ICs_.set_from_odom(msg);
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd = generator_.getPositionCmd();
  if(cmd != NULL)
  {
    cmd->header.stamp = ros::Time::now();
    cmd->header.frame_id = msg->header.frame_id;
    cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
    cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];
    cmd->position.x += ICs_.pos()(0);
    cmd->position.y += ICs_.pos()(1);
    cmd->position.z += ICs_.pos()(2);
    cmd->yaw += ICs_.yaw();
  }
  return cmd;
}

void LissajousTracker::goal_callback(const quadrotor_msgs::LissajousTrackerGoal::ConstPtr &msg)
{
  generator_.setParams(msg);
  ROS_WARN("lissajous trajectory successfully initialized");
}

uint8_t LissajousTracker::status() const
{
  return static_cast<uint8_t>(generator_.status());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousTracker, trackers_manager::Tracker);
