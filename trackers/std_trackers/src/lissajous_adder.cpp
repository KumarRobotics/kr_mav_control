#include <lissajous_generator.h>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <std_srvs/Trigger.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/LissajousAdderGoal.h>
#include <Eigen/Geometry>
#include <initial_conditions.h>
#include <cmath>
#include <iostream>

#define PI 3.14159265359

class LissajousAdder : public trackers_manager::Tracker
{
  public:
    LissajousAdder(void);

    void Initialize(const ros::NodeHandle &nh);
    bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
    void Deactivate(void);

    bool velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
    uint8_t status() const;

  private:
    void goal_callback(const quadrotor_msgs::LissajousAdderGoal::ConstPtr &msg);

    ros::Subscriber sub_goal_;
    ros::ServiceServer vel_control_srv_;
    double kx_[3], kv_[3];
    InitialConditions ICs_;
    LissajousGenerator generator_1_, generator_2_;
};

LissajousAdder::LissajousAdder(void) 
{
}

void LissajousAdder::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "lissajous_adder");
  sub_goal_ = priv_nh.subscribe("goal", 10, &LissajousAdder::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
  vel_control_srv_ = priv_nh.advertiseService("vel_control", &LissajousAdder::velControlCallback, this);
}

bool LissajousAdder::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  return (generator_1_.activate() && generator_2_.activate());
}

void LissajousAdder::Deactivate(void)
{
  ICs_.reset();
  generator_1_.deactivate();
  generator_2_.deactivate();
}

bool LissajousAdder::velControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  kx_[0] = 0;
  kx_[1] = 0;
  kx_[2] = 0;
  res.success = true;
  return true;
}

quadrotor_msgs::PositionCommand::ConstPtr LissajousAdder::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!(generator_1_.isActive() & generator_2_.isActive()))
  {
    ICs_.set_from_odom(msg);
  }

  // Set gains
  quadrotor_msgs::PositionCommand::Ptr cmd1 = generator_1_.getPositionCmd();
  quadrotor_msgs::PositionCommand::Ptr cmd2 = generator_2_.getPositionCmd();
  if(cmd1 != NULL && cmd2 != NULL)
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
  }
  return cmd1;
}

void LissajousAdder::goal_callback(const quadrotor_msgs::LissajousAdderGoal::ConstPtr &msg)
{
  generator_1_.setParams(msg,0);
  generator_2_.setParams(msg,1);
  ROS_WARN("lissajous adder successfully initialized");
}

uint8_t LissajousAdder::status() const
{
  bool status_1 = generator_1_.status();
  bool status_2 = generator_2_.status();
  return static_cast<uint8_t>(status_1 && status_2);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousAdder, trackers_manager::Tracker);
