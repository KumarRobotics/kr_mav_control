#include <cmath>
#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
#include <std_trackers/VelocityTrackerAction.h>

class VelocityTrackerAction : public trackers_manager::Tracker
{
public:
  VelocityTrackerAction(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
// const quadrotor_msgs::TrackerStatus::Ptr status();

private:
  void goal_callback();

  void preempt_callback();

  /*  void velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);
    void position_velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);*/

  typedef actionlib::SimpleActionServer<std_trackers::VelocityTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::shared_ptr<ServerType> tracker_server_;

  //ros::Subscriber sub_vel_cmd_, sub_position_vel_cmd_;
  quadrotor_msgs::PositionCommand position_cmd_;
  bool odom_set_, active_, use_position_gains_;
  double last_t_;
  double kx_[3], kv_[3];
  double pos_[3], cur_yaw_;

  // Time taken to get to the goal.
  float current_traj_duration_;
  // Distance traveled to get to last goal.
  float current_traj_length_;
};

VelocityTrackerAction::VelocityTrackerAction(void) :
  odom_set_(false),
  active_(false),
  use_position_gains_(false),
  last_t_(0) {}

void VelocityTrackerAction::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "velocity_tracker");

/*  sub_vel_cmd_ = priv_nh.subscribe("goal", 10, &VelocityTrackerAction::velocity_cmd_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  sub_position_vel_cmd_ = priv_nh.subscribe("position_velocity_goal", 10, &VelocityTrackerAction::position_velocity_cmd_cb,
                          this, ros::TransportHints().tcpNoDelay());*/

  position_cmd_.kv[0] = kv_[0], position_cmd_.kv[1] = kv_[1], position_cmd_.kv[2] = kv_[2];

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "VelocityTrackerAction", false));
  tracker_server_->registerGoalCallback(boost::bind(&VelocityTrackerAction::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&VelocityTrackerAction::preempt_callback, this));

  tracker_server_->start();
}

bool VelocityTrackerAction::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if (cmd)
  {
    position_cmd_.position = cmd->position;
    position_cmd_.yaw = cmd->yaw;

    active_ = true;
  }
  else if (odom_set_)
  {
    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
    position_cmd_.yaw = cur_yaw_;

    active_ = true;
  }

  // Check server.
  if (active_ && !tracker_server_->isActive()) {
    ROS_WARN("VelocityTrackerAction::Activate: action server has no active goal - not activating.");
    active_ = false;
  }

  return active_;
}

void VelocityTrackerAction::Deactivate(void)
{
  if (tracker_server_->isActive()) {
    ROS_INFO("VelocityTrackerAction::Deactivate: deactivated tracker while still tracking the velocity.");
    // Consider this successful because we are tracking a
    // velocity indefinitely.
    std_trackers::VelocityTrackerResult result;
    result.duration = current_traj_duration_;
    result.length = current_traj_length_;
    result.x = pos_[0];
    result.y = pos_[1];
    result.z = pos_[2];
    result.yaw = cur_yaw_;
    tracker_server_->setSucceeded(result);

  }

  active_ = false;
  odom_set_ = false;
  last_t_ = 0;
}

const quadrotor_msgs::PositionCommand::ConstPtr VelocityTrackerAction::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Record distance between last position and current.
  const float dx = std::sqrt(std::pow(pos_[0] - msg->pose.pose.position.x, 2) +  std::pow(pos_[1] - msg->pose.pose.position.y, 2) + std::pow(pos_[2] - msg->pose.pose.position.z, 2));

  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;
  pos_[2] = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  odom_set_ = true;

  if (!active_) {
    return quadrotor_msgs::PositionCommand::Ptr();
  }


  if (last_t_ == 0) {
    last_t_ = ros::Time::now().toSec();
  }

  const double t_now =  ros::Time::now().toSec();
  const double dt = t_now - last_t_;
  last_t_ = t_now;

  // Track the distance and time in the current trajectory.
  current_traj_duration_ += dt;
  current_traj_length_ += dx;

  if (use_position_gains_)
  {
    position_cmd_.kx[0] = kx_[0], position_cmd_.kx[1] = kx_[1], position_cmd_.kx[2] = kx_[2];

    position_cmd_.position.x = position_cmd_.position.x + dt * position_cmd_.velocity.x;
    position_cmd_.position.y = position_cmd_.position.y + dt * position_cmd_.velocity.y;
    position_cmd_.position.z = position_cmd_.position.z + dt * position_cmd_.velocity.z;
  }
  else
  {
    position_cmd_.kx[0] = 0, position_cmd_.kx[1] = 0, position_cmd_.kx[2] = 0;

    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
  }
  position_cmd_.yaw = position_cmd_.yaw + dt * position_cmd_.yaw_dot;

  position_cmd_.header.stamp = msg->header.stamp;
  position_cmd_.header.frame_id = msg->header.frame_id;

  // Send feedback;
  std_trackers::VelocityTrackerFeedback feedback;
  feedback.duration = current_traj_duration_;
  tracker_server_->publishFeedback(feedback);

  return quadrotor_msgs::PositionCommand::ConstPtr(new quadrotor_msgs::PositionCommand(position_cmd_));
}


void VelocityTrackerAction::goal_callback() {

  // If another goal is already active, cancel that goal
  // and track this one instead.
  if (tracker_server_->isActive()) {
    ROS_INFO("VelocityTrackerAction goal (%2.2f, %2.2f, %2.2f, %2.2f) aborted.", position_cmd_.position.x, position_cmd_.position.y, position_cmd_.position.z, position_cmd_.yaw);
    tracker_server_->setAborted();
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  current_traj_duration_ = 0.0;
  current_traj_length_ = 0.0;

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if (tracker_server_->isPreemptRequested()) {
    ROS_INFO("VelocityTrackerAction going to goal (%2.2f, %2.2f, %2.2f, %2.2f) preempted.", msg->x, msg->y, msg->z, msg->yaw);
    tracker_server_->setPreempted();
    return;
  }

  // Otherwise, populate the current goal with information from the msg.
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;

  use_position_gains_ = msg->use_position_gains;
}

void VelocityTrackerAction::preempt_callback() {
  if (tracker_server_->isActive()) {
    ROS_INFO("VelocityTrackerAction going to goal (%2.2f, %2.2f, %2.2f, %2.2f) aborted.", position_cmd_.position.x, position_cmd_.position.y, position_cmd_.position.z, position_cmd_.yaw);
    tracker_server_->setAborted();
  }
  else {
    ROS_INFO("VelocityTrackerAction going to goal (%2.2f, %2.2f, %2.2f, %2.2f) preempted.", position_cmd_.position.x, position_cmd_.position.y, position_cmd_.position.z, position_cmd_.yaw);
    tracker_server_->setPreempted();
  }

  // TODO: How much overshoot will this cause at high velocities?
  position_cmd_.velocity.x = 0.0;
  position_cmd_.velocity.y = 0.0;
  position_cmd_.velocity.z = 0.0;
  position_cmd_.yaw_dot = 0.0;
  
  use_position_gains_ = true;
}

/*void VelocityTrackerAction::velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;

  use_position_gains_ = false;
}

void VelocityTrackerAction::position_velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;

  use_position_gains_ = true;
}*/

/*const quadrotor_msgs::TrackerStatus::Ptr VelocityTrackerAction::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}*/

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(VelocityTrackerAction, trackers_manager::Tracker)
