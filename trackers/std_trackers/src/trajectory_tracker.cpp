#include <iostream>
#include <memory>

#include <ros/ros.h>
//#include <tf/transform_datatypes.h>

#include <actionlib/server/simple_action_server.h>
#include <initial_conditions.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_trackers/TrajectoryTrackerAction.h>
#include <trackers_manager/Tracker.h>
#include <trackers_manager/TrackerStatus.h>

#include "traj_gen.h"

class TrajectoryTracker : public trackers_manager::Tracker
{
 public:
  TrajectoryTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);

  uint8_t status() const;

 private:
  void goal_callback();

  void preempt_callback();

  typedef actionlib::SimpleActionServer<std_trackers::TrajectoryTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::unique_ptr<ServerType> tracker_server_;

  std::unique_ptr<TrajectoryGenerator> traj_gen_;

  Eigen::Vector3f current_pos_;

  bool pos_set_, goal_set_, goal_reached_;
  float max_v_des_, max_a_des_;
  bool active_;

  InitialConditions ICs_;
  ros::Time traj_start_;
  float traj_total_time_;

  std_trackers::TrajectoryTrackerGoal goal_;

  double kx_[3], kv_[3];

  float current_traj_length_;
};

TrajectoryTracker::TrajectoryTracker(void)
    : pos_set_(false),
      goal_set_(false),
      goal_reached_(true),
      active_(false)
{
}

void TrajectoryTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "trajectory_tracker");

  priv_nh.param("max_vel_des", max_v_des_, 1.0f);
  priv_nh.param("max_acc_des", max_a_des_, 1.0f);

  int continuous_derivative_order, derivative_order_to_minimize;
  priv_nh.param("continuous_derivative_order", continuous_derivative_order, 2);
  priv_nh.param("derivative_order_to_minimize", derivative_order_to_minimize, 3);
  continuous_derivative_order = std::max(0, continuous_derivative_order);
  derivative_order_to_minimize = std::max(1, derivative_order_to_minimize);

  // Trajectory Generator
  traj_gen_.reset(new TrajectoryGenerator(continuous_derivative_order, derivative_order_to_minimize));

  // Set up the action server.
  tracker_server_.reset(new ServerType(priv_nh, "TrajectoryTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&TrajectoryTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&TrajectoryTracker::preempt_callback, this));

  tracker_server_->start();
}

bool TrajectoryTracker::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    if(!tracker_server_->isActive())
    { // check timed
      ROS_WARN("TrajectoryTracker::Activate: goal_set_ is true but action server has no active goal - not activating.");
      active_ = false;
      return false;
    }
    active_ = true;

    current_traj_length_ = 0.0;
  }
  return active_;
}

void TrajectoryTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("TrajectoryTracker::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

quadrotor_msgs::PositionCommand::ConstPtr TrajectoryTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  if(!active_)
  {
    return quadrotor_msgs::PositionCommand::Ptr();
  }

  const ros::Time t_now = ros::Time::now();

  // Record distance between last position and current.
  const float dx =
      Eigen::Vector3f(current_pos_(0) - msg->pose.pose.position.x, current_pos_(1) - msg->pose.pose.position.y,
                      current_pos_(2) - msg->pose.pose.position.z)
          .norm();

  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;

  current_traj_length_ += dx;

  auto cmd = boost::make_shared<quadrotor_msgs::PositionCommand>();
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if(goal_set_)
  {
    traj_start_ = t_now;
    traj_gen_->setInitialConditions(ICs_.pos(), {ICs_.vel(), ICs_.acc(), ICs_.jrk()});
    for(const auto &p : goal_.waypoints)
    {
      traj_gen_->addWaypoint(Eigen::Vector3f(p.position.x, p.position.y, p.position.z));
    }
    std::vector<float> waypoint_times;
    waypoint_times.reserve(goal_.waypoints.size());
    if(goal_.waypoint_times.size() == 0)
    {
      waypoint_times = traj_gen_->computeTimesTrapezoidSpeed(max_v_des_ / 2, max_a_des_ / 2);
    }
    else
    {
      for(const auto &t :goal_.waypoint_times)
        waypoint_times.push_back(t);
    }

    traj_gen_->calculate(waypoint_times);
    float max_jerk_des = 100;
    traj_gen_->optimizeWaypointTimes(max_v_des_, max_a_des_, max_jerk_des);

    traj_total_time_ = traj_gen_->getTotalTime();

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    if(tracker_server_->isActive())
      ROS_ERROR("TrajectoryTracker::update: Action server not completed");

    cmd->position = goal_.waypoints.back().position;
    cmd->yaw = ICs_.yaw();
    cmd->yaw_dot = 0;
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  Eigen::Vector3f x(ICs_.pos()), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero()), j(Eigen::Vector3f::Zero());
  float yaw_des(ICs_.yaw()), yaw_dot_des(0);

  const float traj_time = (t_now - traj_start_).toSec();

  if(traj_time >= traj_total_time_) // Reached goal
  {
    // Send a success message and reset the length and duration variables.
    std_trackers::TrajectoryTrackerResult result;
    result.total_time = traj_time;
    result.total_distance_travelled = current_traj_length_;

    tracker_server_->setSucceeded(result);

    current_traj_length_ = 0.0;

    ROS_DEBUG_THROTTLE(1, "Reached goal");
    j = Eigen::Vector3f::Zero();
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    const auto last_waypoint_pos = goal_.waypoints.back().position;
    x = Eigen::Vector3f(last_waypoint_pos.x, last_waypoint_pos.y, last_waypoint_pos.z);
    yaw_des = ICs_.yaw();
    yaw_dot_des = 0;
    goal_reached_ = true;
  }
  else if(traj_time >= 0)
  {
    traj_gen_->getCommand(traj_time, x, v, a, j);
    yaw_des = ICs_.yaw();
    yaw_dot_des = 0;
  }
  else // (traj_time < 0) can happen with TrajectoryTrackerGoalTimed
    ROS_INFO_THROTTLE(1, "Trajectory hasn't started yet");

  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->yaw = yaw_des;
  cmd->yaw_dot = yaw_dot_des;
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  cmd->jerk.x = j(0), cmd->jerk.y = j(1), cmd->jerk.z = j(2);

  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    std_trackers::TrajectoryTrackerFeedback feedback;
    feedback.remaining_time = traj_total_time_ - traj_time;
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void TrajectoryTracker::goal_callback()
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("TrajectoryTracker goal aborted");
    tracker_server_->setAborted();
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  current_traj_length_ = 0.0;

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("TrajectoryTracker preempted");
    tracker_server_->setPreempted();
    return;
  }

  if(msg->waypoints.size() > 0 &&
     (msg->waypoint_times.size() == 0 || msg->waypoint_times.size() == msg->waypoints.size()))
  {
    goal_ = *msg;
    goal_set_ = true;
    goal_reached_ = false;
  }
  else
  {
    ROS_WARN("TrajectoryTracker: Invalid goal received! Ignoring");
  }
}

void TrajectoryTracker::preempt_callback()
{
  if(tracker_server_->isActive())
  {
    ROS_INFO("TrajectoryTracker aborted");
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("TrajectoryTracker preempted");
    tracker_server_->setPreempted();
  }

  goal_.waypoints.clear();
  goal_.waypoint_times.clear();

  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = ICs_.pos()(0);
  goal_pose.position.y = ICs_.pos()(1);
  goal_pose.position.z = ICs_.pos()(2);
  goal_pose.orientation.w = std::cos(ICs_.yaw() / 2);
  goal_pose.orientation.z = std::sin(ICs_.yaw() / 2);
  goal_.waypoints.push_back(goal_pose);

  goal_set_ = false;
  goal_reached_ = true;
}

uint8_t TrajectoryTracker::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(trackers_manager::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(trackers_manager::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrajectoryTracker, trackers_manager::Tracker);
