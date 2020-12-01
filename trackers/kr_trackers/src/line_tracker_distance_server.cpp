#include <actionlib/server/simple_action_server.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <memory>

class LineTrackerDistance : public kr_trackers_manager::Tracker
{
 public:
  LineTrackerDistance(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);

  uint8_t status() const;

 private:
  void goal_callback();

  void preempt_callback();

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::LineTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::shared_ptr<ServerType> tracker_server_;

  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  InitialConditions ICs_;
  Eigen::Vector3f start_, goal_, pos_;
  float yaw_, start_yaw_;

  // Time taken to get to the goal.
  float current_traj_duration_;
  // Distance traveled to get to last goal.
  float current_traj_length_;
};

LineTrackerDistance::LineTrackerDistance(void) : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false)
{
}

void LineTrackerDistance::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "line_tracker_distance");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.5);
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LineTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&LineTrackerDistance::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LineTrackerDistance::preempt_callback, this));

  tracker_server_->start();
}

bool LineTrackerDistance::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    // Check that the action server has a goal.
    if(!tracker_server_->isActive())
    {
      ROS_WARN("LineTrackerDistance::Activate: goal_set_ is true but action server has no active goal - not "
               "activating.");
      active_ = false;
      return active_;
    }

    // Set start and start_yaw here so that even if the goal was sent at a
    // different position, we still use the current position as start
    start_ = pos_;
    start_yaw_ = yaw_;

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    active_ = true;
  }
  return active_;
}

void LineTrackerDistance::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("LineTrackerDistance::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr LineTrackerDistance::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Record distance between last position and current.
  const float dx = Eigen::Vector3f((pos_(0) - msg->pose.pose.position.x), (pos_(1) - msg->pose.pose.position.y),
                                   (pos_(2) - msg->pose.pose.position.z))
                       .norm();

  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  static ros::Time t_prev = msg->header.stamp;
  const double dT = (msg->header.stamp - t_prev).toSec();
  t_prev = msg->header.stamp;

  if(!active_)
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  // Track the distance and time in the current trajectory.
  current_traj_duration_ += dT;
  current_traj_length_ += dx;

  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;

  if(goal_reached_)
  {
    if(tracker_server_->isActive())
    {
      ROS_ERROR("LineTrackerDistance::update: Action server not completed.\n");
    }

    cmd->position.x = goal_(0), cmd->position.y = goal_(1), cmd->position.z = goal_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_ - start_).normalized();
  const float total_dist = (goal_ - start_).norm();
  const float d = (pos_ - start_).dot(dir);
  const Eigen::Vector3f proj = start_ + d * dir;

  const float v_max = std::min(std::sqrt(a_des_ * total_dist), v_des_);
  const float ramp_dist = v_max * v_max / (2 * a_des_);

  Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

  if((pos_ - goal_).norm() <= epsilon_)  // Reached goal
  {
    // Send a success message and reset the length and duration variables.
    kr_tracker_msgs::LineTrackerResult result;
    result.duration = current_traj_duration_;
    result.length = current_traj_length_;
    result.x = goal_(0);
    result.y = goal_(1);
    result.z = goal_(2);
    result.yaw = start_yaw_;
    tracker_server_->setSucceeded(result);

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(d > total_dist)  // Overshoot
  {
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = goal_;
  }
  else if(d >= (total_dist - ramp_dist) && d <= total_dist)  // Decelerate
  {
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des_ * dir;
    v = std::sqrt(2 * a_des_ * (total_dist - d)) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if(d > ramp_dist && d < total_dist - ramp_dist)  // Constant velocity
  {
    ROS_DEBUG_THROTTLE(1, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max * dir;
    x = proj + v * dT;
  }
  else if(d >= 0 && d <= ramp_dist)  // Accelerate
  {
    ROS_DEBUG_THROTTLE(1, "Accelerate");
    a = a_des_ * dir;
    v = std::sqrt(2 * a_des_ * d) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if(d < 0)
  {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = start_ + 0.5 * a * dT * dT;
  }
  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  ICs_.set_from_cmd(cmd);

  if(!goal_reached_)
  {
    kr_tracker_msgs::LineTrackerFeedback feedback;
    feedback.distance_from_goal = (pos_ - goal_).norm();
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void LineTrackerDistance::goal_callback()
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("LineTrackerDistance goal (%2.2f, %2.2f, %2.2f) aborted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setAborted();
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  current_traj_duration_ = 0.0;
  current_traj_length_ = 0.0;

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) preempted.", msg->x, msg->y, msg->z);
    tracker_server_->setPreempted();
    return;
  }

  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  if(msg->relative)
    goal_ += ICs_.pos();

  if(msg->v_des > 0)
    v_des_ = msg->v_des;
  else
    v_des_ = default_v_des_;

  if(msg->a_des > 0)
    a_des_ = msg->a_des;
  else
    a_des_ = default_a_des_;

  start_ = pos_;
  start_yaw_ = yaw_;

  current_traj_length_ = 0.0;
  current_traj_duration_ = 0.0;

  goal_set_ = true;
  goal_reached_ = false;
}

void LineTrackerDistance::preempt_callback()
{
  if(tracker_server_->isActive())
  {
    ROS_INFO("LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) aborted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("LineTrackerDistance going to goal (%2.2f, %2.2f, %2.2f) preempted.", goal_(0), goal_(1), goal_(2));
    tracker_server_->setPreempted();
  }

  // TODO: How much overshoot will this cause at high velocities?
  goal_ = pos_;

  goal_set_ = false;
  goal_reached_ = true;
}

uint8_t LineTrackerDistance::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerDistance, kr_trackers_manager::Tracker);
