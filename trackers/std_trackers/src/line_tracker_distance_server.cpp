#include <memory>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>

#include <initial_conditions.h>
#include <trackers_manager/Tracker.h>
#include <std_trackers/LineTrackerAction.h>
#include <quadrotor_msgs/PositionCommand.h>

class LineTrackerDistanceAction : public trackers_manager::Tracker {

public:
  LineTrackerDistanceAction(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);

private:
  void goal_callback();

  void preempt_callback();

  typedef actionlib::SimpleActionServer<std_trackers::LineTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::shared_ptr<ServerType> tracker_server_;

  // True if odometry is being recieved and current_pos_, current_yaw_
  // are being updated.
  bool have_odom_;

  // Indicates whether this tracker is active -
  // if yes, position commands will be calculated.
  // If not, only current_pos_ will be updated at each step.
  bool active_;

  // True if goal is set.
  bool goal_set_;

  // True if goal is reached.
  bool goal_reached_;

  // Trajectory properties.
  double default_v_des_, default_a_des_, epsilon_;
  double kx_[3], kv_[3];
  // Initial conditions for the trajectory.
  InitialConditions ICs_;
  // Start and goal of the trajectory.
  Eigen::Vector3f start_pos_;
  float start_yaw_;

  // Track the current position and yaw.
  Eigen::Vector3f current_pos_;
  float current_yaw_;

  // Information recieved from the goal.
  // This will be recieved from the action server, however, if preempted,
  // it will be the current position, indicating the robot should
  // hover in place.
  Eigen::Vector3f goal_pos_;
  float v_des_, a_des_;

  // Time taken to get to the goal.
  float current_traj_duration_;
  // Distance traveled to get to last goal.
  float current_traj_length_;

}; // class LineTrackerDistance


LineTrackerDistanceAction::LineTrackerDistanceAction(void) :
  have_odom_(false),
  active_(false),
  goal_set_(false),
  goal_reached_(true) {}

void LineTrackerDistanceAction::Initialize(const ros::NodeHandle &nh) {
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  nh.param("default_v_des", default_v_des_, 0.5);
  nh.param("default_a_des", default_a_des_, 0.5);

  ros::NodeHandle priv_nh(nh, "line_tracker_distance");
  priv_nh.param("epsilon", epsilon_, 0.1);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LineTrackerAction", false));
  tracker_server_->registerGoalCallback(boost::bind(&LineTrackerDistanceAction::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LineTrackerDistanceAction::preempt_callback, this));

  tracker_server_->start();
}

bool LineTrackerDistanceAction::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {
  // Only allow activation if a goal has been accepted by the action server.
//  if (tracker_server_->isActive() && have_odom_) {
  if (goal_set_ && have_odom_) {
    // Check that the action server has a goal.
    if (!tracker_server_->isActive()) {
      ROS_WARN("LineTrackerDistanceAction::Activate: goal_set_ is true but action server has no active goal - not activating.");
      // std::cout << " WRONG: goal set but server inactive\n";
      active_ = false;
      return active_;
    }
    /*   else {
         std::cout << " CORRECT: goal set and server active\n";
       }*/

    // Set start and start_yaw here so that even if the goal was sent at a
    // different position, we still use the current position as start.
    start_pos_ = current_pos_;
    start_yaw_ = current_yaw_;

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    active_ = true;
  }
  /*
    if (!goal_set_) {
      if (tracker_server_->isActive()) {
        std::cout << " WRONG: server is active but goal is set\n";
      }
      else {
        std::cout << " CORRECT: server inactive nad goal not set\n";
      }
    }*/

  return active_;
}

void LineTrackerDistanceAction::Deactivate(void) {
  // Don't deactivate if there is a goal still active.
  // We must explicitly cancel the request before switching.
  /*  if (goal_set_) {
      if (tracker_server_->isActive()) {
        std::cout << " CORRECT: goal set and tracker active\n";
      }
      else {
        std::cout << " WRONG: goal set and tracker inactive\n";
      }
    }*/
  if (tracker_server_->isActive()) {
    ROS_WARN("LineTrackerDistanceAction::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }

  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

const quadrotor_msgs::PositionCommand::ConstPtr LineTrackerDistanceAction::update(const nav_msgs::Odometry::ConstPtr &msg) {
  // Record distance between last position and current.
  const float dx = Eigen::Vector3f((current_pos_(0) - msg->pose.pose.position.x), (current_pos_(1) - msg->pose.pose.position.y), (current_pos_(2) - msg->pose.pose.position.z)).norm();

  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;
  current_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  have_odom_ = true;
  ICs_.set_from_odom(msg);

  static ros::Time t_prev;
  const double dT = (msg->header.stamp - t_prev).toSec();
  t_prev = msg->header.stamp;

  if (!active_) {
    return quadrotor_msgs::PositionCommand::Ptr();
  }

  // Track the distance and time in the current trajectory.
  current_traj_duration_ += dT;
  current_traj_length_ += dx;

  quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // If inactive, send position of last goal.
//  if (tracker_server_->isActive()) {
  if (goal_reached_) {
    if (tracker_server_->isActive()) {
      ROS_ERROR("LineTrackerDistanceAction::update: Action server not completed.\n");
    }
    /*    if (tracker_server_->isActive()) {
          std::cout << " WRONG: server actived but goal reached\n";
        }
        else {
          std::cout << " CORRECT: server inactive and goal reached\n";
        }*/
    cmd->position.x = goal_pos_(0), cmd->position.y = goal_pos_(1), cmd->position.z = goal_pos_(2);
    cmd->velocity.x = 0, cmd->velocity.y = 0, cmd->velocity.z = 0;
    cmd->acceleration.x = 0, cmd->acceleration.y = 0, cmd->acceleration.z = 0;

    ICs_.set_from_cmd(cmd);
    return cmd;
  }

  const Eigen::Vector3f dir = (goal_pos_ - start_pos_).normalized();
  const float total_dist = (goal_pos_ - start_pos_).norm();
  const float d = (current_pos_ - start_pos_).dot(dir);
  const Eigen::Vector3f proj = start_pos_ + d * dir;

  const float v_max = std::min(std::sqrt(a_des_ * total_dist), v_des_);
  const float ramp_dist = v_max * v_max / (2 * a_des_);

  Eigen::Vector3f x(current_pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

  if ((current_pos_ - goal_pos_).norm() <= epsilon_) { // Reached goal

    // Send a success message and reset the length and duration variables.
    std_trackers::LineTrackerResult result;
    result.duration = current_traj_duration_;
    result.length = current_traj_length_;
    result.x = goal_pos_(0);
    result.y = goal_pos_(1);
    result.z = goal_pos_(2);
    result.yaw = start_yaw_;
    tracker_server_->setSucceeded(result);

    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;

    // Velocity and acceleration commands are therefore 0.
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_pos_;
    goal_reached_ = true;
  }
  else if (d > total_dist) { // Overshoot
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = goal_pos_;
  }
  else if (d >= (total_dist - ramp_dist) && d <= total_dist) { // Decelerate
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des_ * dir;
    v = std::sqrt(2.0 * a_des_ * (total_dist - d)) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if (d > ramp_dist && d < total_dist - ramp_dist) { // Constant velocity
    ROS_DEBUG_THROTTLE(1, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max * dir;
    x = proj + v * dT;
  }
  else if (d >= 0 && d <= ramp_dist) { // Accelerate
    ROS_DEBUG_THROTTLE(1, "Accelerate");
    a = a_des_ * dir;
    v = std::sqrt(2.0 * a_des_ * d) * dir;
    x = proj + v * dT + 0.5 * a * dT * dT;
  }
  else if (d < 0.0) {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = start_pos_ + 0.5 * a * dT * dT;
  }
  cmd->position.x = x(0), cmd->position.y = x(1), cmd->position.z = x(2);
  cmd->velocity.x = v(0), cmd->velocity.y = v(1), cmd->velocity.z = v(2);
  cmd->acceleration.x = a(0), cmd->acceleration.y = a(1), cmd->acceleration.z = a(2);
  ICs_.set_from_cmd(cmd);


  // If not at goal, send the command as feedback.
// if (tracker_server_->isActive()) {
  if (!goal_reached_) {
    /*    if (tracker_server_->isActive()) {
          std::cout << " CORRECT: goal not reached and server active\n";
        }
        else {
          std::cout << " WRONG: goal not reached but server inactive\n";
        }*/
    std_trackers::LineTrackerFeedback feedback;
    feedback.distance_from_goal = (current_pos_ - goal_pos_).norm();
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void LineTrackerDistanceAction::goal_callback() {

  // If another goal is already active, cancel that goal
  // and track this one instead.
  if (tracker_server_->isActive()) {
    ROS_INFO("LineTrackerDistanceAction goal (%2.2f, %2.2f, %2.2f) aborted.", goal_pos_(0), goal_pos_(1), goal_pos_(2));
    tracker_server_->setAborted();
    /*   if (goal_set_) {
         std::cout << " CORRECT: goal st and server active\n";
       }
       else {
         std::cout << " WRONG: goal not set but server active\n";
       }*/
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  current_traj_duration_ = 0.0;
  current_traj_length_ = 0.0;

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if (tracker_server_->isPreemptRequested()) {
    ROS_INFO("LineTrackerDistanceAction going to goal (%2.2f, %2.2f, %2.2f) preempted.", msg->x, msg->y, msg->z);
    tracker_server_->setPreempted();
    return;
  }

  // Otherwise, populate the current goal with information from the msg.
  goal_pos_(0) = msg->x;
  goal_pos_(1) = msg->y;
  goal_pos_(2) = msg->z;

  if (msg->relative) {
    goal_pos_ += ICs_.pos();
  }

  v_des_ = (msg->v_des > 0.0) ? msg->v_des : default_v_des_;
  a_des_ = (msg->a_des > 0.0) ? msg->a_des : default_a_des_;

  start_pos_ = current_pos_;
  start_yaw_ = current_yaw_;

  goal_set_ = true;
  goal_reached_ = false;
}

void LineTrackerDistanceAction::preempt_callback() {
  if (tracker_server_->isActive()) {
    ROS_INFO("LineTrackerDistanceAction going to goal (%2.2f, %2.2f, %2.2f) aborted.", goal_pos_(0), goal_pos_(1), goal_pos_(2));
    tracker_server_->setAborted();
  }
  else {
    ROS_INFO("LineTrackerDistanceAction going to goal (%2.2f, %2.2f, %2.2f) preempted.", goal_pos_(0), goal_pos_(1), goal_pos_(2));
    tracker_server_->setPreempted();
  }

  // TODO: How much overshoot will this cause at high velocities?
  goal_pos_ = current_pos_;

  goal_set_ = false;
  goal_reached_ = true;
}

/*const quadrotor_msgs::TrackerStatus::Ptr LineTrackerDistanceAction::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);

  msg->status = goal_reached_ ? (uint8_t)
    quadrotor_msgs::TrackerStatus::SUCCEEDED : quadrotor_msgs::TrackerStatus::ACTIVE;

  return msg;
}
*/
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LineTrackerDistanceAction, trackers_manager::Tracker);
