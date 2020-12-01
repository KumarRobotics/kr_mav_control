/* Sarah Tang
 * Trajectory tracker to track an ellipse of given height/width and period.
 * x(t) = [Ax*cos(2*pi*t/T); Ay*sin(2*pi*t/T); 0] + init_pos.
 * Yaw is constant. */

#include <actionlib/server/simple_action_server.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/CircleTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <cmath>

class CircleTracker : public kr_trackers_manager::Tracker
{
 public:
  CircleTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate();

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);

  uint8_t status() const;

 private:
  void goal_callback();
  void preempt_callback();

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::CircleTrackerAction> ServerType;
  // Action server that takes a trajectory.
  // Must be a pointer, because plugin does not support a constructor
  // with inputs, but an action server must be initialized with a Nodehandle.
  std::shared_ptr<ServerType> tracker_server_;

  // Is the tracker active?
  bool active_;
  // Do we have odometry?
  bool have_odom_;

  // Time trajectory started.
  ros::Time traj_start_time_;
  // traj_started_ indicates whether to reset start_time_.
  bool traj_started_;

  // Only track for position.
  // Distance traveled to get to last goal.
  float current_traj_length_;

  // Coefficients are ordered such that omega(t) = traj_coeffs_[i]*t^i.
  std::vector<float> omega_coeffs_;
  float Ax_, Ay_, omega_des_;

  // traj_completed_ indicates whether the trajectory is completed.
  bool traj_completed_;
  // Total time we want to track the trajectory for.
  float traj_duration_;
  // Time it takes to ramp up to the desired angular velocity.
  // ramp_dist_ is the theta at the end of the ramp phase.
  float ramp_time_, ramp_dist_;
  // Time that robot should stop circling (includes ramp up time).
  // circle_dist_ is the theta at the end of both phases.
  float circle_time_, circle_dist_;
  // Average angular acceleration for ramp up.
  float alpha_des_;

  // This is set if traj_completed_ = true;
  // Save the final position so we don't have to keep evaluating.
  // This holds the final position of the trajectory itself:
  // if use_offset_ = true, then the robot will evaluate position to
  // pos = final_pos_ + offset_pos_.
  Eigen::Vector3f final_pos_;

  // Offset to apply to the trajectory position.
  Eigen::Vector3f offset_pos_;

  // Record the current state.
  Eigen::Vector3f current_pos_;
  float current_yaw_;
  float constant_yaw_;

  // Pubish trigger for start and goal.
  ros::Publisher pub_start_;
  ros::Publisher pub_end_;
};

CircleTracker::CircleTracker(void)
    : active_(false),
      have_odom_(false),
      traj_started_(false),
      current_traj_length_(0.0),
      Ax_(0.0),
      Ay_(0.0),
      omega_des_(0.0),
      traj_completed_(false),
      traj_duration_(-1.0),
      ramp_time_(-1.0),
      circle_time_(-1.0)
{
}

void CircleTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "circle_tracker");

  priv_nh.param("alpha_des", alpha_des_, static_cast<float>(M_PI / 20.0));

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "CircleTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&CircleTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&CircleTracker::preempt_callback, this));

  tracker_server_->start();

  pub_start_ = priv_nh.advertise<std_msgs::Empty>("traj_start", 10);
  pub_end_ = priv_nh.advertise<std_msgs::Empty>("traj_end", 10);
}

bool CircleTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  if(!have_odom_)
  {
    ROS_WARN("CircleTracker::Activate: could not activate because no odom recieved - not activating.");
    active_ = false;
    return active_;
  }

  if(!tracker_server_->isActive())
  {
    ROS_WARN("CircleTracker::Activate: server has no active goal - not activating.");
    active_ = false;
    return active_;
  }

  if(omega_coeffs_.empty())
  {
    ROS_WARN("CircleTracker::Activate: internal trajectory not initialized - not activating.");
    active_ = false;
    return active_;
  }

  active_ = true;
  traj_started_ = false;
  current_traj_length_ = 0.0;
  traj_completed_ = false;

  // Calculate the offset needed so that
  // x(0) + offset = current_pos_.
  // Here, x(0) is [Ax 0 0].
  offset_pos_ = current_pos_ - Eigen::Vector3f(Ax_, 0.0, 0.0);
  constant_yaw_ = current_yaw_;
  //   std::cout << " RELATIVE ACTIVATE CALCULATED AS " << offset_pos_(0) << " " << offset_pos_(1) << " " <<
  //   offset_pos_(2) << " " << offset_yaw_ << "\n";

  return active_;
}

void CircleTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("CircleTracker::Deactivate: deactivated tracker while still tracking position trajectory.");

    kr_tracker_msgs::CircleTrackerResult result;
    result.duration = std::max(0.0f, static_cast<float>((ros::Time::now() - traj_start_time_).toSec()));
    result.length = current_traj_length_;
    tracker_server_->setAborted(result);
  }

  active_ = false;
  have_odom_ = false;
  traj_started_ = false;
  current_traj_length_ = 0.0;
  traj_completed_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr CircleTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Record distance between last position and current.
  const float dx =
      Eigen::Vector3f((current_pos_(0) - msg->pose.pose.position.x), (current_pos_(1) - msg->pose.pose.position.y),
                      (current_pos_(2) - msg->pose.pose.position.z))
          .norm();

  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;
  current_yaw_ = tf::getYaw(msg->pose.pose.orientation);

  have_odom_ = true;

  if(!active_)
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  current_traj_length_ += dx;

  const ros::Time t_now = ros::Time::now();
  kr_mav_msgs::PositionCommand::Ptr cmd(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  if(!traj_started_)
  {
    // Start the trajectory.
    traj_start_time_ = ros::Time::now();
    traj_started_ = true;
    traj_completed_ = false;

    current_traj_length_ = 0.0;
  }

  const float traj_time = std::max(0.0f, static_cast<float>((ros::Time::now() - traj_start_time_).toSec()));
  constexpr float kEps = 1e-6;

  // Process position trajectory.
  if(traj_completed_)
  {
    cmd->position.x = final_pos_(0);
    cmd->position.y = final_pos_(1);
    cmd->position.z = final_pos_(2);

    cmd->yaw = constant_yaw_;

    cmd->velocity.x = 0.0;
    cmd->velocity.y = 0.0;
    cmd->velocity.z = 0.0;
    cmd->acceleration.x = 0.0;
    cmd->acceleration.y = 0.0;
    cmd->acceleration.z = 0.0;
    cmd->jerk.x = 0.0;
    cmd->jerk.y = 0.0;
    cmd->jerk.z = 0.0;

    cmd->yaw_dot = 0.0;
  }
  else
  {
    float theta, theta_dot, theta_ddot, theta_dddot;

    // theta = omega_des_ * traj_time;
    // theta_dot = omega_des_;
    // theta_ddot = 0.0;
    // theta_dddot = 0.0;
    // theta_d4dot = 0.0;
    // theta_d5dot = 0.0;
    // theta_d6dot = 0.0;

    // Ramping up the angular velocity.
    if(traj_time < ramp_time_)
    {
      // ROS_INFO("Circle Tracker: Accelerating...");

      /*      theta = 0.5 * alpha_des_ * traj_time_ * traj_time_;
            theta_dot = alpha_des_ * traj_time_;
            theta_ddot = alpha_des_;
            theta_dddot = 0.0;*/
      const float dT = traj_time / ramp_time_;
      // std::cout << " ramp time i s" << ramp_time_ << " and dt is " << dT << "\n";
      const float dT2 = dT * dT, dT3 = dT2 * dT, dT4 = dT3 * dT, dT5 = dT4 * dT, dT6 = dT5 * dT;
      theta = omega_coeffs_.at(0) * dT + 0.5 * omega_coeffs_.at(1) * dT2 + 1.0 / 3.0 * omega_coeffs_.at(2) * dT3 +
              0.25 * omega_coeffs_.at(3) * dT4 + 1.0 / 5.0 * omega_coeffs_.at(4) * dT5 +
              1.0 / 6.0 * omega_coeffs_.at(5) * dT6;
      theta_dot = omega_coeffs_.at(0) + omega_coeffs_.at(1) * dT + omega_coeffs_.at(2) * dT2 +
                  omega_coeffs_.at(3) * dT3 + omega_coeffs_.at(4) * dT4 + omega_coeffs_.at(5) * dT5;
      theta_dot /= ramp_time_;
      theta_ddot = omega_coeffs_.at(1) + 2.0 * omega_coeffs_.at(2) * dT + 3.0 * omega_coeffs_.at(3) * dT2 +
                   4.0 * omega_coeffs_.at(4) * dT3 + 5.0 * omega_coeffs_.at(5) * dT4;
      const float ramp_time_2 = ramp_time_ * ramp_time_;
      theta_ddot /= ramp_time_2;
      theta_dddot = 2.0 * omega_coeffs_.at(2) + 6.0 * omega_coeffs_.at(3) * dT + 12.0 * omega_coeffs_.at(4) * dT2 +
                    20.0 * omega_coeffs_.at(5) * dT3;
      const float ramp_time_3 = ramp_time_2 * ramp_time_;
      theta_dddot /= (ramp_time_3);
    }
    // Constant segment.
    else if(traj_time < circle_time_)
    {
      // ROS_INFO("Circle Tracker: Coasting...");
      std_msgs::Empty empty_msg;
      pub_start_.publish(empty_msg);

      // Calculate time in this phase.
      const float dT = traj_time - ramp_time_;

      theta = omega_des_ * dT + ramp_dist_;
      theta_dot = omega_des_;
      theta_ddot = 0.0;
      theta_dddot = 0.0;
    }
    // Ramping down the angular velocity.
    else
    {
      // const float dT = traj_time - circle_time_;
      //       theta = omega_des_ * dT + circle_dist_;
      // theta_dot = omega_des_;
      // theta_ddot = 0.0;
      // theta_dddot = 0.0;
      // theta_d4dot = 0.0;
      // theta_d5dot = 0.0;
      // theta_d6dot = 0.0;

      // ROS_INFO("Circle Tracker: Decelerating...");

      // Publish message indicating trajectory end.
      std_msgs::Empty empty_msg;
      pub_end_.publish(empty_msg);

      // const float dT = traj_time - circle_time_;
      // theta = -0.5 * alpha_des_ * dT * dT + omega_des_ * dT + circle_dist_;
      // theta_dot = -alpha_des_ * dT + omega_des_;
      // theta_ddot = -alpha_des_;
      // theta_dddot = 0.0;

      const float dT = 1.0 - (traj_time - circle_time_) / ramp_time_;
      const float dT2 = dT * dT, dT3 = dT2 * dT, dT4 = dT3 * dT, dT5 = dT4 * dT, dT6 = dT5 * dT;
      theta = omega_coeffs_.at(0) * dT + 0.5 * omega_coeffs_.at(1) * dT2 + 1.0 / 3.0 * omega_coeffs_.at(2) * dT3 +
              0.25 * omega_coeffs_.at(3) * dT4 + 1.0 / 5.0 * omega_coeffs_.at(4) * dT5 +
              1.0 / 6.0 * omega_coeffs_.at(5) * dT6;
      theta = circle_dist_ + ramp_dist_ - theta;
      theta_dot = omega_coeffs_.at(0) + omega_coeffs_.at(1) * dT + omega_coeffs_.at(2) * dT2 +
                  omega_coeffs_.at(3) * dT3 + omega_coeffs_.at(4) * dT4 + omega_coeffs_.at(5) * dT5;
      theta_dot /= ramp_time_;
      theta_ddot = omega_coeffs_.at(1) + 2.0 * omega_coeffs_.at(2) * dT + 3.0 * omega_coeffs_.at(3) * dT2 +
                   4.0 * omega_coeffs_.at(4) * dT3 + 5.0 * omega_coeffs_.at(5) * dT4;
      const float ramp_time_2 = ramp_time_ * ramp_time_;
      theta_ddot /= ramp_time_2;
      theta_dddot = 2.0 * omega_coeffs_.at(2) + 6.0 * omega_coeffs_.at(3) * dT + 12.0 * omega_coeffs_.at(4) * dT2 +
                    20.0 * omega_coeffs_.at(5) * dT3;
      const float ramp_time_3 = ramp_time_2 * ramp_time_;
      theta_dddot /= (ramp_time_3);
    }

    const float sin_t = sin(theta);
    const float cos_t = cos(theta);

    const float pos_x = Ax_ * cos_t;
    const float pos_y = Ay_ * sin_t;
    const float pos_z = 0.0;

    const float vel_x = Ax_ * (-sin_t * theta_dot);
    const float vel_y = Ay_ * (cos_t * theta_dot);
    const float vel_z = 0.0;

    const float theta_dot_2 = theta_dot * theta_dot;
    const float acc_x = Ax_ * (-cos_t * theta_dot_2 - sin_t * theta_ddot);
    const float acc_y = Ay_ * (-sin_t * theta_dot_2 + cos_t * theta_ddot);
    const float acc_z = 0.0;

    const float theta_dot_3 = theta_dot_2 * theta_dot;
    const float jerk_x = Ax_ * (sin_t * theta_dot_3 - 3.0 * cos_t * theta_dot * theta_ddot - sin_t * theta_dddot);
    const float jerk_y = Ay_ * (-cos_t * theta_dot_3 - 3.0 * sin_t * theta_dot * theta_ddot + cos_t * theta_dddot);
    const float jerk_z = 0.0;

    // Note, these values do NOT yet include the offset.
    cmd->position.x = pos_x;
    cmd->position.y = pos_y;
    cmd->position.z = pos_z;
    cmd->yaw = constant_yaw_;

    cmd->velocity.x = vel_x;
    cmd->velocity.y = vel_y;
    cmd->velocity.z = vel_z;
    cmd->yaw_dot = 0.0;

    cmd->acceleration.x = acc_x;
    cmd->acceleration.y = acc_y;
    cmd->acceleration.z = acc_z;

    cmd->jerk.x = jerk_x;
    cmd->jerk.y = jerk_y;
    cmd->jerk.z = jerk_z;

    // Check if we completed the trajectory.
    if(traj_time >= traj_duration_ - kEps)
    {
      traj_completed_ = true;
      final_pos_ = Eigen::Vector3f(pos_x, pos_y, pos_z);
    }
  }

  cmd->position.x += offset_pos_(0);
  cmd->position.y += offset_pos_(1);
  cmd->position.z += offset_pos_(2);

  // If trajectory is completed, indicate this in the action.
  if(tracker_server_->isActive() && traj_completed_)
  {
    kr_tracker_msgs::CircleTrackerResult result;
    result.duration = traj_time;
    result.length = current_traj_length_;
    tracker_server_->setSucceeded(result);
    current_traj_length_ = 0.0;
  }
  // Otherwise, send feedback command.
  else
  {
    kr_tracker_msgs::CircleTrackerFeedback feedback;
    feedback.duration = traj_time;
    tracker_server_->publishFeedback(feedback);
  }

  return cmd;
}

void CircleTracker::goal_callback()
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("CircleTracker trajectory aborted because new goal recieved.");
    kr_tracker_msgs::CircleTrackerResult result;
    result.duration = std::max(0.0f, static_cast<float>((ros::Time::now() - traj_start_time_).toSec()));
    result.length = current_traj_length_;

    tracker_server_->setAborted(result);
  }

  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("CircleTracker trajectory preempted immediately after it was recieved.");
    kr_tracker_msgs::CircleTrackerResult result;
    result.duration = 0.0;
    result.length = 0.0;
    tracker_server_->setPreempted(result);
    return;
  }

  // Calculate the trajectory.
  omega_des_ = 2.0 * M_PI / msg->T;
  Ax_ = msg->Ax;
  Ay_ = msg->Ay;
  // Calculate the ramp time.
  const float dT = omega_des_ / alpha_des_;

  Eigen::MatrixXf Ainv(6, 6);
  Ainv << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, -10.0, -6.0, -1.5,
      10.0, -4.0, 0.5, 15.0, 8.0, 1.5, -15.0, 7.0, -1.0, -6.0, -3.0, -0.5, 6.0, -3.0, 0.5;
  Eigen::VectorXf b(6);
  b << 0.0, 0.0, 0.0, omega_des_ * dT, 0.0, 0.0;
  // std::cout << "A_inv" << Ainv(3, 0) << " " << Ainv(3, 1) << " " << Ainv(3, 2) << " " << Ainv(3, 3) << " " << Ainv(3,
  // 4) << " " << Ainv(3, 5) << " and b " << b(3) << "\n";
  Eigen::VectorXf omega_coeffs_vec = Ainv * b;
  // std::cout << " rowwise multiply " << Ainv.row(3) * b << "\n";
  omega_coeffs_.resize(6);
  // Track the angle traveled when ramping up.
  // We have coefficients for omega(t).
  // We need to evaluate theta(t) = int_0^t omega(t) at 1.
  ramp_dist_ = 0.0;
  // Scale the trajectory back.
  // const std::vector<float> T_powers = {1, dT, dT * dT, dT*dT * dT, dT*dT*dT * dT, dT*dT*dT*dT * dT};
  for(int ii = 0; ii < 6; ++ii)
  {
    //  std::cout << " the condition here is " << b(ii) << "\n";
    omega_coeffs_.at(ii) = omega_coeffs_vec(ii);  // omega_coeffs_vec(ii) / T_powers.at(ii);
    // std::cout << " coefficient " << ii << " is " << omega_coeffs_vec(ii) << "\n";
    ramp_dist_ += omega_coeffs_.at(ii) * 1.0 / static_cast<float>(ii + 1);
  }

  // Calculate the offset needed so that
  // x(0) + offset = current_pos_.
  offset_pos_ = current_pos_ - Eigen::Vector3f(Ax_, 0.0, 0.0);
  //  std::cout << " RELATIVE CALCULATED AS " << offset_pos_(0) << " " << offset_pos_(1) << " " << offset_pos_(2) << " "
  //  << offset_yaw_ << "\n";

  ramp_time_ = dT;
  circle_time_ = msg->duration + ramp_time_;
  traj_duration_ = 2.0 * ramp_time_ + msg->duration;

  // Calculate the distances needed.
  circle_dist_ = ramp_dist_ + omega_des_ * msg->duration;

  // std::cout << " CIRCLE DESIGNED: ramp time " << ramp_time_ << " and circle time " << circle_time_ << " and total
  // duration " << traj_duration_ << " the ramp dist " << ramp_dist_ * 180.0 / M_PI << " and circle dist " <<
  // circle_dist_ * 180.0 / M_PI << " to get to angular velocity " << omega_des_ << "\n";

  traj_started_ = false;
  traj_completed_ = false;
}

void CircleTracker::preempt_callback()
{
  // Send a message reporting about the trajectory that was executed.
  kr_tracker_msgs::CircleTrackerResult result;
  result.duration = std::max(0.0f, static_cast<float>((ros::Time::now() - traj_start_time_).toSec()));
  result.length = current_traj_length_;

  if(tracker_server_->isActive())
  {
    ROS_INFO("CircleTracker trajectory aborted by preempt command.");
    tracker_server_->setAborted(result);
  }
  else
  {
    ROS_INFO("CircleTracker trajectory preempted by preempt command.");
    tracker_server_->setPreempted(result);
  }

  traj_started_ = true;
  traj_completed_ = true;
  traj_duration_ = 0.0;
  current_traj_length_ = 0.0;

  final_pos_ = current_pos_;
}

uint8_t CircleTracker::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CircleTracker, kr_trackers_manager::Tracker);
