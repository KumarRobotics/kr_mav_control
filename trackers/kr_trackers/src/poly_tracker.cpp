#include "rclcpp_action/rclcpp_action.hpp"
#include "kr_mav_msgs/msg/position_command.hpp"
#include "kr_tracker_msgs/action/poly_tracker.hpp"
#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_trackers/initial_conditions.hpp"
#include "kr_trackers/Tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Eigen>
#include <traj_data.hpp>
#include <memory>
#include <mutex>

// traj data
struct TrajData
{
  /* info of generated traj */
  double traj_dur_ = 0, traj_yaw_dur_ = 0;
  rclcpp::Time start_time_;
  int dim_;


  traj_opt::Trajectory2D traj_2d_;
  traj_opt::Trajectory3D traj_3d_;
  traj_opt::Trajectory4D traj_with_yaw_;
  traj_opt::Trajectory1D traj_yaw_;
  bool has_solo_yaw_traj_ = false;

  traj_opt::DiscreteStates traj_discrete_;
};


class PolyTracker : public kr_trackers_manager::Tracker
{
 public:
  PolyTracker(void);

  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent);
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd);
  void Deactivate(void);

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg);
  uint8_t status();

 private:
  // action callbacks
  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const kr_tracker_msgs::action::PolyTracker::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr_tracker_msgs::action::PolyTracker>> goal_handle);
  void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr_tracker_msgs::action::PolyTracker>> goal_handle);

  kr_mav_msgs::msg::PositionCommand position_cmd_;

  /*** odom related ***/
  double cur_yaw_, last_yaw_ = 0.0, last_yawdot_ = 0.0;
  Eigen::Vector3d cur_pos_, last_pos_, last_goal_;
  bool have_last_goal_ = false;

  using PolyTrackerAction = kr_tracker_msgs::action::PolyTracker;
  using PolyTrackerGoalHandle = rclcpp_action::ServerGoalHandle<PolyTrackerAction>;
  rclcpp_action::Server<PolyTrackerAction>::SharedPtr tracker_server_;
  std::shared_ptr<PolyTrackerGoalHandle> current_goal_handle_;
  rclcpp::Logger logger_{rclcpp::get_logger("poly_tracker")};
  rclcpp::Clock::SharedPtr clock_;
  std::recursive_mutex mutex_;

  bool pos_set_, goal_set_, goal_reached_, active_;
  bool traj_set_ = false;
  bool yaw_set_  = false;

  std::shared_ptr<TrajData> current_trajectory_, next_trajectory_;

  /*** yaw set up ***/
  // intial rotation
  double init_final_yaw_, init_dyaw_, init_yaw_time_;
  rclcpp::Time time_last_;


  /*** parameters ***/
  double time_forward_ = 1.5;
  double max_dyaw_ = 0.5 * M_PI;
  double max_ddyaw_ = M_PI;

  std::pair<double, double> calculate_yaw(Eigen::Vector3d &dir, double dt);
  double range(double angle);

};

PolyTracker::PolyTracker(void) : pos_set_(false), goal_set_(false), goal_reached_(false), active_(false)
{
}

void PolyTracker::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  if(!node) return;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Set up the action server.
  tracker_server_ = rclcpp_action::create_server<PolyTrackerAction>(
    node,
    "~/poly_tracker/PolyTracker",
    std::bind(&PolyTracker::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PolyTracker::cancel_callback, this, std::placeholders::_1),
    std::bind(&PolyTracker::handle_accepted_callback, this, std::placeholders::_1)
  );

  current_trajectory_.reset(new TrajData);
  next_trajectory_.reset(new TrajData);
  RCLCPP_WARN(logger_, "PolyTracker initialized!");
}

bool PolyTracker::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  // Only allow activation if a goal has been set
  if(pos_set_)
  {
    if(!current_goal_handle_ || !current_goal_handle_->is_active())
    {
      RCLCPP_WARN(logger_, "TrajectoryTracker::Activate: goal_set_ is true but action server has no active goal - not activating.");
      active_ = false;
      return false;
    }
    active_ = true;
    RCLCPP_WARN(logger_, "TrajectoryTracker::Activate: !");
  }
  return active_;
}

void PolyTracker::Deactivate(void)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_WARN(logger_, "PolyTracker::Deactivate: deactivated tracker while still tracking the goal.");
    current_goal_handle_->abort(std::make_shared<PolyTrackerAction::Result>());
    current_goal_handle_.reset();
  }

  goal_set_ = false;
  active_ = false;
}

double PolyTracker::range(double angle)
{
  // range the angle into (-PI, PI]
  double psi = angle;
  while(psi > M_PI)
  {
    psi -= 2 * M_PI;
  }
  while(psi <= -M_PI)
  {
    psi += 2 * M_PI;
  }
  return psi;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr PolyTracker::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pos_set_ = true;

  cur_pos_(0) = msg->pose.pose.position.x;
  cur_pos_(1) = msg->pose.pose.position.y;
  cur_pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf2::getYaw(msg->pose.pose.orientation);

  rclcpp::Time time_now = clock_->now();

  if(!active_)
  {
    last_yaw_ = cur_yaw_;
    last_goal_ = cur_pos_;
    time_last_ = time_now;
    last_pos_ = cur_pos_;
  return std::make_shared<kr_mav_msgs::msg::PositionCommand>(position_cmd_);
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(last_yaw_, 0.0);

  Eigen::VectorXd wp, dwp, ddwp;  // only
  int cur_dim = 0;
  if (next_trajectory_ != NULL)
  {
    cur_dim = next_trajectory_->dim_;

  }
  else
  {
    cur_dim = current_trajectory_->dim_;
  }

  wp.resize(cur_dim);
  dwp.resize(cur_dim);
  ddwp.resize(cur_dim);

  // safety 
  if(have_last_goal_ && (cur_pos_ - last_goal_).norm() <= 0.3)
  {
    pos = last_goal_;
  }
  else
  {
    pos = last_pos_;
  }

  if(yaw_set_) // 1. rotate yaw mode
  {
    double dyaw = range(init_final_yaw_ - cur_yaw_);

    if(std::abs(dyaw) < 0.5 || init_yaw_time_ > 2.0)
    {
      yaw_set_ = false;
      // ROS_INFO(" yaw_set finished ");
      time_last_ = time_now;
      return std::make_shared<kr_mav_msgs::msg::PositionCommand>(position_cmd_);
    }

    double yaw_temp = cur_yaw_ + (time_now - time_last_).seconds() * init_dyaw_;
    double desired_yaw =
        init_final_yaw_ - cur_yaw_ >= 0 ? std::min(yaw_temp, init_final_yaw_) : std::max(yaw_temp, init_final_yaw_);

    yaw_yawdot.first = desired_yaw;
    yaw_yawdot.second = init_dyaw_;

    init_yaw_time_ += (time_now - time_last_).seconds();
  }
  else if(traj_set_)
  {

    if(next_trajectory_ != NULL && (time_now - next_trajectory_->start_time_).seconds() >= 0.0)
    {
      current_trajectory_ = next_trajectory_;
      next_trajectory_ = NULL;
    }

    double t_cur = (time_now - current_trajectory_->start_time_).seconds();

    if(t_cur < current_trajectory_->traj_dur_ && t_cur >= 0.0)
    {
      switch(current_trajectory_->dim_)
      {
        case 1:
        {

          Eigen::VectorXd cur_state = current_trajectory_->traj_discrete_.getState(t_cur);
          pos = cur_state.head(3);
          vel = cur_state.segment(3, 3);
          acc = cur_state.tail(3);

          /*** calculate yaw ***/
          Eigen::Vector3d dir = t_cur + time_forward_ <= current_trajectory_->traj_dur_ ? 
                                                          current_trajectory_->traj_discrete_.getNextPos(t_cur + time_forward_) - pos :
                                                          current_trajectory_->traj_discrete_.getNextPos(current_trajectory_->traj_dur_) - pos;
            yaw_yawdot = calculate_yaw(dir, (time_now - time_last_).seconds());

          break;
        }
        case 2:
        {
          wp  = current_trajectory_->traj_2d_.getPos(t_cur);
          dwp = current_trajectory_->traj_2d_.getVel(t_cur);
          pos.head(2) = wp;
          pos(2) = last_goal_(2);
          vel.head(2) = dwp;
          break;
        }
        case 3:
        {
          pos = current_trajectory_->traj_3d_.getPos(t_cur);
          vel = current_trajectory_->traj_3d_.getVel(t_cur);
          acc = current_trajectory_->traj_3d_.getAcc(t_cur);

          if(current_trajectory_->has_solo_yaw_traj_)
          {
            yaw_yawdot.first = current_trajectory_->traj_yaw_.getPos(t_cur)(0);
            yaw_yawdot.second = range(current_trajectory_->traj_yaw_.getVel(t_cur)(0));
          }
          else
          {
            /*** calculate yaw ***/
            Eigen::Vector3d dir = t_cur + time_forward_ <= current_trajectory_->traj_dur_ ? 
                                                            current_trajectory_->traj_3d_.getPos(t_cur + time_forward_) - pos :
                                                            current_trajectory_->traj_3d_.getPos(current_trajectory_->traj_dur_) - pos;
            yaw_yawdot = calculate_yaw(dir, (time_now - time_last_).seconds());
          }

          break;
        }
        case 4:
        {
          wp   = current_trajectory_->traj_with_yaw_.getPos(t_cur);
          dwp  = current_trajectory_->traj_with_yaw_.getVel(t_cur);
          ddwp = current_trajectory_->traj_with_yaw_.getAcc(t_cur);

          pos = wp.head(3);
          vel = dwp.head(3);
          acc = ddwp.head(3);

          yaw_yawdot.first = wp(3);
          yaw_yawdot.second = range(dwp(3));


          break;
        }
      }
      last_pos_ = pos;
    }
    else
    {
      pos = last_pos_;
      last_goal_ = pos;
      have_last_goal_ = true;
      // finish executing the trajectory
      if(t_cur <= current_trajectory_->traj_yaw_dur_ && current_trajectory_->has_solo_yaw_traj_)
      {
        yaw_yawdot.first  = current_trajectory_->traj_yaw_.getPos(t_cur)(0);
        yaw_yawdot.second = range(current_trajectory_->traj_yaw_.getVel(t_cur)(0));
      }
      else
      {
        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0.0;
        traj_set_ = false;
      }
    }

  }

  // publish the command
  position_cmd_.header.frame_id = msg->header.frame_id;
  position_cmd_.header.stamp = time_now;
  position_cmd_.position.x = pos(0);
  position_cmd_.position.y = pos(1);
  position_cmd_.position.z = pos(2);
  position_cmd_.velocity.x = vel(0);
  position_cmd_.velocity.y = vel(1);
  position_cmd_.velocity.z = vel(2);
  position_cmd_.acceleration.x = acc(0);
  position_cmd_.acceleration.y = acc(1);
  position_cmd_.acceleration.z = acc(2);
  position_cmd_.yaw = yaw_yawdot.first;
  position_cmd_.yaw_dot = yaw_yawdot.second;

  time_last_ = time_now;
  last_yaw_  = yaw_yawdot.first;

  return std::make_shared<kr_mav_msgs::msg::PositionCommand>(position_cmd_);
}

rclcpp_action::GoalResponse PolyTracker::goal_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PolyTrackerAction::Goal> goal)
{
  (void)uuid;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // If another goal is already active, we will preempt it by accepting the new goal
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_INFO(logger_, "PolyTracker: accepting new goal and will preempt current goal");
  }

  // Accept all goals (preemption will be handled in handle_accepted_callback)
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PolyTracker::cancel_callback(const std::shared_ptr<PolyTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO(logger_, "PolyTracker goal cancel requested");
  
  // If this is the current active goal, abort it
  if(goal_handle == current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_INFO(logger_, "PolyTracker: aborting current goal due to cancel request");
    goal_set_ = false;
    goal_reached_ = true;
    active_ = false;
  }
  
  // Allow cancellation
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PolyTracker::handle_accepted_callback(const std::shared_ptr<PolyTrackerGoalHandle> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  // If another goal is already active, abort it before accepting the new one
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    RCLCPP_INFO(logger_, "PolyTracker: aborting previous goal");
    current_goal_handle_->abort(std::make_shared<PolyTrackerAction::Result>());
  }
  
  // Store the current goal handle so update/activate can reference it
  current_goal_handle_ = goal_handle;
  // The goal payload can be accessed via goal_handle->get_goal()
  auto goal = goal_handle->get_goal();

  // Map goal fields to internal state similarly to original goal_callback logic
  if(goal->set_yaw)
  {
    goal_set_ = true;
    goal_reached_ = false;
    init_final_yaw_ = goal->final_yaw;
    init_dyaw_ = goal->final_yaw - cur_yaw_;
    if(goal->final_yaw < 0 && std::abs(init_dyaw_ + 2 * M_PI) < std::abs(init_dyaw_))
    {
      init_dyaw_ = init_dyaw_ + 2 * M_PI;
    }
    else if(cur_yaw_ < 0 && std::abs(init_dyaw_ - 2 * M_PI) < std::abs(init_dyaw_))
    {
      init_dyaw_ = init_dyaw_ - 2 * M_PI;
    }
    init_dyaw_ = range(init_dyaw_);
    if(init_dyaw_ > max_dyaw_) init_dyaw_ = max_dyaw_;
    else if(init_dyaw_ < -max_dyaw_) init_dyaw_ = -max_dyaw_;
    yaw_set_ = true;
    init_yaw_time_ = 0.0;
  }
  else if(goal->seg_x.size() > 0 || goal->knots.size() > 0)
  {
    // continuous trajectory: reuse original parsing logic but with goal message
    goal_set_ = true;
    goal_reached_ = false;
    double total_duration = 0.0;
    double total_yaw_duration = 0.0;
    std::vector<traj_opt::Piece<1>> segs_1d;
    std::vector<traj_opt::Piece<2>> segs_2d;
    std::vector<traj_opt::Piece<3>> segs_3d;
    std::vector<traj_opt::Piece<4>> segs_4d;
    next_trajectory_.reset(new TrajData);

    if(goal->cpts_status == 1)
    {
      RCLCPP_INFO(logger_, "PolyTracker: not implement now");
      return;
    }
    else if(goal->cpts_status == 2)
    {
      if(goal->yaw_pts.size() <= 0)
      {
        next_trajectory_->dim_ = 3;
      }
      else
      {
        next_trajectory_->dim_ = 4;
      }

      size_t N = goal->pos_pts.size() - 1;
      size_t M = goal->knots.size();
      size_t degree = M - N - 1;

      Eigen::MatrixXd pos_pts(N + 1, next_trajectory_->dim_);
      Eigen::VectorXd knots(M);
      for(size_t i = 0; i < M; ++i) knots(i) = goal->knots[i];
      for(size_t i = 0; i <= N; ++i)
      {
        pos_pts(i, 0) = goal->pos_pts[i].x;
        pos_pts(i, 1) = goal->pos_pts[i].y;
        pos_pts(i, 2) = goal->pos_pts[i].z;
      }

      if(next_trajectory_->dim_ == 3)
      {
        for(size_t i = 0; i < M - 2 * degree; i++)
        {
          Eigen::MatrixXd cpts(degree + 1, next_trajectory_->dim_);
          for(size_t j = 0; j <= degree; j++) cpts.row(j) = pos_pts.row(i + j);
          double dt = knots(degree + i + 1) - knots(degree + i);
          traj_opt::Piece<3> seg(traj_opt::BEZIER, cpts, dt, degree);
          segs_3d.push_back(seg);
          total_duration += dt;
        }
      }
      if(next_trajectory_->dim_ == 4)
      {
        for(size_t i = 0; i < goal->yaw_pts.size(); ++i) pos_pts(i, 3) = goal->yaw_pts[i];
        for(size_t i = 0; i < M - 2 * degree; i++)
        {
          Eigen::MatrixXd cpts(degree + 1, next_trajectory_->dim_);
          for(size_t j = 0; j <= degree; j++) cpts.row(j) = pos_pts.row(i + j);
          double dt = knots(degree + i + 1) - knots(degree + i);
          traj_opt::Piece<4> seg(traj_opt::BEZIER, cpts, dt, degree);
          segs_4d.push_back(seg);
          total_duration += dt;
        }
      }
    }
    else
    {
      size_t deg = goal->seg_x[0].degree;
      if(goal->seg_z.size() <= 0)
      {
        next_trajectory_->dim_ = 2;
      }
      else if(goal->seg_yaw.size() <= 0)
      {
        next_trajectory_->dim_ = 3;
      }
      else if(goal->separate_yaw)
      {
        next_trajectory_->dim_ = 3;
        next_trajectory_->has_solo_yaw_traj_ = true;
        for(size_t i = 0; i < goal->seg_yaw.size(); ++i)
        {
          Eigen::MatrixXd Coeffs_yaw(1, deg + 1);
          float dt = goal->seg_yaw[i].dt;
          total_yaw_duration += dt;
          for(size_t j = 0; j < deg + 1; ++j) Coeffs_yaw(0, j) = goal->seg_yaw[i].coeffs[j];
          traj_opt::Piece<1> seg(traj_opt::STANDARD, Coeffs_yaw, dt);
          segs_1d.push_back(seg);
        }
        next_trajectory_->traj_yaw_ = traj_opt::Trajectory1D(segs_1d, total_yaw_duration);
        next_trajectory_->traj_yaw_dur_ = total_yaw_duration;
      }

      for(size_t i = 0; i < goal->seg_x.size(); ++i)
      {
        Eigen::MatrixXd Coeffs(next_trajectory_->dim_, deg + 1);
        float dt = goal->seg_x[i].dt;
        total_duration += dt;
        for(size_t j = 0; j < deg + 1; ++j)
        {
          Coeffs(0, j) = goal->seg_x[i].coeffs[j];
          Coeffs(1, j) = goal->seg_y[i].coeffs[j];
        }
        switch(next_trajectory_->dim_)
        {
          case 2:
          {
            traj_opt::Piece<2> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_2d.push_back(seg);
            break;
          }
          case 3:
          {
            for(size_t j = 0; j < deg + 1; ++j) Coeffs(2, j) = goal->seg_z[i].coeffs[j];
            traj_opt::Piece<3> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_3d.push_back(seg);
            break;
          }
          case 4:
          {
            for(size_t j = 0; j < deg + 1; ++j)
            {
              Coeffs(2, j) = goal->seg_z[i].coeffs[j];
              Coeffs(3, j) = goal->seg_yaw[i].coeffs[j];
            }
            traj_opt::Piece<4> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_4d.push_back(seg);
            break;
          }
        }
      }
    }

    next_trajectory_->start_time_ = goal->t_start;
    next_trajectory_->traj_dur_ = total_duration;
    switch(next_trajectory_->dim_)
    {
      case 2:
        next_trajectory_->traj_2d_ = traj_opt::Trajectory2D(segs_2d, total_duration);
        break;
      case 3:
        next_trajectory_->traj_3d_ = traj_opt::Trajectory3D(segs_3d, total_duration);
        break;
      case 4:
        next_trajectory_->traj_with_yaw_ = traj_opt::Trajectory4D(segs_4d, total_duration);
        break;
    }
    traj_set_ = true;
    RCLCPP_WARN(logger_, "[Poly tracker] Duration: %f", total_duration);
    RCLCPP_INFO(logger_, "PolyTracker: Set the poly trajectory");
  }
  else if(goal->vel_pts.size() > 0)
  {
    double interval = goal->dt;
    int Num = goal->n;
    std::vector<Eigen::VectorXd> discrete_states;
    for(size_t j = 0; j < static_cast<size_t>(Num); ++j)
    {
      Eigen::VectorXd state(9);
      state << goal->pos_pts[j].x, goal->pos_pts[j].y, goal->pos_pts[j].z,
               goal->vel_pts[j].x, goal->vel_pts[j].y, goal->vel_pts[j].z,
               goal->acc_pts[j].x, goal->acc_pts[j].y, goal->acc_pts[j].z;
      discrete_states.push_back(state);
    }

    next_trajectory_->traj_discrete_ = traj_opt::DiscreteStates(interval, Num, discrete_states);
    next_trajectory_->dim_ = 1;
    next_trajectory_->start_time_ = goal->t_start;
    next_trajectory_->traj_dur_ = interval * (Num - 1);
    traj_set_ = true;
    RCLCPP_INFO(logger_, "PolyTracker: Set the discrete trajectory");
  }
  else
  {
    RCLCPP_WARN(logger_, "PolyTracker: Invalid goal received! Ignoring");
  }
}

/////////   some helper functions
std::pair<double, double> PolyTracker::calculate_yaw(Eigen::Vector3d &dir, double dt)
{
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double yawdot = 0;
  double d_yaw;

  d_yaw = range(yaw_temp - last_yaw_);



  const double YDM = d_yaw >= 0 ? max_dyaw_ : -max_dyaw_;
  const double YDDM = d_yaw >= 0 ? max_ddyaw_ : -max_ddyaw_;
  double d_yaw_max;
  if(fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if(fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }

  yawdot = d_yaw / dt;
  double yaw = last_yaw_ + d_yaw;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

uint8_t PolyTracker::status()
{
  // prefer checking current goal handle if available (other trackers use this pattern)
  if(current_goal_handle_ && current_goal_handle_->is_active())
  {
    return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::ACTIVE);
  }
  return static_cast<uint8_t>(kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(PolyTracker, kr_trackers_manager::Tracker);