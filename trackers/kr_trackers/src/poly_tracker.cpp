#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/PolyTrackerAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>
#include <traj_data.hpp>

class PolyTracker : public kr_trackers_manager::Tracker
{
 public:
  PolyTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

 private:
  void goal_callback();
  void preempt_callback();

  ros::Subscriber sub_poly_cmd_;
  kr_mav_msgs::PositionCommand position_cmd_;

  /*** current odom***/
  double cur_yaw_;
  Eigen::Vector3d cur_pos_, last_goal_;
  bool have_last_goal_ = false;

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::PolyTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer because plugin does not support a constructor with inputs, but an action server must be
  // initialized with a Nodehandle.
  std::unique_ptr<ServerType> tracker_server_;

  bool pos_set_, goal_set_, goal_reached_, active_;
  bool traj_set_ = false;

  // traj data
  ros::Time start_time_;
  int dim_;
  double traj_dur_ = 0, traj_yaw_dur_ = 0;
  traj_opt::Trajectory2D traj_2d_;
  traj_opt::Trajectory3D traj_3d_;
  traj_opt::Trajectory4D traj_with_yaw_;
  traj_opt::Trajectory1D traj_yaw_;
  bool has_solo_yaw_traj_ = false;

  // intial rotation
  double init_final_yaw_, init_dyaw_, init_yaw_time_;

  // yaw settings
  Eigen::Vector3d last_pos_;
  double last_yaw_ = 0.0, last_yawdot_ = 0.0;
  ros::Time time_last_;
  double time_forward_ = 1.5;
  bool yaw_set_ = false;
  double max_dyaw_ = 0.5 * M_PI;
  double max_ddyaw_ = M_PI;
  std::pair<double, double> calculate_yaw(Eigen::Vector3d &dir, double dt);
  double range(double angle);
};

PolyTracker::PolyTracker(void) : pos_set_(false), goal_set_(false), goal_reached_(false), active_(false) {}

void PolyTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "poly_tracker");

  // Set up the action server.
  tracker_server_.reset(new ServerType(priv_nh, "PolyTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&PolyTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&PolyTracker::preempt_callback, this));

  tracker_server_->start();
}

bool PolyTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(pos_set_)
  {
    if(!tracker_server_->isActive())
    {
      ROS_WARN("TrajectoryTracker::Activate: goal_set_ is true but action server has no active goal - not activating.");
      active_ = false;
      return false;
    }
    active_ = true;
    ROS_WARN("TrajectoryTracker::Activate: !");
  }
  return active_;
}

void PolyTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("PolyTracker::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }

  goal_set_ = false;
  active_ = false;
}

double PolyTracker::range(double angle)
{
  // range the angle into (-PI, PI]
  double psi = angle;
  if(angle > M_PI)
  {
    psi = 2 * M_PI - angle;
  }
  else if(angle <= -M_PI)
  {
    psi = angle + 2 * M_PI;
  }
  return psi;
}

kr_mav_msgs::PositionCommand::ConstPtr PolyTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;

  cur_pos_(0) = msg->pose.pose.position.x;
  cur_pos_(1) = msg->pose.pose.position.y;
  cur_pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);

  ros::Time time_now = ros::Time::now();

  if(!active_)
  {
    last_yaw_ = cur_yaw_;
    last_goal_ = cur_pos_;
    time_last_ = time_now;
    last_pos_ = cur_pos_;
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(last_yaw_, 0.0);
  Eigen::VectorXd wp(dim_), dwp(dim_), ddwp(dim_);

  if(have_last_goal_ && (cur_pos_ - last_goal_).norm() <= 0.3)
  {
    pos = last_goal_;
  }
  else
  {
    pos = last_pos_;
  }

  if(yaw_set_)
  {
    double dyaw = range(init_final_yaw_ - cur_yaw_);

    if(abs(dyaw) < 0.5 || init_yaw_time_ > 2.0)
    {
      yaw_set_ = false;
      // ROS_INFO(" yaw_set finished ");
      time_last_ = time_now;
      return kr_mav_msgs::PositionCommand::Ptr();
    }

    double yaw_temp = cur_yaw_ + (time_now - time_last_).toSec() * init_dyaw_;
    double desired_yaw =
        init_final_yaw_ - cur_yaw_ >= 0 ? std::min(yaw_temp, init_final_yaw_) : std::max(yaw_temp, init_final_yaw_);

    yaw_yawdot.first = desired_yaw;
    yaw_yawdot.second = init_dyaw_;

    init_yaw_time_ += (time_now - time_last_).toSec();
  }
  else if(traj_set_)
  {
    double t_cur = (time_now - start_time_).toSec();

    if(t_cur < traj_dur_ && t_cur >= 0.0)
    {
      
      switch(dim_)
      {
        case 2:
        {
          wp = traj_2d_.getPos(t_cur);
          dwp = traj_2d_.getVel(t_cur);
          pos.head(2) = wp;
          pos(2) = last_goal_(2);
          vel.head(2) = dwp;
          break;
        }
        case 3:
        {
          pos = traj_3d_.getPos(t_cur);
          vel = traj_3d_.getVel(t_cur);
          acc = traj_3d_.getAcc(t_cur);

          if(has_solo_yaw_traj_)
          {
            yaw_yawdot.first = traj_yaw_.getPos(t_cur)(0);
            yaw_yawdot.second = range(traj_yaw_.getVel(t_cur)(0));
          }
          else
          {
            /*** calculate yaw ***/
            Eigen::Vector3d dir = t_cur + time_forward_ <= traj_dur_ ? traj_3d_.getPos(t_cur + time_forward_) - pos :
                                                                       traj_3d_.getPos(traj_dur_) - pos;
            yaw_yawdot = calculate_yaw(dir, (time_now - time_last_).toSec());
          }

          break;
        }
        case 4:
        {
          wp = traj_with_yaw_.getPos(t_cur);
          dwp = traj_with_yaw_.getVel(t_cur);
          ddwp = traj_with_yaw_.getAcc(t_cur);

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
      if(t_cur <= traj_yaw_dur_ && has_solo_yaw_traj_)
      {
        yaw_yawdot.first = traj_yaw_.getPos(t_cur)(0);
        yaw_yawdot.second = range(traj_yaw_.getVel(t_cur)(0));
      }
      else
      {
        has_solo_yaw_traj_ = false;
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
  last_yaw_ = yaw_yawdot.first;

  return kr_mav_msgs::PositionCommand::ConstPtr(new kr_mav_msgs::PositionCommand(position_cmd_));
}

void PolyTracker::goal_callback()
{
  // If another goal is already active, cancel that goal and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("PolyTracker goal aborted");
    tracker_server_->setAborted();
  }

  // Pointer to the recieved goal.
  const auto msg = tracker_server_->acceptNewGoal();

  // If preempt has been requested, then set this goal to preempted and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("PolyTracker preempted");
    tracker_server_->setPreempted();
    return;
  }

  if(msg->set_yaw == true)
  {
    goal_set_ = true;
    goal_reached_ = false;

    /***  set the desired dyaw   ***/
    init_final_yaw_ = msg->final_yaw;
    init_dyaw_ = msg->final_yaw - cur_yaw_;

    if(msg->final_yaw < 0 && abs(init_dyaw_ + 2 * M_PI) < abs(init_dyaw_))
    {
      init_dyaw_ = init_dyaw_ + 2 * M_PI;
    }
    else if(cur_yaw_ < 0 && abs(init_dyaw_ - 2 * M_PI) < abs(init_dyaw_))
    {
      init_dyaw_ = init_dyaw_ - 2 * M_PI;
    }

    init_dyaw_ = range(init_dyaw_);

    // clip the yaw dot
    if(init_dyaw_ > max_dyaw_)
    {
      init_dyaw_ = max_dyaw_;
    }
    else if(init_dyaw_ < -max_dyaw_)
    {
      init_dyaw_ = -max_dyaw_;
    }

    yaw_set_ = true;
    init_yaw_time_ = 0.0;
  }
  else if(msg->seg_x.size() > 0 || msg->pos_pts.size() > 0)
  {
    goal_set_ = true;
    goal_reached_ = false;
    traj_set_ = true;
    traj_dur_ = 0.0;
    dim_ = 4;
    std::vector<traj_opt::Piece<1>> segs_1d;
    std::vector<traj_opt::Piece<2>> segs_2d;
    std::vector<traj_opt::Piece<3>> segs_3d;
    std::vector<traj_opt::Piece<4>> segs_4d;

    if(msg->cpts_status == 1)
    {
    }
    else if(msg->cpts_status == 2)  // bspline only support 3d or 3d with yaw
    {
      if(msg->yaw_pts.size() <= 0)
      {
        dim_ = 3;
      }

      int N = msg->pos_pts.size() - 1;
      int M = msg->knots.size();
      int degree = M - N - 1;

      Eigen::MatrixXd pos_pts(N + 1, dim_);  // N + 1
      Eigen::VectorXd knots(M);              // N + degree + 1

      for(long unsigned int i = 0; i < M; ++i)
      {
        knots(i) = msg->knots[i];
      }

      for(unsigned int i = 0; i <= N; ++i)
      {
        pos_pts(i, 0) = msg->pos_pts[i].x;
        pos_pts(i, 1) = msg->pos_pts[i].y;
        pos_pts(i, 2) = msg->pos_pts[i].z;
      }

      if(dim_ == 3)
      {
        for(int i = 0; i < M - 2 * degree; i++)
        {
          Eigen::MatrixXd cpts;
          cpts.resize(degree + 1, dim_);
          for(int j = 0; j <= degree; j++)
          {
            cpts.row(j) = pos_pts.row(i + j);
          }

          double dt = knots(degree + i + 1) - knots(degree + i);
          traj_opt::Piece<3> seg(traj_opt::BEZIER, cpts, dt, degree);
          segs_3d.push_back(seg);
          traj_dur_ += dt;
        }
      }
      if(dim_ == 4)
      {
        for(unsigned int i = 0; i < msg->yaw_pts.size(); ++i)
        {
          pos_pts(i, 3) = msg->yaw_pts[i];
        }
        // M =  N + degree + 1
        // std::cout << "pos_pts is " <<pos_pts<< std::endl;
        for(int i = 0; i < M - 2 * degree; i++)
        {
          Eigen::MatrixXd cpts;
          cpts.resize(degree + 1, dim_);
          for(int j = 0; j <= degree; j++)
          {
            cpts.row(j) = pos_pts.row(i + j);
          }
          // std::cout << "cpts is " <<cpts<< std::endl;
          double dt = knots(degree + i + 1) - knots(degree + i);  // t_degree, t_M-degree
          traj_opt::Piece<4> seg(traj_opt::BEZIER, cpts, dt, degree);
          segs_4d.push_back(seg);
          traj_dur_ += dt;
        }
      }
    }
    else
    {
      int deg = msg->seg_x[0].degree;

      if(msg->seg_z.size() <= 0)
      {
        dim_ = 2;
      }
      else if(msg->seg_yaw.size() <= 0)
      {
        dim_ = 3;
      }
      else if(msg->separate_yaw)  // position and yaw are optimized separately
      {
        dim_ = 3;
        has_solo_yaw_traj_ = true;
        traj_yaw_dur_ = 0.0;  // always larger than normal trajectory time
        for(int i = 0; i < msg->seg_yaw.size(); ++i)
        {
          Eigen::MatrixXd Coeffs_yaw(1, deg + 1);
          float dt = msg->seg_yaw[i].dt;
          traj_yaw_dur_ += dt;
          for(int j = 0; j < deg + 1; ++j)
          {
            Coeffs_yaw(0, j) = msg->seg_yaw[i].coeffs[j];
          }
          traj_opt::Piece<1> seg(traj_opt::STANDARD, Coeffs_yaw, dt);
          segs_1d.push_back(seg);
        }
        traj_yaw_ = traj_opt::Trajectory1D(segs_1d, traj_yaw_dur_);
      }

      for(int i = 0; i < msg->seg_x.size(); ++i)
      {
        Eigen::MatrixXd Coeffs(dim_, deg + 1);

        float dt = msg->seg_x[i].dt;
        traj_dur_ += dt;

        for(int j = 0; j < deg + 1; ++j)
        {
          Coeffs(0, j) = msg->seg_x[i].coeffs[j];
          Coeffs(1, j) = msg->seg_y[i].coeffs[j];
        }
        switch(dim_)
        {
          case 2:
          {
            traj_opt::Piece<2> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_2d.push_back(seg);
            break;
          }
          case 3:
          {
            for(int j = 0; j < deg + 1; ++j)
            {
              Coeffs(2, j) = msg->seg_z[i].coeffs[j];
            }
            traj_opt::Piece<3> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_3d.push_back(seg);
            break;
          }
          case 4:
          {
            for(int j = 0; j < deg + 1; ++j)
            {
              Coeffs(2, j) = msg->seg_z[i].coeffs[j];
              Coeffs(3, j) = msg->seg_yaw[i].coeffs[j];
            }
            traj_opt::Piece<4> seg(traj_opt::STANDARD, Coeffs, dt);
            segs_4d.push_back(seg);
            break;
          }
        }
      }
    }

    /* Store data */
    start_time_ = msg->t_start;

    switch(dim_)
    {
      case 2:
        traj_2d_ = traj_opt::Trajectory2D(segs_2d, traj_dur_);
        break;
      case 3:
        traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj_dur_);
        break;
      case 4:
        traj_with_yaw_ = traj_opt::Trajectory4D(segs_4d, traj_dur_);
        break;
    }
    ROS_INFO("PolyTracker: set the poly trajectory");
  }
  else
  {
    ROS_WARN("PolyTracker: Invalid goal received! Ignoring");
  }

  // ROS_INFO("goal callback");
  return;
}

void PolyTracker::preempt_callback()
{
  if(tracker_server_->isActive())
  {
    ROS_INFO("PolyTracker aborted");
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("PolyTracker preempted");
    tracker_server_->setPreempted();
  }

  goal_set_ = false;
  goal_reached_ = true;
}

/////////   some helper functions
std::pair<double, double> PolyTracker::calculate_yaw(Eigen::Vector3d &dir, double dt)
{
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double yawdot = 0;
  double d_yaw = range(yaw_temp - last_yaw_);

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
  double yaw = range(last_yaw_ + d_yaw);

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

uint8_t PolyTracker::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PolyTracker, kr_trackers_manager::Tracker)
