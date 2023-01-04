#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/PolyTrackerAction.h>
#include <poly_util.hpp>
#include <Eigen/Eigen>
#include <actionlib/server/simple_action_server.h>
#include <kr_trackers/initial_conditions.h>


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

  double last_t_;

  /*** current odom***/
  double cur_yaw_;
  Eigen::Vector3d cur_pos_, last_goal_;
  bool have_last_goal_ = false;

  double range(double angle);




  typedef actionlib::SimpleActionServer<kr_tracker_msgs::PolyTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer because plugin does not support a constructor with inputs, but an action server must be
  // initialized with a Nodehandle.
  std::unique_ptr<ServerType> tracker_server_;


  bool pos_set_, goal_set_, goal_reached_, active_;
  bool traj_set_ = false;

  ros::Time start_time_;
  min_jerk::Trajectory traj_;
  min_yaw_jerk::Trajectory yaw_traj_;
  double traj_dur_;

  // for intial rotation
  double init_final_yaw_, init_dyaw_;
  double init_yaw_time_;

  // yaw control
  double last_yaw_ = 0.0, last_yawdot_ = 0.0;
  ros::Time time_last_;
  double time_forward_ = 1.5;
  bool yaw_set_ = false;
  double YAW_DOT_MAX_PER_SEC = 0.5* M_PI;
  double YAW_DOT_DOT_MAX_PER_SEC = M_PI;



  std::pair<double, double> calculate_yaw(Eigen::Vector3d &dir, double dt);



  //kr_tracker_msgs::TrajectoryTrackerGoal goal_;

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

double PolyTracker::range(double angle){
  // range the angle into (-PI, PI]
  double psi = angle;

  if (angle > M_PI)
  {
    psi = 2 * M_PI - angle;
  }
  else if (angle <= -M_PI)
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

  if(!active_){
    last_yaw_ = cur_yaw_;
    time_last_ = time_now;
    //ROS_INFO("PolyTracker::not active!");
    return kr_mav_msgs::PositionCommand::Ptr();
  }
    
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(last_yaw_ , 0.0);

  if (have_last_goal_ && (cur_pos_- last_goal_).norm() <= 0.3){
    pos = last_goal_;
  }else{
    pos = cur_pos_;
  }
  

  if (yaw_set_){
    //std::cout << "cur_yaw_ is " << cur_yaw_ << " init_final_yaw_ is " << init_final_yaw_ << std::endl;

    double dyaw = range(init_final_yaw_ - cur_yaw_);

    if (abs(dyaw) < 0.5 || init_yaw_time_ > 2.0){
      yaw_set_ = false;
      //ROS_INFO(" yaw_set finished ");
      time_last_ = time_now;
      return kr_mav_msgs::PositionCommand::Ptr();
    }

    double yaw_temp = cur_yaw_ + (time_now  - time_last_).toSec() * init_dyaw_;

    double desired_yaw = init_final_yaw_ - cur_yaw_ >= 0 ? std::min(yaw_temp, init_final_yaw_) : std::max(yaw_temp, init_final_yaw_);

    yaw_yawdot.first  = desired_yaw;
    yaw_yawdot.second = init_dyaw_;
    
    init_yaw_time_ += (time_now  - time_last_).toSec();

  }else if (traj_set_){

    double t_cur = (time_now - start_time_).toSec();

    if (t_cur < traj_dur_ && t_cur >= 0.0)
    {
      pos = traj_.getPos(t_cur);
      vel = traj_.getVel(t_cur);
      acc = traj_.getAcc(t_cur);

      if(yaw_traj_.is_empty())
      {
        /*** calculate yaw ***/
        Eigen::Vector3d dir = t_cur + time_forward_ <= traj_dur_
                                ? traj_.getPos(t_cur + time_forward_) - pos
                                : traj_.getPos(traj_dur_) - pos;
        yaw_yawdot = calculate_yaw(dir, (time_now - time_last_).toSec());
        /*** calculate yaw ***/

      }else{

        yaw_yawdot.first   = range(yaw_traj_.getPos(t_cur));
        yaw_yawdot.second  = range(yaw_traj_.getVel(t_cur));

      }

    }else if ( t_cur >= traj_dur_ ){

      pos = traj_.getPos(traj_dur_);
      last_goal_ = pos;
      have_last_goal_ = true;
      // finish executing the trajectory
      traj_set_ = false;

      yaw_yawdot.first   = last_yaw_;
      yaw_yawdot.second  = 0.0;
    }

  }
  
  //publish the command
  position_cmd_.header.frame_id = msg->header.frame_id;
  position_cmd_.header.stamp = time_now;
  //std::cout << " yaw_yawdot.first " << yaw_yawdot.first << std::endl;
  position_cmd_.position.x       = pos(0);
  position_cmd_.position.y       = pos(1);
  position_cmd_.position.z       = pos(2);
  position_cmd_.velocity.x       = vel(0);
  position_cmd_.velocity.y       = vel(1);
  position_cmd_.velocity.z       = vel(2);
  position_cmd_.acceleration.x   = acc(0);
  position_cmd_.acceleration.y   = acc(1);
  position_cmd_.acceleration.z   = acc(2);
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

    std::cout << "receive goal"  <<std::endl;

  if (msg->set_yaw == true){
    goal_set_ = true;
    goal_reached_ = false;

    /***  set the desired dyaw   ***/
    init_final_yaw_ = msg->final_yaw;


    init_dyaw_ = msg->final_yaw - cur_yaw_;

    if ( msg->final_yaw  < 0 && abs(init_dyaw_+ 2 * M_PI) < abs(init_dyaw_) ){

      init_dyaw_ = init_dyaw_ + 2 * M_PI;

    }
    else if (cur_yaw_ < 0 && abs(init_dyaw_- 2 * M_PI) < abs(init_dyaw_) ){

      init_dyaw_ = init_dyaw_ - 2 * M_PI;

    }

    init_dyaw_ = range(init_dyaw_);

    // clip the yaw dot
    if (init_dyaw_ > YAW_DOT_MAX_PER_SEC)
    {
      init_dyaw_ = YAW_DOT_MAX_PER_SEC;
    }
    else if (init_dyaw_ < -YAW_DOT_MAX_PER_SEC)
    {
      init_dyaw_ = -YAW_DOT_MAX_PER_SEC;
    }


    yaw_set_ = true;
    init_yaw_time_ = 0.0;

  }else if(msg->order == 5 && msg->duration.size() * (msg->order + 1) == msg->bound_x.size() )
  {
    goal_set_ = true;
    goal_reached_ = false;
    traj_set_ = true;

    int piece_nums = msg->duration.size();


    std::vector<double> dura(piece_nums);
    std::vector<min_jerk::BoundaryCond> bCond(piece_nums);

    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      bCond[i].row(0) << msg->bound_x[i6 + 0], msg->bound_x[i6 + 1], msg->bound_x[i6 + 2],
          msg->bound_x[i6 + 3], msg->bound_x[i6 + 4], msg->bound_x[i6 + 5];
      bCond[i].row(1) << msg->bound_y[i6 + 0], msg->bound_y[i6 + 1], msg->bound_y[i6 + 2],
          msg->bound_y[i6 + 3], msg->bound_y[i6 + 4], msg->bound_y[i6 + 5];
      bCond[i].row(2) << msg->bound_z[i6 + 0], msg->bound_z[i6 + 1], msg->bound_z[i6 + 2],
          msg->bound_z[i6 + 3], msg->bound_z[i6 + 4], msg->bound_z[i6 + 5];
      
      dura[i] = msg->duration[i];
    }

    /* Store data */
    start_time_ = msg->start_time;
    traj_ = min_jerk::Trajectory(bCond, dura);
    traj_dur_ = traj_.getTotalDuration();

    /* If has yaw traj */
    if(msg->bound_yaw.size() > 0)
    {
      std::vector<min_yaw_jerk::BoundaryCond> bCond_yaw(piece_nums);
      for (int i = 0; i < piece_nums; ++i)
      {
        int i6 = i * 6;
        bCond_yaw[i].row(0) << msg->bound_yaw[i6 + 0], msg->bound_yaw[i6 + 1], msg->bound_yaw[i6 + 2],
            msg->bound_yaw[i6 + 3], msg->bound_yaw[i6 + 4], msg->bound_yaw[i6 + 5]; 
      }
      yaw_traj_ = min_yaw_jerk::Trajectory(bCond_yaw, dura);
    }


    ROS_WARN("PolyTracker: set the poly trajectory");
  }
  else
  {
    ROS_WARN("PolyTracker: Invalid goal received! Ignoring");
  }

  ROS_INFO("goal callback");
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

  double yaw_temp = dir.norm() > 0.1
                  ? atan2(dir(1), dir(0))
                  : last_yaw_;
  //std::cout << " last_yaw_ is  " << last_yaw_ << std::endl;
  double yawdot = 0;
  double d_yaw = range(yaw_temp - last_yaw_);

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }

  // std::cout << " d_yaw is  " << d_yaw << std::endl;
  // std::cout << " dt is  " << dt << std::endl;

  yawdot = d_yaw / dt;
  //std::cout << " yawdot is  " << yawdot << std::endl;

  double yaw = range(last_yaw_ + d_yaw);
  
  //std::cout << " yaw is  " << yaw << std::endl;
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
