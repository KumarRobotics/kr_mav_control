#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LissajousAdderAction.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers/lissajous_generator.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

class LissajousAdder : public kr_trackers_manager::Tracker
{
 public:
  LissajousAdder(void);
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

 private:
  void goal_callback(void);
  void preempt_callback(void);

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::LissajousAdderAction> ServerType;
  std::shared_ptr<ServerType> tracker_server_;
  ros::Publisher path_pub_;

  InitialConditions ICs_;
  LissajousGenerator generator_1_, generator_2_;
  double distance_traveled_;
  Eigen::Vector3d position_last_;
  bool traj_start_set_;
  std::string frame_id_;
};

LissajousAdder::LissajousAdder(void) : traj_start_set_(false) {}

void LissajousAdder::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "lissajous_adder");
  priv_nh.param<std::string>("frame_id", frame_id_, "world");
  path_pub_ = priv_nh.advertise<nav_msgs::Path>("lissajous_path", 1);

  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(priv_nh, "LissajousAdder", false));
  tracker_server_->registerGoalCallback(boost::bind(&LissajousAdder::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&LissajousAdder::preempt_callback, this));
  tracker_server_->start();
}

bool LissajousAdder::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(generator_1_.goalIsSet() && generator_2_.goalIsSet())
  {
    if(!tracker_server_->isActive())
    {
      ROS_WARN("LissajousAdder::Activate: goal_set is true but action server has no active goal - not activating.");
      return false;
    }
    return (generator_1_.activate() && generator_2_.activate());
  }
  return false;
}

void LissajousAdder::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("Deactivated tracker prior to reaching goal");
    tracker_server_->setAborted();
  }
  ICs_.reset();
  generator_1_.deactivate();
  generator_2_.deactivate();
  traj_start_set_ = false;
}

kr_mav_msgs::PositionCommand::ConstPtr LissajousAdder::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!(generator_1_.isActive() & generator_2_.isActive()))
  {
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  if(!traj_start_set_)
  {
    traj_start_set_ = true;
    ICs_.set_from_odom(msg);
    position_last_ = Eigen::Vector3d(ICs_.pos()(0), ICs_.pos()(1), ICs_.pos()(2));

    // Generate path for visualizing
    geometry_msgs::Point initial_pt, initial_pt2;
    initial_pt.x = ICs_.pos()(0);
    initial_pt.y = ICs_.pos()(1);
    initial_pt.z = ICs_.pos()(2);
    double dt = 0.1;
    nav_msgs::Path path1, path2;
    path1.header.frame_id = frame_id_;
    path1.header.stamp = ros::Time::now();
    generator_1_.generatePath(path1, initial_pt, dt);
    generator_2_.generatePath(path2, initial_pt2, dt);
    for(unsigned int i = 0; i < path1.poses.size(); i++)
    {
      path1.poses[i].pose.position.x += path2.poses[i].pose.position.x;
      path1.poses[i].pose.position.y += path2.poses[i].pose.position.y;
      path1.poses[i].pose.position.z += path2.poses[i].pose.position.z;
    }
    path_pub_.publish(path1);
  }

  // Set gains
  kr_mav_msgs::PositionCommand::Ptr cmd1 = generator_1_.getPositionCmd();
  kr_mav_msgs::PositionCommand::Ptr cmd2 = generator_2_.getPositionCmd();
  if(cmd1 == NULL && cmd2 == NULL)
  {
    return cmd1;
  }
  else
  {
    cmd1->header.stamp = ros::Time::now();
    cmd1->header.frame_id = msg->header.frame_id;
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

    // Publish feedback and compute distance traveled
    if(!generator_1_.status() || !generator_2_.status())
    {
      kr_tracker_msgs::LissajousAdderFeedback feedback;
      feedback.time_to_completion = std::max(generator_1_.timeRemaining(), generator_2_.timeRemaining());
      tracker_server_->publishFeedback(feedback);

      Eigen::Vector3d position_current =
          Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      distance_traveled_ += (position_current - position_last_).norm();
      position_last_ = position_current;
    }
    else if(tracker_server_->isActive())
    {
      kr_tracker_msgs::LissajousAdderResult result;
      result.x = msg->pose.pose.position.x;
      result.y = msg->pose.pose.position.y;
      result.z = msg->pose.pose.position.z;
      result.yaw = ICs_.yaw();  // TODO: Change this to the yaw from msg
      result.duration = std::max(generator_1_.timeElapsed(), generator_2_.timeElapsed());
      result.length = distance_traveled_;
      tracker_server_->setSucceeded(result);
    }
    return cmd1;
  }
}

uint8_t LissajousAdder::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

void LissajousAdder::goal_callback(void)
{
  if(generator_1_.goalIsSet() || generator_2_.goalIsSet())
  {
    return;
  }

  kr_tracker_msgs::LissajousAdderGoal::ConstPtr msg = tracker_server_->acceptNewGoal();

  if(tracker_server_->isPreemptRequested())
  {
    tracker_server_->setPreempted();
    return;
  }

  traj_start_set_ = false;
  generator_1_.setParams(msg, 0);
  generator_2_.setParams(msg, 1);
  distance_traveled_ = 0;
  generator_1_.activate();
  generator_2_.activate();
}

void LissajousAdder::preempt_callback(void)
{
  if(tracker_server_->isActive())
  {
    tracker_server_->setAborted();
  }
  else
  {
    tracker_server_->setPreempted();
  }

  traj_start_set_ = false;
  generator_1_.deactivate();
  generator_2_.deactivate();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LissajousAdder, kr_trackers_manager::Tracker);
