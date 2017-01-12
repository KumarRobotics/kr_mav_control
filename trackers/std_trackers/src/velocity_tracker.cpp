#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <tf/transform_datatypes.h>

class VelocityTracker : public trackers_manager::Tracker
{
 public:
  VelocityTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);
  void position_velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg);

  ros::Subscriber sub_vel_cmd_, sub_position_vel_cmd_;
  quadrotor_msgs::PositionCommand position_cmd_;
  bool odom_set_, active_, use_position_gains_;
  double last_t_;
  double kx_[3], kv_[3];
  double pos_[3], cur_yaw_;
};

VelocityTracker::VelocityTracker(void) :
    odom_set_(false),
    active_(false),
    use_position_gains_(false),
    last_t_(0)
{
}

void VelocityTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "velocity_tracker");

  sub_vel_cmd_ = priv_nh.subscribe("goal", 10, &VelocityTracker::velocity_cmd_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  sub_position_vel_cmd_ = priv_nh.subscribe("position_velocity_goal", 10, &VelocityTracker::position_velocity_cmd_cb,
                                            this, ros::TransportHints().tcpNoDelay());

  position_cmd_.kv[0] = kv_[0], position_cmd_.kv[1] = kv_[1], position_cmd_.kv[2] = kv_[2];
}

bool VelocityTracker::Activate(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if(cmd)
  {
    position_cmd_.position = cmd->position;
    position_cmd_.yaw = cmd->yaw;

    active_ = true;
  }
  else if(odom_set_)
  {
    position_cmd_.position.x = pos_[0];
    position_cmd_.position.y = pos_[1];
    position_cmd_.position.z = pos_[2];
    position_cmd_.yaw = cur_yaw_;

    active_ = true;
  }

  return active_;
}

void VelocityTracker::Deactivate(void)
{
  active_ = false;
  odom_set_ = false;
  last_t_ = 0;
}

const quadrotor_msgs::PositionCommand::ConstPtr VelocityTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;
  pos_[2] = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  odom_set_ = true;

  if(!active_)
    return quadrotor_msgs::PositionCommand::Ptr();


  if(last_t_ == 0)
    last_t_ = ros::Time::now().toSec();

  const double t_now =  ros::Time::now().toSec();
  const double dt = t_now - last_t_;
  last_t_ = t_now;

  if(use_position_gains_)
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

  return quadrotor_msgs::PositionCommand::ConstPtr(new quadrotor_msgs::PositionCommand(position_cmd_));
}

void VelocityTracker::velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;

  use_position_gains_ = false;
}

void VelocityTracker::position_velocity_cmd_cb(const quadrotor_msgs::FlatOutputs::ConstPtr &msg)
{
  position_cmd_.velocity.x = msg->x;
  position_cmd_.velocity.y = msg->y;
  position_cmd_.velocity.z = msg->z;
  position_cmd_.yaw_dot = msg->yaw;

  use_position_gains_ = true;
}

const quadrotor_msgs::TrackerStatus::Ptr VelocityTracker::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(VelocityTracker, trackers_manager::Tracker)
