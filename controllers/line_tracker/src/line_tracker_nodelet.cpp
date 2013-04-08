#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_datatypes.h>

class LineTracker : public nodelet::Nodelet
{
 public:
  LineTracker() :
      pos_set_(false),
      goal_set_(false),
      goal_reached_(false)
  {
  }

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void goal_callback(const geometry_msgs::Point::ConstPtr &msg);

  ros::Publisher pub_cmd_;
  ros::Subscriber sub_odom_, sub_goal_;
  Eigen::Vector3f start_, goal_, pos_;
  bool pos_set_, goal_set_, goal_reached_;
  double v_des_, a_des_, epsilon_;
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];
};

void LineTracker::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  static ros::Time t_prev;
  const double dT = (msg->header.stamp - t_prev).toSec();
  t_prev = msg->header.stamp;

  if(!goal_set_) return;

  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = msg->header.frame_id;
  cmd.yaw = start_yaw_;
  cmd.yaw_dot = 0;
  cmd.kx[0] = kx_[0], cmd.kx[1] = kx_[1], cmd.kx[2] = kx_[2];
  cmd.kv[0] = kv_[0], cmd.kv[1] = kv_[1], cmd.kv[2] = kv_[2];

  if(goal_reached_)
  {
    cmd.position.x = goal_(0), cmd.position.y = goal_(1), cmd.position.z = goal_(2);
    cmd.velocity.x = 0, cmd.velocity.y = 0, cmd.velocity.z = 0;
    cmd.acceleration.x = 0, cmd.acceleration.y = 0, cmd.acceleration.z = 0;
    pub_cmd_.publish(cmd);
    return;
  }

  const Eigen::Vector3f dir = (goal_ - start_).normalized();
  const float total_dist = (goal_ - start_).norm();
  const float d = (pos_ - start_).dot(dir);
  const Eigen::Vector3f proj = start_ + d*dir;

  const float v_max = std::min(std::sqrt(a_des_*total_dist), v_des_);
  const float ramp_dist = v_max*v_max/(2*a_des_);

  Eigen::Vector3f x, v, a;

  if((pos_ - goal_).norm() <= epsilon_) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(d > total_dist) // Overshoot
  {
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des_*dir;
    v = Eigen::Vector3f::Zero();
    x = goal_;
  }
  else if(d >= (total_dist - ramp_dist) && d <= total_dist) // Decelerate
  {
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des_*dir;
    v = std::sqrt(2*a_des_*(total_dist - d))*dir;
    x = proj + v*dT + 0.5*a*dT*dT;
  }
  else if(d > ramp_dist && d < total_dist - ramp_dist) // Constant velocity
  {
    ROS_DEBUG_THROTTLE(1, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max*dir;
    x = proj + v*dT;
  }
  else if(d >= 0 && d <= ramp_dist) // Accelerate
  {
    ROS_DEBUG_THROTTLE(1, "Accelerate");
    a = a_des_*dir;
    v = std::sqrt(2*a_des_*d)*dir;
    x = proj + v*dT + 0.5*a*dT*dT;
  }
  else if(d < 0)
  {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des_*dir;
    v = Eigen::Vector3f::Zero();
    x = start_;
  }
  cmd.position.x = x(0), cmd.position.y = x(1), cmd.position.z = x(2);
  cmd.velocity.x = v(0), cmd.velocity.y = v(1), cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0), cmd.acceleration.y = a(1), cmd.acceleration.z = a(2);
  pub_cmd_.publish(cmd);
}

void LineTracker::goal_callback(const geometry_msgs::Point::ConstPtr &msg)
{
  if(!pos_set_)
  {
    ROS_WARN("Initial position not set, not setting goal");
    return;
  }

  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;

  start_ = pos_;
  start_yaw_ = yaw_;

  goal_set_ = true;
  goal_reached_ = false;
}

void LineTracker::onInit(void)
{
  ros::NodeHandle nh(getPrivateNodeHandle());

  nh.param("v_des", v_des_, 0.5);
  nh.param("a_des", a_des_, 0.5);
  nh.param("epsilon", epsilon_, 0.1);

  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  sub_odom_ = nh.subscribe("odom", 10, &LineTracker::odom_callback, this, ros::TransportHints().tcpNoDelay());
  sub_goal_ = nh.subscribe("goal", 2, &LineTracker::goal_callback, this, ros::TransportHints().tcpNoDelay());

  pub_cmd_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(line_tracker, LineTracker, LineTracker, nodelet::Nodelet);
