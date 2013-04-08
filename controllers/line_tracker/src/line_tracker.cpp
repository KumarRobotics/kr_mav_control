#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_datatypes.h>

static ros::Publisher pub_cmd;
static Eigen::Vector3f start, goal, pos;
static bool pos_set = false, goal_set = false, goal_reached = false;
static double v_des, a_des, epsilon;
static float yaw, start_yaw;
static double kx[3], kv[3];

static void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos(0) = msg->pose.pose.position.x;
  pos(1) = msg->pose.pose.position.y;
  pos(2) = msg->pose.pose.position.z;
  yaw = tf::getYaw(msg->pose.pose.orientation);
  pos_set = true;

  static ros::Time t_prev;
  const double dT = (msg->header.stamp - t_prev).toSec();
  t_prev = msg->header.stamp;

  if(!goal_set) return;

  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = msg->header.frame_id;
  cmd.yaw = start_yaw;
  cmd.yaw_dot = 0;
  cmd.kx[0] = kx[0], cmd.kx[1] = kx[1], cmd.kx[2] = kx[2];
  cmd.kv[0] = kv[0], cmd.kv[1] = kv[1], cmd.kv[2] = kv[2];

  if(goal_reached)
  {
    cmd.position.x = goal(0), cmd.position.y = goal(1), cmd.position.z = goal(2);
    cmd.velocity.x = 0, cmd.velocity.y = 0, cmd.velocity.z = 0;
    cmd.acceleration.x = 0, cmd.acceleration.y = 0, cmd.acceleration.z = 0;
    pub_cmd.publish(cmd);
    return;
  }

  const Eigen::Vector3f dir = (goal - start).normalized();
  const float total_dist = (goal - start).norm();
  const float d = (pos - start).dot(dir);
  const Eigen::Vector3f proj = start + d*dir;

  const float v_max = std::min(std::sqrt(a_des*total_dist), v_des);
  const float ramp_dist = v_max*v_max/(2*a_des);

  Eigen::Vector3f x, v, a;

  if((pos - goal).norm() <= epsilon) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal;
    goal_reached = true;
  }
  else if(d > total_dist) // Overshoot
  {
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des*dir;
    v = Eigen::Vector3f::Zero();
    x = goal;
  }
  else if(d >= (total_dist - ramp_dist) && d <= total_dist) // Decelerate
  {
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des*dir;
    v = std::sqrt(2*a_des*(total_dist - d))*dir;
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
    a = a_des*dir;
    v = std::sqrt(2*a_des*d)*dir;
    x = proj + v*dT + 0.5*a*dT*dT;
  }
  else if(d < 0)
  {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des*dir;
    v = Eigen::Vector3f::Zero();
    x = start;
  }
  cmd.position.x = x(0), cmd.position.y = x(1), cmd.position.z = x(2);
  cmd.velocity.x = v(0), cmd.velocity.y = v(1), cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0), cmd.acceleration.y = a(1), cmd.acceleration.z = a(2);
  pub_cmd.publish(cmd);
}

static void goal_callback(const geometry_msgs::Point::ConstPtr &msg)
{
  if(!pos_set)
  {
    ROS_WARN("Initial position not set, not setting goal");
    return;
  }

  goal(0) = msg->x;
  goal(1) = msg->y;
  goal(2) = msg->z;

  start = pos;
  start_yaw = yaw;

  goal_set = true;
  goal_reached = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_tracker");

  ros::NodeHandle nh("~");

  nh.param("v_des", v_des, 0.5);
  nh.param("a_des", a_des, 0.5);
  nh.param("epsilon", epsilon, 0.1);

  nh.param("gains/pos/x", kx[0], 2.5);
  nh.param("gains/pos/y", kx[1], 2.5);
  nh.param("gains/pos/z", kx[2], 5.0);
  nh.param("gains/vel/x", kv[0], 2.2);
  nh.param("gains/vel/y", kv[1], 2.2);
  nh.param("gains/vel/z", kv[2], 4.0);

  ros::Subscriber sub_odom = nh.subscribe("odom", 10, &odom_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_goal = nh.subscribe("goal", 2, &goal_callback, ros::TransportHints().tcpNoDelay());

  pub_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);

  ros::spin();

  return 0;
}
