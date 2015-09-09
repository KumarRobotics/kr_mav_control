#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mav_manager/manager.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

// Services
ros::Subscriber
  sub_motors_,
  sub_takeoff_,
  sub_goTo_,
  sub_setDesVelWorld_,
  sub_setDesVelBody_,
  sub_hover_,
  sub_ehover_,
  sub_estop_;

// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Subscribers
{
  public:

  // Let's make an MAV
  MAVManager mav_;

  void motors_cb(const std_msgs::Bool &msg)
  {
    mav_.set_motors(msg.data);
  }
  void takeoff_cb(const std_msgs::Empty &msg)
  {
    ROS_INFO("Trying to takeoff");
    if (!mav_.takeoff())
    {
      ROS_ERROR("Takeoff failed");
    }
  }
  void goTo_cb(const geometry_msgs::Pose &msg)
  {
    Vec4 goal(msg.position.x, msg.position.y, msg.position.z, tf::getYaw(msg.orientation));
    if (!mav_.goTo(goal))
    {
      ROS_ERROR("GoTo failed");
    }
  }
  void setDesVelWorld_cb(const geometry_msgs::Twist &msg)
  {
    Vec4 goal(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
    mav_.setDesVelWorld(goal);
  }
  void setDesVelBody_cb(const geometry_msgs::Twist &msg)
  {
    Vec4 goal(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
    mav_.setDesVelBody(goal);
  }
  void hover_cb(const std_msgs::Empty &msg)
  {
    if (!mav_.hover())
    {
      ROS_ERROR("Hover failed");
    }
  }
  void ehover_cb(const std_msgs::Empty &msg)
  {
    if (!mav_.ehover())
    {
      ROS_ERROR("Ehover failed");
    }
  }
  void estop_cb(const std_msgs::Empty &msg)
  {
    mav_.estop();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh("~");

  MAV_Subscribers mav_subs;

  // Services
  sub_motors_ = nh.subscribe("motors", 10, &MAV_Subscribers::motors_cb, &mav_subs);
  sub_takeoff_ = nh.subscribe("takeoff", 10, &MAV_Subscribers::takeoff_cb, &mav_subs);
  sub_goTo_ = nh.subscribe("goTo", 10, &MAV_Subscribers::goTo_cb, &mav_subs);
  sub_setDesVelWorld_ = nh.subscribe("setDesVelWorld", 10, &MAV_Subscribers::setDesVelWorld_cb, &mav_subs);
  sub_setDesVelBody_ = nh.subscribe("setDesVelBody", 10, &MAV_Subscribers::setDesVelBody_cb, &mav_subs);
  sub_hover_ = nh.subscribe("hover", 10, &MAV_Subscribers::hover_cb, &mav_subs);
  sub_ehover_ = nh.subscribe("ehover", 10, &MAV_Subscribers::ehover_cb, &mav_subs);
  sub_estop_ = nh.subscribe("estop", 10, &MAV_Subscribers::estop_cb, &mav_subs);

  // Let's spin some rotors
  ros::spin();

  return 0;
}
