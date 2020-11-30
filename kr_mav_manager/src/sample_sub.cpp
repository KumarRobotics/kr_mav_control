#include <kr_mav_manager/manager.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

// Services
ros::Subscriber sub_motors_, sub_takeoff_, sub_goTo_, sub_setDesVelInWorldFrame_, sub_setDesVelInBodyFrame_, sub_hover_,
    sub_ehover_, sub_estop_;

// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Subscribers
{
 public:
  // Let's make an MAV
  MAVManager mav_;

  void motors_cb(const std_msgs::Bool &msg) { mav_.set_motors(msg.data); }
  void takeoff_cb(const std_msgs::Empty &msg)
  {
    ROS_INFO("Trying to takeoff");
    if(!mav_.takeoff())
    {
      ROS_ERROR("Takeoff failed");
    }
  }
  void goTo_cb(const geometry_msgs::Pose &msg)
  {
    if(!mav_.goTo(msg.position.x, msg.position.y, msg.position.z, tf::getYaw(msg.orientation)))
    {
      ROS_ERROR("GoTo failed");
    }
  }
  void setDesVelInWorldFrame_cb(const geometry_msgs::Twist &msg)
  {
    mav_.setDesVelInWorldFrame(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
  }
  void setDesVelInBodyFrame_cb(const geometry_msgs::Twist &msg)
  {
    mav_.setDesVelInBodyFrame(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
  }
  void hover_cb(const std_msgs::Empty &msg)
  {
    if(!mav_.hover())
    {
      ROS_ERROR("Hover failed");
    }
  }
  void ehover_cb(const std_msgs::Empty &msg)
  {
    if(!mav_.ehover())
    {
      ROS_ERROR("Ehover failed");
    }
  }
  void estop_cb(const std_msgs::Empty &msg) { mav_.estop(); }
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
  sub_setDesVelInWorldFrame_ =
      nh.subscribe("setDesVelInWorldFrame", 10, &MAV_Subscribers::setDesVelInWorldFrame_cb, &mav_subs);
  sub_setDesVelInBodyFrame_ =
      nh.subscribe("setDesVelInBodyFrame", 10, &MAV_Subscribers::setDesVelInBodyFrame_cb, &mav_subs);
  sub_hover_ = nh.subscribe("hover", 10, &MAV_Subscribers::hover_cb, &mav_subs);
  sub_ehover_ = nh.subscribe("ehover", 10, &MAV_Subscribers::ehover_cb, &mav_subs);
  sub_estop_ = nh.subscribe("estop", 10, &MAV_Subscribers::estop_cb, &mav_subs);

  // Let's spin some rotors
  ros::spin();

  return 0;
}
