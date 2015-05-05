#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  MAVManager mav;

  ros::spin();

  return 0;
}
