#include "ros/ros.h"

void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Timer timer1 = n.createTimer(ros::Duration(0.05), callback1);

  ros::spin();

  return 0;
}
