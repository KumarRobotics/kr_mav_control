#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <snav/snapdragon_navigator.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vio_odom_publisher");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Rate loop_rate(100);

  int update_ret;
  float roll, pitch, yaw;
  while(ros::ok())
  {
    // read the flight data
    SnavCachedData *snav_data = NULL;
    if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
    {
      ROS_ERROR("failed to get flight data ptr");
      continue;
    }
    update_ret = sn_update_data();
    if(update_ret != 0)
    {
      ROS_ERROR("detected likely failure in snav, ensure it is running");
      continue;
    }
    else
    {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";

      // set the position
      odom.pose.pose.position.x = snav_data->vio_pos_vel.position_estimated[0];
      odom.pose.pose.position.y = snav_data->vio_pos_vel.position_estimated[1];
      odom.pose.pose.position.z = snav_data->vio_pos_vel.position_estimated[2];
      // set the attitude
      roll = snav_data->attitude_estimate.roll;
      pitch = snav_data->attitude_estimate.pitch;
      yaw = snav_data->attitude_estimate.yaw;
      // convert to quaternion
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      odom.pose.pose.orientation = odom_quat;

      // set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = snav_data->vio_pos_vel.velocity_estimated[0];
      odom.twist.twist.linear.y = snav_data->vio_pos_vel.velocity_estimated[1];
      odom.twist.twist.linear.z = snav_data->vio_pos_vel.velocity_estimated[2];

      // publish the message
      odom_pub.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}