#include <iostream>
#include <ros/ros.h>
#include <quadrotor_msgs/MotorRPM.h>
#include <tf/transform_broadcaster.h>
#include <snav/snapdragon_navigator.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motor_rpm_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int rate;
  pnh.param("rate", rate, 100);
  ros::Publisher motor_speeds_pub_ = nh.advertise<quadrotor_msgs::MotorRPM>("motor_rpm", 2);
  ros::Rate loop_rate(rate);

  SnavCachedData* snav_cached_data_struct = NULL;

  // Snav/Imu samples are timestamped with the monotonic clock
  // ROS timestamps use the realtime clock.  Compute the difference and apply to messages
  // Note: timestamp_in_us is apps monotonic clock, raw_timestamp_in_us is DSP monotonic clock
  // Here, we use timestamp_in_us
  struct timespec time_monotonic;
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  clock_gettime(CLOCK_MONOTONIC, &time_monotonic);

  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);
  ros::Time monotonic(time_monotonic.tv_sec, time_monotonic.tv_nsec);

  ros::Duration monotonic_offset = realtime - monotonic;

  ROS_INFO_STREAM("Monotonic offset: " << monotonic_offset);

  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct) != 0)
  {
    ROS_ERROR("failed to get flight data ptr");
    return 0;
  }
  if(sn_update_data() != 0)
  {
    ROS_ERROR("detected likely failure in snav, ensure it is running");
    return 0;
  }

  unsigned long stmp(snav_cached_data_struct->general_status.time);
  ros::Time sntime;
  sntime.fromNSec(stmp*1000);
  ros::Duration snav_offset = realtime - sntime;

  ROS_INFO_STREAM("Snav offset: " << snav_offset);

  while(ros::ok())
  {
    //read the flight data
    if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct) != 0)
    {
      ROS_ERROR("failed to get flight data ptr");
      continue;
    }
    if(sn_update_data() != 0)
    {
      ROS_ERROR("detected likely failure in snav, ensure it is running");
      continue;
    }
    else
    {
      unsigned long stamp(snav_cached_data_struct->general_status.time);
      ros::Time data_time;
      data_time.fromNSec(stamp*1000);
      data_time += monotonic_offset;

      quadrotor_msgs::MotorRPM speed;
      int16_t *rpm = snav_cached_data_struct->esc_raw.rpm;
      speed.header.stamp = data_time;
      speed.motor_count = 4;
      speed.rpm[0] = *rpm;
      speed.rpm[1] = *(rpm+1);
      speed.rpm[2] = *(rpm+2);
      speed.rpm[3] = *(rpm+3);
      motor_speeds_pub_.publish(speed);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}
