#include <iostream>
#include <ros/ros.h>
#include <quadrotor_msgs/MotorRPM.h>
#include <tf/transform_broadcaster.h>
#include <snav/snapdragon_navigator.h>

ros::Duration get_monotonic_offset()
{
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
  return  monotonic_offset;
}

ros::Duration get_snav_offset(SnavCachedData* snav_cached_data_struct)
{
  if(sn_update_data() != 0)
  {
    throw "snav data retrieval failure";
  }
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);

  unsigned long long stmp(snav_cached_data_struct->general_status.time);
  ros::Time sntime;
  sntime.fromNSec(stmp*1000);
  ros::Duration snav_offset = realtime - sntime;

  return snav_offset;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "snav_status_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int rate;
  pnh.param("rate", rate, 100);
  ros::Publisher motor_speeds_pub_ = nh.advertise<quadrotor_msgs::MotorRPM>("motor_rpm", 2);
  ros::Rate loop_rate(rate);

  SnavCachedData* snav_cached_data_struct = NULL;
  // It seems there can only be one pointer in snav API? (wenxin)
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct) != 0)
  {
    ROS_ERROR("failed to get flight data ptr");
    return 0;
  }

  ros::Duration monotonic_offset;
  ros::Duration snav_offset;
  try {
    monotonic_offset = get_monotonic_offset();
    snav_offset = get_snav_offset(snav_cached_data_struct);
  } catch (const char* e) {
    ROS_ERROR("%s", e);
    return 0;
  }
  ROS_INFO_STREAM("Monotonic offset: " << monotonic_offset);
  ROS_INFO_STREAM("Snav offset: " << snav_offset);

  while(ros::ok())
  {
    if(sn_update_data() != 0)
    {
      ROS_ERROR("detected likely failure in snav, ensure it is running");
      continue;
    }
    else
    {
      int64_t esc_timestamp_ns = (int64_t) snav_cached_data_struct->esc_raw.time * 1000;
      ros::Time esc_time;
      esc_time.fromNSec(esc_timestamp_ns);
      esc_time += snav_offset;

      quadrotor_msgs::MotorRPM speed;
      int16_t *rpm = snav_cached_data_struct->esc_raw.rpm;
      speed.header.stamp = esc_time;
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
