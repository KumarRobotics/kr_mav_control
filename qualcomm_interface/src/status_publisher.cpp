#include <iostream>
#include <ros/ros.h>
#include <quadrotor_msgs/MotorRPM.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <snav/snapdragon_navigator.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "status_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int rate;
  pnh.param("rate", rate, 5);
  uint8_t max_channels = 16;
  ros::Publisher battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery", 2);
  ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("spektrum_joy", 2);
  ros::Publisher on_ground_pub = nh.advertise<std_msgs::Bool>("on_ground", 2);
  ros::Publisher props_state_pub = nh.advertise<std_msgs::String>("props_state", 2);

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
      int64_t gen_timestamp_ns = (int64_t) snav_cached_data_struct->general_status.time * 1000;
      ros::Time data_time;
      data_time.fromNSec(gen_timestamp_ns);
      data_time += monotonic_offset;

      //Battery
      sensor_msgs::BatteryState bat_state;
      bat_state.header.stamp = data_time;
      bat_state.voltage = snav_cached_data_struct->general_status.voltage;
      bat_state.current = -snav_cached_data_struct->general_status.current;
      battery_pub_.publish(bat_state);

      //Spektrum joy
      int64_t rc_timestamp_ns = (int64_t) snav_cached_data_struct->spektrum_rc_0_raw.time * 1000;
      ros::Time rc_time;
      rc_time.fromNSec(rc_timestamp_ns);
      rc_time += monotonic_offset;

      int32_t rc_status = snav_cached_data_struct->data_status.spektrum_rc_0_status;
      uint8_t num_channels = snav_cached_data_struct->spektrum_rc_0_raw.num_channels;

      if(rc_status == SN_DATA_VALID && num_channels < max_channels)
      {
        sensor_msgs::Joy joy;
        joy.header.frame_id = "spektrum";
        joy.header.stamp = rc_time;
        joy.axes.reserve(5);
        joy.axes.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[0]);
        joy.axes.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[3]);
        joy.axes.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[1]);
        joy.axes.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[2]);
        joy.axes.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[7]);

        joy.buttons.reserve(2);
        joy.buttons.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[4]);
        joy.buttons.push_back(snav_cached_data_struct->spektrum_rc_0_raw.vals[5]);

        joy_pub.publish(joy);
      }

      //On Ground
      bool on_ground = snav_cached_data_struct->general_status.on_ground;
      on_ground_pub.publish(on_ground);

      //Prop status
      std_msgs::String props_state_msg;
      if ((SnPropsState)snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_NOT_SPINNING) {
        props_state_msg.data = "NOT_SPINNING";
      }
      else if ((SnPropsState)snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_STARTING) {
        props_state_msg.data = "STARTING";
      }
      else if ((SnPropsState)snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_SPINNING) {
        props_state_msg.data = "SPINNING";
      }
      else {
        props_state_msg.data = "UNKNOWN";
      }
      props_state_pub.publish(props_state_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}