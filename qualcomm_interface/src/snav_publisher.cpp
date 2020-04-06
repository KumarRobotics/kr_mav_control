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
    throw "snav data retrieval failure";
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);

  unsigned long long stmp(snav_cached_data_struct->general_status.time);
  ros::Time sntime;
  sntime.fromNSec(stmp*1000);
  ros::Duration snav_offset = realtime - sntime;
  if (realtime < sntime)
    throw "snavtime larger than realtime, potential overflow";

  return snav_offset;
}

class rpmSampler
{
public:
  rpmSampler(ros::NodeHandle* nh, ros::NodeHandle* pnh, SnavCachedData *sn_struct);
  ~rpmSampler();
  void rpmTimerCallback(const ros::TimerEvent& event);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  float rpm_rate_;
  ros::Publisher motor_speeds_pub_;
  ros::Timer rpm_timer_;
  SnavCachedData* sn_struct_;
  ros::Duration snav_offset_;
};

rpmSampler::rpmSampler(ros::NodeHandle *nh, ros::NodeHandle *pnh, SnavCachedData* sn_struct)
  : nh_(*nh), pnh_(*pnh), sn_struct_(sn_struct)
{
  pnh_.param<float>("rpm_rate", rpm_rate_, 100.0);
  motor_speeds_pub_ = nh_.advertise<quadrotor_msgs::MotorRPM>("motor_rpm", 2);
  rpm_timer_ = nh_.createTimer(ros::Duration(1.0/rpm_rate_),
                              &rpmSampler::rpmTimerCallback, this);
  try {
    snav_offset_ = get_snav_offset(sn_struct_);
  } catch (const char* e) {
    ROS_ERROR("%s", e);
  }
}

rpmSampler::~rpmSampler()
{
  delete sn_struct_;
}

void rpmSampler::rpmTimerCallback(const ros::TimerEvent& event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    quadrotor_msgs::MotorRPM speed;

    int64_t esc_timestamp_ns = (int64_t) sn_struct_->esc_raw.time * 1000;
    ros::Time esc_time;
    esc_time.fromNSec(esc_timestamp_ns);
    esc_time += snav_offset_;
    speed.header.stamp = esc_time;

    speed.stamp_timer_real = event.current_real;
    speed.stamp_timer_expected = event.current_expected;

    int16_t *rpm = sn_struct_->esc_raw.rpm;
    speed.motor_count = 4;
    speed.rpm[0] = *rpm;
    speed.rpm[1] = *(rpm+1);
    speed.rpm[2] = *(rpm+2);
    speed.rpm[3] = *(rpm+3);
    motor_speeds_pub_.publish(speed);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "snav_status_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  SnavCachedData* snav_cached_data_struct = NULL;
  // It seems there can only be one pointer in snav API? (wenxin)
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct) != 0)
  {
    ROS_ERROR("failed to get flight data ptr");
    return 0;
  }

  rpmSampler rpm(&nh, &pnh, snav_cached_data_struct);
  ros::spin();

  return 0;
}
