#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <snav/snapdragon_navigator.h>

#include <quadrotor_msgs/MotorRPM.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>

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

class snavSampler
{
public:
  snavSampler(ros::NodeHandle* nh, ros::NodeHandle* pnh);
  ~snavSampler();
  void rpmTimerCallback(const ros::TimerEvent& event);
  void statusTimerCallback(const ros::TimerEvent& event);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  SnavCachedData* sn_struct_;
  ros::Duration snav_offset_;
  float rpm_rate_;
  ros::Publisher motor_speeds_pub_;
  ros::Timer rpm_timer_;
  float status_rate_;
  ros::Publisher battery_pub_;
  ros::Publisher joy_pub_;
  ros::Publisher on_ground_pub_;
  ros::Publisher props_state_pub_;
  ros::Timer status_timer_;
};

snavSampler::snavSampler(ros::NodeHandle *nh, ros::NodeHandle *pnh)
  : nh_(*nh), pnh_(*pnh), sn_struct_(NULL)
{
  // It seems there can only be one pointer in snav API? (wenxin)
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &sn_struct_) != 0)
    ROS_ERROR("failed to get flight data ptr");
  try {
    snav_offset_ = get_snav_offset(sn_struct_);
  } catch (const char* e) {
    ROS_ERROR("%s", e);
  }

  pnh_.param<float>("rpm_rate", rpm_rate_, 100.0);
  motor_speeds_pub_ = nh_.advertise<quadrotor_msgs::MotorRPM>("motor_rpm", 2);
  rpm_timer_ = nh_.createTimer(ros::Duration(1.0/rpm_rate_),
                              &snavSampler::rpmTimerCallback, this);
  pnh_.param<float>("status_rate", status_rate_, 5.0);
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("battery", 2);
  joy_pub_ = nh_.advertise<sensor_msgs::Joy>("spektrum_joy", 2);
  on_ground_pub_ = nh_.advertise<std_msgs::Bool>("on_ground", 2);
  props_state_pub_ = nh_.advertise<std_msgs::String>("props_state", 2);
  status_timer_ = nh_.createTimer(ros::Duration(1.0/status_rate_),
                                  &snavSampler::statusTimerCallback, this);
}

snavSampler::~snavSampler()
{
  delete sn_struct_;
}

void snavSampler::rpmTimerCallback(const ros::TimerEvent& event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    int64_t esc_timestamp_ns = (int64_t) sn_struct_->esc_raw.time * 1000;
    ros::Time esc_time;
    esc_time.fromNSec(esc_timestamp_ns);
    esc_time += snav_offset_;

    quadrotor_msgs::MotorRPM speed;
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

void snavSampler::statusTimerCallback(const ros::TimerEvent& event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    int64_t gen_timestamp_ns = (int64_t) sn_struct_->general_status.time * 1000;
    ros::Time data_time;
    data_time.fromNSec(gen_timestamp_ns);
    data_time += snav_offset_;

    //Battery
    sensor_msgs::BatteryState bat_state;
    bat_state.header.stamp = data_time;
    bat_state.voltage = sn_struct_->general_status.voltage;
    bat_state.current = -sn_struct_->general_status.current;
    battery_pub_.publish(bat_state);

    //Spektrum joy
    int64_t rc_timestamp_ns = (int64_t) sn_struct_->spektrum_rc_0_raw.time * 1000;
    ros::Time rc_time;
    rc_time.fromNSec(rc_timestamp_ns);
    rc_time += snav_offset_;

    int32_t rc_status = sn_struct_->data_status.spektrum_rc_0_status;
    uint8_t num_channels = sn_struct_->spektrum_rc_0_raw.num_channels;

    uint8_t max_channels = 16;
    if(rc_status == SN_DATA_VALID && num_channels < max_channels)
    {
      sensor_msgs::Joy joy;
      joy.header.frame_id = "spektrum";
      joy.header.stamp = rc_time;
      joy.axes.reserve(5);
      joy.axes.push_back(sn_struct_->spektrum_rc_0_raw.vals[0]);
      joy.axes.push_back(sn_struct_->spektrum_rc_0_raw.vals[3]);
      joy.axes.push_back(sn_struct_->spektrum_rc_0_raw.vals[1]);
      joy.axes.push_back(sn_struct_->spektrum_rc_0_raw.vals[2]);
      joy.axes.push_back(sn_struct_->spektrum_rc_0_raw.vals[7]);

      joy.buttons.reserve(2);
      joy.buttons.push_back(sn_struct_->spektrum_rc_0_raw.vals[4]);
      joy.buttons.push_back(sn_struct_->spektrum_rc_0_raw.vals[5]);

      joy_pub_.publish(joy);
    }

    //On Ground
    bool on_ground = sn_struct_->general_status.on_ground;
    on_ground_pub_.publish(on_ground);

    //Prop status
    std_msgs::String props_state_msg;
    if ((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_NOT_SPINNING) {
      props_state_msg.data = "NOT_SPINNING";
    }
    else if ((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_STARTING) {
      props_state_msg.data = "STARTING";
    }
    else if ((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING) {
      props_state_msg.data = "SPINNING";
    }
    else {
      props_state_msg.data = "UNKNOWN";
    }
    props_state_pub_.publish(props_state_msg);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "snav_status_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  snavSampler snav_sampler(&nh, &pnh);
  ros::spin();
  return 0;
}
