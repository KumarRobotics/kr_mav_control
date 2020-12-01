#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <kr_mav_msgs/MotorRPM.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>
#include <snav/snapdragon_navigator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

class SnavSampler
{
 public:
  SnavSampler(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void rpmTimerCallback(const ros::TimerEvent &event);
  void statusTimerCallback(const ros::TimerEvent &event);
  void attitudeTimerCallback(const ros::TimerEvent &event);

 private:
  bool get_snav_offset();

  SnavCachedData *sn_struct_;
  ros::Duration snav_offset_;

  ros::Publisher motor_speeds_pub_;
  ros::Timer rpm_timer_;

  ros::Publisher battery_pub_;
  ros::Publisher joy_pub_;
  ros::Publisher on_ground_pub_;
  ros::Publisher props_state_pub_;
  ros::Timer status_timer_;

  ros::Publisher attitude_estimate_pub_;
  ros::Timer attitude_estimate_timer_;
};

SnavSampler::SnavSampler(ros::NodeHandle &nh, ros::NodeHandle &pnh) : sn_struct_(NULL)
{
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &sn_struct_) != 0)
    throw "failed to get flight data ptr";
  if(!get_snav_offset())
    throw "unable to obtain snav offset due to update failure";

  float rpm_rate;
  float status_rate;
  float attitude_rate;
  pnh.param<float>("rpm_rate", rpm_rate, 100.0);
  pnh.param<float>("status_rate", status_rate, 5.0);
  pnh.param<float>("attitude_rate", attitude_rate, 100.0);

  if(rpm_rate > 1e-3)
  {
    ROS_INFO("Publish motor_rpm at %4.2fHz", rpm_rate);
    motor_speeds_pub_ = nh.advertise<kr_mav_msgs::MotorRPM>("motor_rpm", 2);
    rpm_timer_ = nh.createTimer(ros::Duration(1.0 / rpm_rate), &SnavSampler::rpmTimerCallback, this);
  }
  if(status_rate > 1e-3)
  {
    ROS_INFO("Publish status (battery, spektrum_joy, on_ground, props_state) at %4.2fHz", status_rate);
    battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery", 2);
    joy_pub_ = nh.advertise<sensor_msgs::Joy>("spektrum_joy", 2);
    on_ground_pub_ = nh.advertise<std_msgs::Bool>("on_ground", 2);
    props_state_pub_ = nh.advertise<std_msgs::String>("props_state", 2);
    status_timer_ = nh.createTimer(ros::Duration(1.0 / status_rate), &SnavSampler::statusTimerCallback, this);
  }
  if(attitude_rate > 1e-3)
  {
    ROS_INFO("Publish attitude estimates at %4.2fHz", attitude_rate);
    attitude_estimate_pub_ = nh.advertise<geometry_msgs::QuaternionStamped>("attitude_estimate", 2);
    attitude_estimate_timer_ =
        nh.createTimer(ros::Duration(1.0 / attitude_rate), &SnavSampler::attitudeTimerCallback, this);
  }
}

bool SnavSampler::get_snav_offset()
{
  if(sn_update_data() != 0)
  {
    ROS_ERROR("snav data retrieval failure");
    return false;
  }
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);

  int64_t gen_timestamp_ns = sn_struct_->general_status.time * 1000;
  ros::Time sntime;
  sntime.fromNSec(gen_timestamp_ns);
  snav_offset_ = realtime - sntime;

  if(realtime < sntime)
    ROS_WARN("snavtime larger than realtime, potential overflow");

  return true;
}

void SnavSampler::attitudeTimerCallback(const ros::TimerEvent &event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    int64_t attitude_timestamp_ns = sn_struct_->attitude_estimate.time * 1000;
    ros::Time attitude_time;
    attitude_time.fromNSec(attitude_timestamp_ns);
    attitude_time += snav_offset_;

    geometry_msgs::QuaternionStamped attitude_msg;
    attitude_msg.header.stamp = attitude_time;
    tf2::Matrix3x3 R(sn_struct_->attitude_estimate.rotation_matrix[0], sn_struct_->attitude_estimate.rotation_matrix[1],
                     sn_struct_->attitude_estimate.rotation_matrix[2], sn_struct_->attitude_estimate.rotation_matrix[3],
                     sn_struct_->attitude_estimate.rotation_matrix[4], sn_struct_->attitude_estimate.rotation_matrix[5],
                     sn_struct_->attitude_estimate.rotation_matrix[6], sn_struct_->attitude_estimate.rotation_matrix[7],
                     sn_struct_->attitude_estimate.rotation_matrix[8]);
    tf2::Quaternion q;
    R.getRotation(q);
    attitude_msg.quaternion.x = q.getX();
    attitude_msg.quaternion.y = q.getY();
    attitude_msg.quaternion.z = q.getZ();
    attitude_msg.quaternion.w = q.getW();
    attitude_estimate_pub_.publish(attitude_msg);
  }
}

void SnavSampler::rpmTimerCallback(const ros::TimerEvent &event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    int64_t esc_timestamp_ns = sn_struct_->esc_raw.time * 1000;
    ros::Time esc_time;
    esc_time.fromNSec(esc_timestamp_ns);
    esc_time += snav_offset_;

    kr_mav_msgs::MotorRPM speed;
    speed.header.stamp = esc_time;
    int16_t *rpm = sn_struct_->esc_raw.rpm;
    speed.motor_count = 4;
    speed.rpm[0] = rpm[0];
    speed.rpm[1] = rpm[1];
    speed.rpm[2] = rpm[2];
    speed.rpm[3] = rpm[3];
    motor_speeds_pub_.publish(speed);
  }
}

void SnavSampler::statusTimerCallback(const ros::TimerEvent &event)
{
  if(sn_update_data() != 0)
    ROS_ERROR("snav data retrieval failure");
  else
  {
    int64_t gen_timestamp_ns = sn_struct_->general_status.time * 1000;
    ros::Time data_time;
    data_time.fromNSec(gen_timestamp_ns);
    data_time += snav_offset_;

    // Battery
    sensor_msgs::BatteryState bat_state;
    bat_state.header.stamp = data_time;
    bat_state.voltage = sn_struct_->general_status.voltage;
    bat_state.current = -sn_struct_->general_status.current;
    battery_pub_.publish(bat_state);

    // Spektrum joy
    int64_t rc_timestamp_ns = sn_struct_->spektrum_rc_0_raw.time * 1000;
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

    // On Ground
    bool on_ground = sn_struct_->general_status.on_ground;
    on_ground_pub_.publish(on_ground);

    // Prop status
    std_msgs::String props_state_msg;
    if((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
    {
      props_state_msg.data = "NOT_SPINNING";
    }
    else if((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_STARTING)
    {
      props_state_msg.data = "STARTING";
    }
    else if((SnPropsState)sn_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING)
    {
      props_state_msg.data = "SPINNING";
    }
    else
    {
      props_state_msg.data = "UNKNOWN";
    }
    props_state_pub_.publish(props_state_msg);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "snav_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    SnavSampler snav_sampler(nh, pnh);
    ros::spin();
  }
  catch(const char *e)
  {
    ROS_ERROR("%s", e);
  }
  return 0;
}
