#ifndef STD_TRACKERS_RADIO_TRACKER_H_
#define STD_TRACKERS_RADIO_TRACKER_H_

#include <ros/ros.h>
#include <trackers_manager/Tracker.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <quadrotor_msgs/OutputData.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <array>

class Radio
{
  public:

    ros::Time last_update_t;

    int x_channel,
        y_channel,
        z_channel,
        yaw_channel,
        n_channels;

    bool params_set;

    // These should eventually be set as params
    float ch_min,
          ch_max,
          ch_mid,
          ch_range,
          deadband_val;

    void updateFromMsg(const quadrotor_msgs::OutputData::ConstPtr &msg);

    float channel(int ch);

    float x()   {return deadband(channel(x_channel));}
    float y()   {return deadband(channel(y_channel));}
    float z()   {return channel(z_channel);}
    float yaw() {return deadband(channel(yaw_channel));}

    Radio() :
      last_update_t(0.0),
      x_channel(  0),
      y_channel(  1),
      z_channel(  2),
      yaw_channel(3),
      params_set(false),
      ch_min(0.0),
      ch_max(255.0)
  {
    ch_mid = (ch_max - ch_min) / 2.0;
    ch_range = (ch_max - ch_min);
    deadband_val = 2.0 / ch_range;
  }

  private:

    std::vector<uint8_t> channel_values;
    float deadband(float val);
};

void Radio::updateFromMsg(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  last_update_t = ros::Time::now();

  n_channels = msg->radio_channel.size();
  channel_values.resize(n_channels);

  for (unsigned int i = 0; i < channel_values.size(); i++)
    channel_values[i] = msg->radio_channel[i];
}

float Radio::channel(int ch)
{
  if (ch <= n_channels)
    return ((float)channel_values[ch] - ch_mid) / (ch_range / 2.0);
  else
    return 0.0;
}

float Radio::deadband(float val) {
  if (val > deadband_val)
    return val - deadband_val;
  else if (val < -deadband_val)
    return val + deadband_val;
  else
    return 0.0;
}

//////////////////////
//////////////////////
//////////////////////

// TODO: Combine this into the radio class above, and instantiate as an object (not inherit) in the trackers
namespace std_trackers
{
  class RadioTracker
  {
    public:

      void StartOutputDataSubscriber(const ros::NodeHandle &nh);
      void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg);

      Radio radio;

    private:

      ros::Subscriber output_data_sub_;
  };

  void RadioTracker::StartOutputDataSubscriber(const ros::NodeHandle &nh)
  {
    std::array<bool,4> flags = {{false, false, false, false}};
    flags[0] = nh.getParam("radio/x_channel_index",   radio.x_channel);
    flags[1] = nh.getParam("radio/y_channel_index",   radio.y_channel);
    flags[2] = nh.getParam("radio/z_channel_index",   radio.z_channel);
    flags[3] = nh.getParam("radio/yaw_channel_index", radio.yaw_channel);

    bool flag = true;
    for (uint8_t i=0; i < flags.size(); i++) {
      flag = flag && flags[i];
    }
    radio.params_set = flag;

    ros::NodeHandle priv_nh(nh);
    output_data_sub_ = priv_nh.subscribe("output_data", 1, &RadioTracker::output_data_cb, this, ros::TransportHints().tcpNoDelay());
  }

  void RadioTracker::output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg) {
    radio.updateFromMsg(msg);
  }
}

#endif // STD_TRACKERS_RADIO_TRACKER_H_
