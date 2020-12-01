#include <kr_mav_msgs/Serial.h>
#include <kr_serial_interface/decode_msgs.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class QuadDecodeMsg : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void serial_callback(const kr_mav_msgs::Serial::ConstPtr &msg);
  ros::Publisher output_data_pub_, imu_output_pub_, status_pub_;
  ros::Subscriber serial_sub_;
};

void QuadDecodeMsg::serial_callback(const kr_mav_msgs::Serial::ConstPtr &msg)
{
  if(msg->type == kr_mav_msgs::Serial::OUTPUT_DATA)
  {
    kr_mav_msgs::OutputData::Ptr output_msg(new kr_mav_msgs::OutputData);
    sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);

    if(kr_mav_msgs::decodeOutputData(msg->data, *output_msg))
    {
      output_msg->header.stamp = msg->header.stamp;
      output_msg->header.frame_id = "/quadrotor";
      output_data_pub_.publish(output_msg);

      imu_msg->header = output_msg->header;
      imu_msg->orientation = output_msg->orientation;
      imu_msg->angular_velocity = output_msg->angular_velocity;
      imu_msg->linear_acceleration = output_msg->linear_acceleration;
      imu_output_pub_.publish(imu_msg);
    }
  }
  else if(msg->type == kr_mav_msgs::Serial::STATUS_DATA)
  {
    kr_mav_msgs::StatusData::Ptr status_msg(new kr_mav_msgs::StatusData);
    if(kr_mav_msgs::decodeStatusData(msg->data, *status_msg))
    {
      status_msg->header.stamp = msg->header.stamp;
      status_msg->header.frame_id = "/quadrotor";
      status_pub_.publish(status_msg);
    }
  }
}

void QuadDecodeMsg::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  output_data_pub_ = priv_nh.advertise<kr_mav_msgs::OutputData>("output_data", 10);
  imu_output_pub_ = priv_nh.advertise<sensor_msgs::Imu>("imu", 10);
  status_pub_ = priv_nh.advertise<kr_mav_msgs::StatusData>("status", 10);

  serial_sub_ =
      priv_nh.subscribe("serial", 10, &QuadDecodeMsg::serial_callback, this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(QuadDecodeMsg, nodelet::Nodelet);
