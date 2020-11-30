#include <kr_mav_msgs/Serial.h>
#include <kr_serial_interface/ASIOSerialDevice.h>
#include <kr_serial_interface/serial_interface.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

class QuadSerialComm : public nodelet::Nodelet
{
 public:
  void onInit(void);
  ~QuadSerialComm();

 private:
  void serial_callback(const kr_mav_msgs::Serial::ConstPtr &msg);
  void output_data_callback(kr_mav_msgs::Serial &msg);
  void serial_read_callback(const unsigned char *data, size_t count);

  ASIOSerialDevice sd_;
  ros::Publisher output_data_pub_;
  ros::Subscriber serial_sub_;
};

void QuadSerialComm::serial_callback(const kr_mav_msgs::Serial::ConstPtr &msg)
{
  std::vector<unsigned char> serial_msg;

  encode_serial_msg(*msg, serial_msg);

  sd_.Write(serial_msg);
}

void QuadSerialComm::output_data_callback(kr_mav_msgs::Serial &msg)
{
  msg.header.stamp = ros::Time::now();
  output_data_pub_.publish(msg);
}

void QuadSerialComm::serial_read_callback(const unsigned char *data, size_t count)
{
  process_serial_data(data, count, boost::bind(&QuadSerialComm::output_data_callback, this, _1));
}

void QuadSerialComm::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string device;
  n.param("device", device, std::string("/dev/ttyUSB0"));

  int baud_rate;
  n.param("baud_rate", baud_rate, 57600);

  sd_.Open(device, baud_rate);

  output_data_pub_ = n.advertise<kr_mav_msgs::Serial>("from_robot", 10);

  serial_sub_ = n.subscribe("to_robot", 10, &QuadSerialComm::serial_callback, this, ros::TransportHints().tcpNoDelay());

  sd_.SetReadCallback(boost::bind(&QuadSerialComm::serial_read_callback, this, _1, _2));
  sd_.Start();
}

QuadSerialComm::~QuadSerialComm()
{
  sd_.Close();
  sd_.Stop();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(QuadSerialComm, nodelet::Nodelet);
