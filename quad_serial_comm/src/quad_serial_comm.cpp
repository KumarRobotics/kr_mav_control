#include <ros/ros.h>
#include <quadrotor_msgs/Serial.h>
#include <asio_serial_device/ASIOSerialDevice.h>
#include <quad_serial_comm/serial_interface.h>

static ASIOSerialDevice sd;
static ros::Publisher output_data_pub;

static void serial_callback(const quadrotor_msgs::Serial::ConstPtr &msg)
{
  std::vector<unsigned char> serial_msg;

  encode_serial_msg(*msg, serial_msg);

  sd.Write(serial_msg);
}

static void output_data_callback(quadrotor_msgs::Serial &msg)
{
  msg.header.stamp = ros::Time::now();
  output_data_pub.publish(msg);
}

static void serial_read_callback(const unsigned char *data, size_t count)
{
  process_serial_data(data, count, &output_data_callback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_serial_comm");

  ros::NodeHandle n("~");

  std::string device;
  n.param("device", device, std::string("/dev/ttyUSB0"));

  int baud_rate;
  n.param("baud_rate", baud_rate, 57600);

  sd.Open(device, baud_rate);

  ros::Subscriber serial_sub = n.subscribe("to_robot", 10, &serial_callback,
                                           ros::TransportHints().tcpNoDelay());

  output_data_pub = n.advertise<quadrotor_msgs::Serial>("from_robot", 10);

  sd.SetReadCallback(&serial_read_callback);
  sd.Start();

  ros::spin();

  sd.Close();
  sd.Stop();

  return 0;
}
