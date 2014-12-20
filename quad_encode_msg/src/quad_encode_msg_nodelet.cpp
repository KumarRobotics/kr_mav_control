#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/PWMCommand.h>
#include <quadrotor_msgs/encode_msgs.h>
#include <quadrotor_msgs/Serial.h>

class QuadEncodeMsg : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &msg);
  void trpy_cmd_callback(const quadrotor_msgs::TRPYCommand::ConstPtr &msg);
  void pwm_cmd_callback(const quadrotor_msgs::PWMCommand::ConstPtr &msg);
  ros::Publisher serial_msg_pub_;
  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber trpy_cmd_sub_;
  ros::Subscriber pwm_cmd_sub_;
  int channel_;

};

void QuadEncodeMsg::so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &msg)
{
  quadrotor_msgs::Serial::Ptr serial_msg(new quadrotor_msgs::Serial);
  serial_msg->header.seq = msg->header.seq;
  serial_msg->channel = channel_;
  serial_msg->type = quadrotor_msgs::Serial::SO3_CMD;

  quadrotor_msgs::encodeSO3Command(*msg, serial_msg->data);

  serial_msg->header.stamp = ros::Time::now();
  serial_msg_pub_.publish(serial_msg);
}

void QuadEncodeMsg::trpy_cmd_callback(const quadrotor_msgs::TRPYCommand::ConstPtr &msg)
{
  quadrotor_msgs::Serial::Ptr serial_msg(new quadrotor_msgs::Serial);
  serial_msg->header.seq = msg->header.seq;
  serial_msg->channel = channel_;
  serial_msg->type = quadrotor_msgs::Serial::TRPY_CMD;

  quadrotor_msgs::encodeTRPYCommand(*msg, serial_msg->data);

  serial_msg->header.stamp = ros::Time::now();
  serial_msg_pub_.publish(serial_msg);
}

void QuadEncodeMsg::pwm_cmd_callback(const quadrotor_msgs::PWMCommand::ConstPtr &msg)
{
  quadrotor_msgs::Serial::Ptr serial_msg(new quadrotor_msgs::Serial);
  serial_msg->header.seq = msg->header.seq;
  serial_msg->channel = channel_;
  serial_msg->type = quadrotor_msgs::Serial::PWM_CMD;
  quadrotor_msgs::encodePWMCommand(*msg, serial_msg->data);

  serial_msg->header.stamp = ros::Time::now();
  serial_msg_pub_.publish(serial_msg);
}

void QuadEncodeMsg::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  priv_nh.param("channel", channel_, 0);

  so3_cmd_sub_ = priv_nh.subscribe("so3_cmd", 10, &QuadEncodeMsg::so3_cmd_callback, this,
                                                  ros::TransportHints().tcpNoDelay());
  trpy_cmd_sub_ = priv_nh.subscribe("trpy_cmd", 10, &QuadEncodeMsg::trpy_cmd_callback, this,
                                    ros::TransportHints().tcpNoDelay());
  pwm_cmd_sub_ = priv_nh.subscribe("pwm_cmd", 10, &QuadEncodeMsg::pwm_cmd_callback, this,
                                                  ros::TransportHints().tcpNoDelay());

  serial_msg_pub_ = priv_nh.advertise<quadrotor_msgs::Serial>("serial_msg", 10);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(QuadEncodeMsg, nodelet::Nodelet);
