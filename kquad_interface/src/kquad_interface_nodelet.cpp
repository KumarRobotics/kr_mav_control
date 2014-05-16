#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <kQuadInterface.hh>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <list>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

class kQuadInterfaceNodelet : public nodelet::Nodelet
{
 public:
  ~kQuadInterfaceNodelet(void);
  void onInit(void);

 private:
  void trpy_command_cb(int id, int type, int channel, const quadrotor_msgs::TRPYCommand::ConstPtr &msg);
  void output_thread(void);

  kQuadInterface kqi_;

  int num_robots_;
  std::vector<ros::Subscriber> trpy_command_sub_;
  std::vector<ros::Publisher> output_pub_, imu_pub_, status_pub_;
  std::map<int, int> id_to_index_;

  volatile bool output_thread_running_;
  boost::shared_ptr<boost::thread> output_thread_ptr_;
};

void kQuadInterfaceNodelet::onInit(void)
{
  ros::NodeHandle n(getMTPrivateNodeHandle());

  std::string device;
  int baud_rate;
  n.param("serial_device", device, std::string("/dev/ttyUSB0"));
  n.param("serial_baud_rate", baud_rate, 921600);

  XmlRpc::XmlRpcValue quad_id_list, quad_type_list, quad_channel_list;
  if(!n.hasParam("ids") || !n.hasParam("types") || !n.hasParam("channels"))
  {
    ROS_ERROR("ids, types and/or channels params are not set");
    return;
  }
  n.getParam("ids", quad_id_list);
  n.getParam("types", quad_type_list);
  n.getParam("channels", quad_channel_list);
  ROS_ASSERT(quad_id_list.getType() == quad_type_list.getType() == quad_channel_list.getType() ==
             XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(quad_id_list.size() == quad_type_list.size() == quad_channel_list.size());

  num_robots_ = quad_id_list.size();
  ROS_INFO("num_robots: %d", num_robots_);
  for(int i = 0; i < num_robots_; i++)
  {
    std::string base_topic = "robot" + boost::lexical_cast<std::string>(i+1);
    output_pub_.push_back(n.advertise<quadrotor_msgs::OutputData>(base_topic + "/output", 10));
    imu_pub_.push_back(n.advertise<sensor_msgs::Imu>(base_topic + "/imu", 10));
    status_pub_.push_back(n.advertise<geometry_msgs::Vector3Stamped>(base_topic+"/status", 10));

    ROS_ASSERT(quad_id_list[i].getType() == quad_type_list[i].getType() == quad_channel_list[i].getType() ==
               XmlRpc::XmlRpcValue::TypeInt);
    const int id = static_cast<int>(quad_id_list[i]);
    const int type = static_cast<int>(quad_type_list[i]);
    const int channel = static_cast<int>(quad_channel_list[i]);
    ROS_INFO("id: %d, type: %d, channel: %d", id, type, channel);
    id_to_index_[id] = i;
    trpy_command_sub_.push_back(n.subscribe<quadrotor_msgs::TRPYCommand>(base_topic+"/trpy_command", 10, boost::bind(
                &kQuadInterfaceNodelet::trpy_command_cb, this, id, type, channel, _1), ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay()));
  }

  if(kqi_.Connect(device.c_str(), baud_rate))
  {
    NODELET_FATAL("could not connect to the device");
  }

  if (kqi_.StartSendThread())
  {
    NODELET_FATAL("could not start send thread");
  }

  if (kqi_.StartRecvThread())
  {
    NODELET_FATAL("could not start receive thread");
  }

  // spawn data output thread
  output_thread_running_ = true;
  output_thread_ptr_ = boost::shared_ptr<boost::thread>(new boost::thread(
          boost::bind(&kQuadInterfaceNodelet::output_thread, this)));
}

void kQuadInterfaceNodelet::trpy_command_cb(int id, int type, int channel, const quadrotor_msgs::TRPYCommand::ConstPtr &msg)
{
  kqi_.SendQuadCmd1(id, type, channel, msg->thrust*1000/9.81f, msg->roll, msg->pitch, msg->yaw);
  //kqi_.SendQuadCmd3(id, type, channel, msg->thrust*1000/9.81f, msg->roll, msg->pitch, msg->yaw,
  //                  msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->kR[0], msg->kR[1],
  //                  msg->kR[2], msg->kOm[0], msg->kOm[1], msg->kOm[2]);
  //kqi_.SendQuadCmd4(id, type, channel, msg->thrust*1000/9.81f, msg->roll, msg->pitch, msg->yaw,
  //                  msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->kR[0], msg->kR[1],
  //                  msg->kR[2], msg->kOm[0], msg->kOm[1], msg->kOm[2], 0, 0, 0);
}

void kQuadInterfaceNodelet::output_thread(void)
{
  uint8_t rc_channels[8] = {127,127,127,127,127,127,127,127};
  while(output_thread_running_)
  {
    std::list<ImuFiltData> if_data;
    std::list<RcData> rc_data_list;
    int nmsg = kqi_.GetImuFiltData(if_data, 1);
    int nmsg_rc = kqi_.GetRcData(rc_data_list, 1);
    unsigned int rc_channels_id = 0;
    if(nmsg_rc > 0)
    {
      RcData &rc_data = rc_data_list.front();
      rc_channels_id = rc_data.id;
      for(unsigned int i = 0; i < 8; i++)
      {
        rc_channels[i] = rc_data.data[i]/4;
      }
    }
    if(nmsg > 0)
    {
      ImuFiltData &imu_data = if_data.front();
      NODELET_INFO("imu_data.id: %u, tpc: %f, az: %f", imu_data.id, imu_data.tpc, imu_data.az);
      quadrotor_msgs::OutputData::Ptr output_msg(new quadrotor_msgs::OutputData);
      output_msg->header.stamp = ros::Time(imu_data.tpc);
      output_msg->header.frame_id = "/quadrotor" + boost::lexical_cast<std::string>(imu_data.id);

      const float roll = imu_data.roll, pitch = imu_data.pitch, yaw = imu_data.yaw;
      output_msg->orientation.x = sinf(roll/2)*cosf(pitch/2)*cosf(yaw/2) - cosf(roll/2)*sinf(pitch/2)*sinf(yaw/2);
      output_msg->orientation.y = cosf(roll/2)*sinf(pitch/2)*cosf(yaw/2) + sinf(roll/2)*cosf(pitch/2)*sinf(yaw/2);
      output_msg->orientation.z = cosf(roll/2)*cosf(pitch/2)*sinf(yaw/2) - sinf(roll/2)*sinf(pitch/2)*cosf(yaw/2);
      output_msg->orientation.w = cosf(roll/2)*cosf(pitch/2)*cosf(yaw/2) + sinf(roll/2)*sinf(pitch/2)*sinf(yaw/2);

      output_msg->angular_velocity.x = imu_data.wroll;
      output_msg->angular_velocity.y = imu_data.wpitch;
      output_msg->angular_velocity.z = imu_data.wyaw;

      output_msg->linear_acceleration.x = imu_data.ax * 9.81;
      output_msg->linear_acceleration.y = imu_data.ay * 9.81;
      output_msg->linear_acceleration.z = imu_data.az * 9.81;

      if(rc_channels_id == imu_data.id)
      {
        for(unsigned int i = 0; i < 8; i++)
          output_msg->radio_channel[i] = rc_channels[i];
      }

      const std::map<int, int>::const_iterator id_to_index_it = id_to_index_.find(imu_data.id);
      const int idx = id_to_index_it->second;
      output_pub_[idx].publish(output_msg);

      sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
      imu_msg->header.stamp = ros::Time(imu_data.tpc);
      imu_msg->header.frame_id = "/quadrotor" + boost::lexical_cast<std::string>(imu_data.id);

      imu_msg->orientation.x = output_msg->orientation.x;
      imu_msg->orientation.y = output_msg->orientation.y;
      imu_msg->orientation.z = output_msg->orientation.z;
      imu_msg->orientation.w = output_msg->orientation.w;

      imu_msg->angular_velocity.x = imu_data.wroll;
      imu_msg->angular_velocity.y = imu_data.wpitch;
      imu_msg->angular_velocity.z = imu_data.wyaw;

      imu_msg->linear_acceleration.x = imu_data.ax * 9.81;
      imu_msg->linear_acceleration.y = imu_data.ay * 9.81;
      imu_msg->linear_acceleration.z = imu_data.az * 9.81;

      imu_pub_[idx].publish(imu_msg);
    }

    std::list<QuadStatusData> status_list;
    int nmsg_status = kqi_.GetQuadStatusData(status_list, 1);
    if(nmsg_status > 0)
    {
      QuadStatusData &status_data = status_list.front();
      geometry_msgs::Vector3Stamped::Ptr status_msg(new geometry_msgs::Vector3Stamped);
      status_msg->header.stamp = ros::Time(status_data.tpc);
      status_msg->header.frame_id = "/quadrotor" + boost::lexical_cast<std::string>(status_data.id);
      status_msg->vector.x = status_data.voltage;
      status_msg->vector.y = status_data.current;
      const std::map<int, int>::const_iterator id_to_index_it = id_to_index_.find(status_data.id);
      const int idx = id_to_index_it->second;
      status_pub_[idx].publish(status_msg);
    }

    //boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    usleep(2000);
  }
}

kQuadInterfaceNodelet::~kQuadInterfaceNodelet(void)
{
  if(output_thread_running_)
  {
    output_thread_running_ = false;
    output_thread_ptr_->join();
  }
  output_pub_.clear();
  status_pub_.clear();
  imu_pub_.clear();
  trpy_command_sub_.clear();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(kquad_interface, kQuadInterfaceNodelet, kQuadInterfaceNodelet, nodelet::Nodelet);
