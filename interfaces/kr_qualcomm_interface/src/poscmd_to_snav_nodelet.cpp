#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <snav/snapdragon_navigator.h>
#include <std_msgs/Bool.h>

class PosCmdToSnav : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);
  void position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void motors_on();
  void motors_off();

  // controller state
  SnavCachedData *snav_cached_data_struct_;

  bool pos_cmd_set_;

  ros::Subscriber odom_sub_, imu_sub_, position_cmd_sub_, enable_motors_sub_;

  int motor_status_;

  double pos_cmd_timeout_;
  ros::Time last_pos_cmd_time_;
  kr_mav_msgs::PositionCommand last_pos_cmd_;
};

void PosCmdToSnav::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  // Send min thrust as heartbeat
  if(motor_status_ && ((ros::Time::now() - last_pos_cmd_time_).toSec() >= pos_cmd_timeout_))
  {
    sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void PosCmdToSnav::imu_callback(const sensor_msgs::Imu::ConstPtr &pose)
{
  if(pos_cmd_set_ && ((ros::Time::now() - last_pos_cmd_time_).toSec() >= pos_cmd_timeout_))
  {
    ROS_DEBUG("pos_cmd timeout. %f seconds since last command", (ros::Time::now() - last_pos_cmd_time_).toSec());
    const auto last_pos_cmd_ptr = boost::make_shared<kr_mav_msgs::PositionCommand>(last_pos_cmd_);

    position_cmd_callback(last_pos_cmd_ptr);
  }
}

void PosCmdToSnav::motors_on()
{
  // call the update 0 success
  int res_update = sn_update_data();
  if(res_update == -1)
  {
    ROS_ERROR("Likely failure in snav, ensure it is running");
    return;
  }

  switch(snav_cached_data_struct_->general_status.props_state)
  {
    case SN_PROPS_STATE_NOT_SPINNING:
    {
      // sn_send_thrust_att_ang_vel_command (0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      int ret = sn_spin_props();
      if(ret == -1)
        ROS_ERROR("Not able to send spinning command");
      break;
    }
    case SN_PROPS_STATE_STARTING:
    {
      ROS_WARN("Propellers are starting to spin");
      break;
    }
    case SN_PROPS_STATE_SPINNING:
    {
      ROS_INFO("Propellers are spinning");
      motor_status_ = 1;
      break;
    }
    default:
      ROS_ERROR("SN_PROPS_STATE_UNKNOWN");
  }
}

void PosCmdToSnav::motors_off()
{
  do
  {
    // call the update, 0-success
    int res_update = sn_update_data();
    if(res_update == -1)
    {
      ROS_ERROR("Likely failure in snav, ensure it is running");
    }

    // send minimum thrust and identity attitude
    sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // stop the props, 0-success
    int r = sn_stop_props();
    if(r == 0)
      motor_status_ = 0;
    else if(r == -1)
    {
      ROS_ERROR("Not able to send switch off propellers");
    }
  } while(snav_cached_data_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING);

  // check the propellers status
  if(snav_cached_data_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING)
  {
    ROS_ERROR("All the propellers are still spinning");
    motor_status_ = 1;
  }
  else
    ROS_INFO("All the propellers are now off");
}

void PosCmdToSnav::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
  {
    ROS_INFO("Enabling motors");
    for(int i = 0; i < 50; i++)
    {
      sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      if(!motor_status_)
        motors_on();
      else
        return;
      ros::Duration(0.05).sleep();
    }
  }
  else
  {
    ROS_INFO("Disabling motors");
    motors_off();
  }
}

void PosCmdToSnav::position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &pos)
{
  float yaw = static_cast<float>(pos->yaw);
  float yaw_rate = static_cast<float>(pos->yaw_dot);

  float x = static_cast<float>(pos->position.x);
  float y = static_cast<float>(pos->position.y);
  float z = static_cast<float>(pos->position.z);

  float xd = static_cast<float>(pos->velocity.x);
  float yd = static_cast<float>(pos->velocity.y);
  float zd = static_cast<float>(pos->velocity.z);

  float xdd = static_cast<float>(pos->acceleration.x);
  float ydd = static_cast<float>(pos->acceleration.y);
  float zdd = static_cast<float>(pos->acceleration.z);

  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct_) != 0)
  {
    ROS_ERROR("failed to get flight data pointer");
    return;
  }

  int update_ret = sn_update_data();

  if(update_ret != 0)
  {
    ROS_ERROR("detected likely failure in SN, Ensure it is running");
  }
  else
  {
    sn_send_trajectory_tracking_command(SN_POSITION_CONTROL_VIO, SN_TRAJ_DEFAULT, x, y, z, xd, yd, zd, xdd, ydd, zdd,
                                        yaw, yaw_rate);
  }

  if(!pos_cmd_set_)
    pos_cmd_set_ = true;

  // save last pos_cmd
  last_pos_cmd_ = *pos;
  last_pos_cmd_time_ = ros::Time::now();
}

void PosCmdToSnav::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get param for pos command timeout duration
  priv_nh.param("pos_cmd_timeout", pos_cmd_timeout_, 0.25);

  pos_cmd_set_ = false;
  motor_status_ = 0;
  snav_cached_data_struct_ = NULL;

  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct_) != 0)
  {
    ROS_ERROR("Failed to get flight data pointer!");
    return;
  }

  odom_sub_ = priv_nh.subscribe("odom", 10, &PosCmdToSnav::odom_callback, this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &PosCmdToSnav::imu_callback, this, ros::TransportHints().tcpNoDelay());

  position_cmd_sub_ = priv_nh.subscribe("position_cmd", 10, &PosCmdToSnav::position_cmd_callback, this,
                                        ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ =
      priv_nh.subscribe("motors", 2, &PosCmdToSnav::enable_motors_callback, this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PosCmdToSnav, nodelet::Nodelet)
