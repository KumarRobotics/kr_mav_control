#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <snav/snapdragon_navigator.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

class SO3CmdToQualcomm : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);
  void so3_cmd_to_qc_interface(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void motors_on();
  void motors_off();

  // controller state
  SnavCachedData *snav_cached_data_struct_;

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  int motor_status_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;
};

void SO3CmdToQualcomm::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  if(so3_cmd_set_ && ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_DEBUG("so3_cmd timeout. %f seconds since last command", (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr = boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToQualcomm::imu_callback(const sensor_msgs::Imu::ConstPtr &pose)
{
  if(!imu_set_)
    imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);

  if(so3_cmd_set_ && ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_DEBUG("so3_cmd timeout. %f seconds since last command", (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr = boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToQualcomm::motors_on()
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
      sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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

void SO3CmdToQualcomm::motors_off()
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
}

void SO3CmdToQualcomm::so3_cmd_to_qc_interface(const kr_mav_msgs::SO3Command::ConstPtr &msg)
{
  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  const Eigen::Vector3d ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  // convert to tf::Quaternion
  tf::Quaternion imu_tf = tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf::Quaternion odom_tf = tf::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  const Eigen::Matrix3d R_cur(odom_q_);

  double throttle = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  // convert throttle in grams
  throttle = throttle * 1000 / 9.81;

  int res_update = sn_update_data();
  if(res_update == -1)
  {
    ROS_ERROR("Likely failure in snav, ensure it is running");
    return;
  }

  int r = sn_send_thrust_att_ang_vel_command(throttle, q_des.w(), q_des.x(), q_des.y(), q_des.z(), ang_vel(0),
                                             ang_vel(1), ang_vel(2));
  if(r == -1)
    ROS_ERROR("Control command not send");
}

void SO3CmdToQualcomm::so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

  // switch on motors
  if(msg->aux.enable_motors && !motor_status_)
    motors_on();
  else if(!msg->aux.enable_motors)
    motors_off();

  so3_cmd_to_qc_interface(msg);

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = ros::Time::now();
}

void SO3CmdToQualcomm::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.25);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;
  motor_status_ = 0;
  snav_cached_data_struct_ = NULL;

  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct_) != 0)
  {
    ROS_ERROR("Failed to get flight data pointer!");
    return;
  }
  // attitude_raw_pub_ =
  // priv_nh.advertise<mavros_msgs::AttitudeTarget>("attitude_raw", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToQualcomm::so3_cmd_callback, this, ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToQualcomm::odom_callback, this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &SO3CmdToQualcomm::imu_callback, this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToQualcomm, nodelet::Nodelet);
