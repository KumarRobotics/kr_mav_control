#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include "snapdragon_navigator.h"


class SO3CmdToQualcomm : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);
  void motors_on();
  void motors_off();

  //controller state
  SnavCachedData *snav_cached_data_struct;
  
  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;

  //ros::Publisher attitude_raw_pub_;

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;


  int motor_status;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  quadrotor_msgs::SO3Command last_so3_cmd_;
};

void SO3CmdToQualcomm::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
}

void SO3CmdToQualcomm::imu_callback(const sensor_msgs::Imu::ConstPtr &pose)
{
  if(!imu_set_)
    imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x,
                              pose->orientation.y, pose->orientation.z);

  if(so3_cmd_set_ &&
     ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_INFO("so3_cmd timeout. %f seconds since last command",
             (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr =
        boost::make_shared<quadrotor_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToQualcomm::motors_on(){


     int counter_motors = 0;
		do{
		//call the update 0 success
		int res_update = sn_update_data();
		if(res_update ==  -1){
		printf("\nINIT 1: flight software non functional\n");
		return;
		}
		//send minimum thrust and identity attitude
		sn_send_thrust_att_ang_vel_command (0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		
		//run the props 0 success
		//int r = sn_spin_props();
		//if(r == -1)
		  //printf("\nINIT 2: not able to send spinning command\n");
		
		//controller state
		//int size_cached_struct;
		counter_motors++;

		}
		while(counter_motors > 100);//snav_cached_data_struct->general_status.props_state != SN_PROPS_STATE_SPINNING);
		
		//sn_send_thrust_att_ang_vel_command (0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		int r = sn_spin_props();
		if(r == -1)
		  printf("\nINIT 2: not able to send spinning command\n");
		else
			motor_status = 1;
		  
		if(snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_SPINNING)
		  printf("\nINIT 5: all the propellers are spinnig\n");
		else
		  printf("\nINIT 5: all the propellers are not spinnig\n");

}


void SO3CmdToQualcomm::motors_off(){

		do{
		//call the update 0 success
		int res_update = sn_update_data();
	      	//stop the props 0 success
		int r = sn_stop_props();
		if(r == -1)
		  printf("\nOUT 1: not able to send switch off propellers\n");
		//send minimum thrust and identity attitude
		sn_send_thrust_att_ang_vel_command (0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		motor_status = 0;
		}
		while(snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_SPINNING);
		
		//check the propellers status
		if(snav_cached_data_struct->general_status.props_state == SN_PROPS_STATE_SPINNING)
		  printf("\nOUT 2: all the propellers are still spinnig\n");
		else
		  printf("\nOUT 2: all the propellers are now off\n");

}

void SO3CmdToQualcomm::so3_cmd_callback(
    const quadrotor_msgs::SO3Command::ConstPtr &msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

//switch on motors
  if(msg->aux.enable_motors && !motor_status)
  	motors_on();
  else if(!msg->aux.enable_motors)
  	motors_off();

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);
  const Eigen::Vector3d ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  // convert to tf::Quaternion
  tf::Quaternion imu_tf =
      tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf::Quaternion odom_tf =
      tf::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());


  // extract RPY's
  //double imu_roll, imu_pitch, imu_yaw;
  //double odom_roll, odom_pitch, odom_yaw;
  //tf::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  //tf::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);

  // create only yaw tf:Quaternions
  //tf::Quaternion imu_tf_yaw;
  //tf::Quaternion odom_tf_yaw;
  //imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  //odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  //const tf::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // transform!
  //Eigen::Quaterniond q_des_transformed =
  //    Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(),
   //                      tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) *
   //   q_des;

  // check psi for stability
  //const Eigen::Matrix3d R_des(q_des_transformed);
  const Eigen::Matrix3d R_cur(odom_q_);

  //const float Psi =
  //    0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) +
  //                    R_des(2, 0) * R_cur(2, 0) + R_des(0, 1) * R_cur(0, 1) +
  //                    R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
  //                    R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) +
   //                   R_des(2, 2) * R_cur(2, 2)));

  double throttle = 0.0;
  //if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
  //{
    throttle = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) +
               f_des(2) * R_cur(2, 2);

  //convert throttle in grams
  throttle = throttle*1000/9.81;             
  //}
  //else
  //{
    //ROS_WARN_THROTTLE(1,"psi > 1.0, throttle set to 0.0 in mavros_interface.");
  //}

  // Scale force to be proportional to rotor velocity (rad/s).
  // Note: This ignores the number of propellers.
  //throttle = std::sqrt(throttle / kf_);

  // Scaling from proportional rotor velocity (rad/s) to att_throttle for pixhawk
  //throttle = lin_cof_a_ * throttle + lin_int_b_;

  // clamp from 0.0 to 1.0
  //throttle = std::min(1.0, throttle);
  //throttle = std::max(0.0, throttle);

  //if(!msg->aux.enable_motors)
    //throttle = 0;


  // publish messages
  //auto setpoint_msg = boost::make_shared<mavros_msgs::AttitudeTarget>();
  //setpoint_msg->header = msg->header;
  //setpoint_msg->type_mask = 0;
  /*setpoint_msg->orientation.w = q_des_transformed.w();
  setpoint_msg->orientation.x = q_des_transformed.x();
  setpoint_msg->orientation.y = q_des_transformed.y();
  setpoint_msg->orientation.z = q_des_transformed.z();
  setpoint_msg->body_rate.x = msg->angular_velocity.x;
  setpoint_msg->body_rate.y = msg->angular_velocity.y;
  setpoint_msg->body_rate.z = msg->angular_velocity.z;
  setpoint_msg->thrust = throttle;*/

  //attitude_raw_pub_.publish(setpoint_msg);
  //send attitude to the Qualcomm robot
  //call the API to update the internal cache of flight 0 success
  int res_update = sn_update_data();
  if(res_update ==  -1){
  printf("\nTRAJ 1: flight software non functional\n");
   return;
  }


  int r = sn_send_thrust_att_ang_vel_command (throttle, q_des.w(), q_des.x(), q_des.y(), q_des.z(), ang_vel(0), ang_vel(1), ang_vel(2));
  if(r == -1)
  printf("\nTRAJ 2: control command not send\n");

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
   motor_status = 0;
	snav_cached_data_struct = NULL;
if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return;
  }
  //attitude_raw_pub_ =
      //priv_nh.advertise<mavros_msgs::AttitudeTarget>("attitude_raw", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToQualcomm::so3_cmd_callback, this,
                        ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToQualcomm::odom_callback,
                                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &SO3CmdToQualcomm::imu_callback, this,
                               ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToQualcomm, nodelet::Nodelet);
