#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <snav/snapdragon_navigator.h>

void pos_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &pos)
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

  SnavCachedData* snav_data = NULL;
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data)!=0)
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
    if(snav_data->general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
      sn_spin_props();
    sn_send_trajectory_tracking_command(SN_POSITION_CONTROL_VIO, SN_TRAJ_DEFAULT, x, y, z, xd, yd, zd, xdd, ydd, zdd, yaw, yaw_rate);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_cmd_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("position_cmd", 10, pos_cmd_callback);
  ros::spin();
  return 0;
}
