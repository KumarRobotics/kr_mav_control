#ifndef ATTITUDE_TUNING_TOOL_H
#define ATTITUDE_TUNING_TOOL_H

// Standard C++
#include <Eigen/Geometry>

// ROS related
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>

class AttitudeTuningTool
{
  public:

    AttitudeTuningTool();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Publisher pub_sp_as_pose_msg_, pub_thrust_vector_error_;
    ros::Subscriber odom_sub_, pose_sub_, setpoint_sub_;

    // Note: these should be in the same frame
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

    // The setpoint quaternion
    Eigen::Quaternionf sp_q_;
};

#endif /* ATTITUDE_TUNING_TOOL_H */
