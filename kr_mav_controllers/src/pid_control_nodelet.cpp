#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/TRPYCommand.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <kr_pid_control/PIDControl.hpp>

#define CLAMP(x, min, max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class PIDControlNodelet : public nodelet::Nodelet
{
 public:
  PIDControlNodelet()
      : position_cmd_updated_(false),
        position_cmd_init_(false),
        des_yaw_(0),
        current_yaw_(0),
        enable_motors_(false),
        use_external_yaw_(false)
  {
  }

  void onInit(void);

 private:
  void publishTRPYCommand(void);
  void position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);

  PIDControl controller_;
  ros::Publisher trpy_command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, kx_, kv_, ki_;
  float des_yaw_;
  float current_yaw_;
  float ki_yaw_;
  float max_roll_pitch_;
  bool enable_motors_;
  bool use_external_yaw_;
};

void PIDControlNodelet::publishTRPYCommand(void)
{
  float ki_yaw = 0;
  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  // Only enable integral terms when motors are on
  if(enable_motors_)
  {
    ki_yaw = ki_yaw_;
    ki = ki_;
  }
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, kx_, kv_, ki, ki_yaw);

  const Eigen::Vector4f &trpy = controller_.getControls();

  kr_mav_msgs::TRPYCommand::Ptr trpy_command(new kr_mav_msgs::TRPYCommand);
  trpy_command->header.stamp = ros::Time::now();
  trpy_command->header.frame_id = frame_id_;
  if(enable_motors_)
  {
    trpy_command->thrust = trpy(0);
    trpy_command->roll = CLAMP(trpy(1), -max_roll_pitch_, max_roll_pitch_);
    trpy_command->pitch = CLAMP(trpy(2), -max_roll_pitch_, max_roll_pitch_);
    trpy_command->yaw = trpy(3);
  }
  trpy_command->aux.current_yaw = current_yaw_;
  trpy_command->aux.enable_motors = enable_motors_;
  trpy_command->aux.use_external_yaw = use_external_yaw_;
  trpy_command_pub_.publish(trpy_command);
}

void PIDControlNodelet::position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3f(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3f(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishTRPYCommand();
}

void PIDControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  const Eigen::Vector3f position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setYaw(current_yaw_);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishTRPYCommand();
    position_cmd_updated_ = false;
  }
}

void PIDControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;

  // Reset integrals when toggling motor state
  controller_.resetIntegrals();
}

void PIDControlNodelet::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.235);
  controller_.setMass(mass);
  controller_.setMaxIntegral(mass * 3);

  n.param("use_external_yaw", use_external_yaw_, true);

  double max_roll_pitch;
  n.param("max_roll_pitch", max_roll_pitch, 30.0);
  max_roll_pitch_ = max_roll_pitch;

  double ki_x, ki_y, ki_z, ki_yaw;
  n.param("gains/ki/x", ki_x, 0.0);
  n.param("gains/ki/y", ki_y, 0.0);
  n.param("gains/ki/z", ki_z, 0.0);
  n.param("gains/ki/yaw", ki_yaw, 0.01);
  ki_[0] = ki_x, ki_[1] = ki_y, ki_[2] = ki_z;
  ki_yaw_ = ki_yaw;

  trpy_command_pub_ = n.advertise<kr_mav_msgs::TRPYCommand>("trpy_cmd", 10);

  odom_sub_ = n.subscribe("odom", 10, &PIDControlNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = n.subscribe("position_cmd", 10, &PIDControlNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ =
      n.subscribe("motors", 2, &PIDControlNodelet::enable_motors_callback, this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PIDControlNodelet, nodelet::Nodelet);
