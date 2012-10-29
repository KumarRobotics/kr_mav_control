#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <so3_control/SO3Control.h>
#include <tf/transform_datatypes.h>

class SO3ControlNodelet : public nodelet::Nodelet
{
 public:
  SO3ControlNodelet() :
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_(0),
      des_yaw_dot_(0),
      current_yaw_(0),
      enable_motors_(false),
      use_external_yaw_(false),
      use_angle_corrections_(false)
  {
  }

  void onInit(void);

 private:
  void publishSO3Command(void);
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);

  SO3Control controller_;
  ros::Publisher so3_command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber corrections_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double des_yaw_, des_yaw_dot_;
  double current_yaw_;
  bool enable_motors_;
  bool use_external_yaw_;
  bool use_angle_corrections_;
  double kR_[3], kOm_[3], corrections_[3];
};


void SO3ControlNodelet::publishSO3Command(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_,
                              kx_, kv_);

  const Eigen::Vector3d &force = controller_.getComputedForce();
  const Eigen::Quaterniond &orientation = controller_.getComputedOrientation();

  quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
  so3_command->header.stamp = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x = force(0);
  so3_command->force.y = force(1);
  so3_command->force.z = force(2);
  so3_command->orientation.x = orientation.x();
  so3_command->orientation.y = orientation.y();
  so3_command->orientation.z = orientation.z();
  so3_command->orientation.w = orientation.w();
  for(int i = 0; i < 3; i++)
  {
    so3_command->kR[i] = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw = current_yaw_;
  so3_command->aux.corrections[0] = corrections_[0];
  so3_command->aux.corrections[1] = corrections_[1];
  so3_command->aux.corrections[2] = corrections_[2];
  so3_command->aux.enable_motors = enable_motors_;
  so3_command->aux.use_external_yaw = use_external_yaw_;
  so3_command->aux.use_angle_corrections = use_angle_corrections_;
  so3_command_pub_.publish(so3_command);
}

void SO3ControlNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                            cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  //position_cmd_init_ = true;

  publishSO3Command();
}

void SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }
}

void SO3ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void SO3ControlNodelet::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->corrections[0];
  corrections_[1] = msg->corrections[1];
  corrections_[2] = msg->corrections[2];
}

void SO3ControlNodelet::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.5);
  controller_.setMass(mass);

  n.param("use_external_yaw", use_external_yaw_, true);
  n.param("use_angle_corrections", use_angle_corrections_, false);

  XmlRpc::XmlRpcValue gains_rot, gains_ang;
  n.getParam("gains/rot", gains_rot);
  n.getParam("gains/ang", gains_ang);
  if(gains_rot.getType() != XmlRpc::XmlRpcValue::TypeArray ||
     gains_ang.getType() != XmlRpc::XmlRpcValue::TypeArray ||
     gains_rot.size() != 3 || gains_ang.size() != 3)
  {
    ROS_FATAL("%s: Error parsing gains", ros::this_node::getName().c_str());
    n.shutdown();
    return;
  }
  for(int i = 0; i < 3; i++)
  {
    kR_[i] = static_cast<double>(gains_rot[i]);
    kOm_[i] = static_cast<double>(gains_ang[i]);
  }

  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  odom_sub_ = n.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = n.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ = n.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                                   ros::TransportHints().tcpNoDelay());
  corrections_sub_ = n.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback, this,
                                 ros::TransportHints().tcpNoDelay());

  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet, nodelet::Nodelet);
