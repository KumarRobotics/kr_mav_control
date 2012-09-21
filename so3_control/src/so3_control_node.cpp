#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <so3_control/SO3Control.h>
#include <tf/transform_datatypes.h>

static SO3Control controller;
static ros::Publisher so3_command_pub;
static quadrotor_msgs::SO3Command so3_command;
static bool position_cmd_updated = false, position_cmd_init = false;
static Eigen::Vector3d des_pos, des_vel, des_acc, kx, kv;
static double des_yaw = 0, des_yaw_dot = 0;
static double current_yaw = 0;
static bool enable_motors = false;
static bool use_external_yaw = true;
static bool use_angle_corrections = false;
static double corrections[3];

static void publishSO3Command(void)
{
  controller.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot,
                              kx, kv);

  const Eigen::Vector3d &force = controller.getComputedForce();
  const Eigen::Quaterniond &orientation = controller.getComputedOrientation();

  so3_command.header.stamp = ros::Time::now();
  so3_command.force.x = force(0);
  so3_command.force.y = force(1);
  so3_command.force.z = force(2);
  so3_command.orientation.x = orientation.x();
  so3_command.orientation.y = orientation.y();
  so3_command.orientation.z = orientation.z();
  so3_command.orientation.w = orientation.w();
  so3_command.aux.current_yaw = current_yaw;
  so3_command.aux.corrections[0] = corrections[0];
  so3_command.aux.corrections[1] = corrections[1];
  so3_command.aux.corrections[2] = corrections[2];
  so3_command.aux.enable_motors = enable_motors;
  so3_command.aux.use_external_yaw = use_external_yaw;
  so3_command.aux.use_angle_corrections = use_angle_corrections;
  so3_command_pub.publish(so3_command);
}

static void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                            cmd->acceleration.z);
  kx = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw = cmd->yaw;
  des_yaw_dot = cmd->yaw_dot;
  position_cmd_updated = true;
  //position_cmd_init = true;

  publishSO3Command();
}

static void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw = tf::getYaw(odom->pose.pose.orientation);

  controller.setPosition(position);
  controller.setVelocity(velocity);

  if(position_cmd_init)
  {
    // We set position_cmd_updated = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    if(!position_cmd_updated)
      publishSO3Command();
    position_cmd_updated = false;
  }
}

static void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors = msg->data;
}

static void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections[0] = msg->corrections[0];
  corrections[1] = msg->corrections[1];
  corrections[2] = msg->corrections[2];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3_control");

  ros::NodeHandle n("~");

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  so3_command.header.frame_id = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.5);
  controller.setMass(mass);

  n.param("use_external_yaw", use_external_yaw, true);
  n.param("use_angle_corrections", use_angle_corrections, false);

  XmlRpc::XmlRpcValue gains_rot, gains_ang;
  n.getParam("gains/rot", gains_rot);
  n.getParam("gains/ang", gains_ang);
  if(gains_rot.getType() != XmlRpc::XmlRpcValue::TypeArray ||
     gains_ang.getType() != XmlRpc::XmlRpcValue::TypeArray ||
     gains_rot.size() != 3 || gains_ang.size() != 3)
  {
    ROS_FATAL("%s: Error parsing gains", ros::this_node::getName().c_str());
    n.shutdown();
    return -1;
  }
  for(int i = 0; i < 3; i++)
  {
    so3_command.kR[i] = static_cast<double>(gains_rot[i]);
    so3_command.kOm[i] = static_cast<double>(gains_ang[i]);
  }

  n.param("corrections/z", corrections[0], 0.0);
  n.param("corrections/r", corrections[1], 0.0);
  n.param("corrections/p", corrections[2], 0.0);

  ros::Subscriber odom_sub = n.subscribe("odom", 10, &odom_callback,
                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber position_cmd_sub = n.subscribe("position_cmd", 10, &position_cmd_callback,
                                                 ros::TransportHints().tcpNoDelay());

  ros::Subscriber enable_motors_sub = n.subscribe("motors", 2, &enable_motors_callback,
                                                  ros::TransportHints().tcpNoDelay());
  ros::Subscriber corrections_sub = n.subscribe("corrections", 10, &corrections_callback,
                                                ros::TransportHints().tcpNoDelay());

  so3_command_pub = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  ros::spin();

  return 0;
}
