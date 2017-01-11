#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <so3_control/SO3Control.h>

#define CLAMP(x,min,max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

class SO3TRPYControlNodelet : public nodelet::Nodelet
{
 public:
  SO3TRPYControlNodelet() :
      odom_set_(false),
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_(0),
      des_yaw_dot_(0),
      yaw_int_(0),
      enable_motors_(false),
      use_external_yaw_(false),
      g_(9.81)
  {
    controller_.resetIntegrals();
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishCommand(void);
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);
  void mass_callback(const std_msgs::Float32::ConstPtr &msg);

  SO3Control controller_;
  ros::Publisher trpy_command_pub_;
  ros::Subscriber odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_, mass_sub_;

  bool odom_set_, position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, kx_, kv_, ki_, ki_b_;
  float des_yaw_, des_yaw_dot_, yaw_int_;
  bool enable_motors_, use_external_yaw_;
  float kR_[3], kOm_[3], corrections_[3];
  float ki_yaw_;
  float mass_;
  const float g_;
  Eigen::Quaternionf current_orientation_;
};


void SO3TRPYControlNodelet::publishCommand(void)
{
  if(!odom_set_)
  {
    NODELET_INFO_THROTTLE(1, "Odom not set, not publishing command!");
    return;
  }

  float ki_yaw = 0;
  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f ki_b = Eigen::Vector3f::Zero();
  if(enable_motors_)
  {
    ki_yaw = ki_yaw_;
    ki = ki_;
    ki_b = ki_b_;
  }
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, ki_b);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &q_des = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  const Eigen::Matrix3f R_des(q_des);
  const Eigen::Matrix3f R_cur(current_orientation_);

  const float yaw_cur = std::atan2(R_cur(1,0), R_cur(0,0));
  const float yaw_des = std::atan2(R_des(1,0), R_des(0,0));
  const float pitch_des = -std::asin(R_des(2,0));
  const float roll_des = std::atan2(R_des(2,1), R_des(2,2));

  const float Psi = 0.5f*(3.0f - (R_des(0,0)*R_cur(0,0) + R_des(1,0)*R_cur(1,0) + R_des(2,0)*R_cur(2,0) +
                                  R_des(0,1)*R_cur(0,1) + R_des(1,1)*R_cur(1,1) + R_des(2,1)*R_cur(2,1) +
                                  R_des(0,2)*R_cur(0,2) + R_des(1,2)*R_cur(1,2) + R_des(2,2)*R_cur(2,2)));

  float thrust = 0.0f;
  if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
    thrust = force(0)*R_cur(0,2) + force(1)*R_cur(1,2) + force(2)*R_cur(2,2);

  float e_yaw = yaw_des - yaw_cur;
  if(e_yaw > M_PI)
    e_yaw -= 2*M_PI;
  else if(e_yaw < -M_PI)
    e_yaw += 2*M_PI;

  // Yaw integral
  yaw_int_ += ki_yaw*e_yaw;
  if(yaw_int_ > M_PI)
    yaw_int_ = M_PI;
  else if(yaw_int_ < -M_PI)
    yaw_int_ = -M_PI;

  float yaw_cmd = yaw_des + yaw_int_;
  if(yaw_cmd > M_PI)
    yaw_cmd -= 2*M_PI;
  else if(yaw_cmd < -M_PI)
    yaw_cmd += 2*M_PI;

  quadrotor_msgs::TRPYCommand::Ptr trpy_command(new quadrotor_msgs::TRPYCommand);
  trpy_command->header.stamp = ros::Time::now();
  trpy_command->header.frame_id = frame_id_;
  if(enable_motors_)
  {
    trpy_command->thrust = CLAMP(thrust, 0.01*9.81, 10*9.81);
    trpy_command->roll = roll_des;
    trpy_command->pitch = pitch_des;
    trpy_command->yaw = yaw_cmd;
    trpy_command->angular_velocity.x = ang_vel(0);
    trpy_command->angular_velocity.y = ang_vel(1);
    trpy_command->angular_velocity.z = ang_vel(2);
    for(int i = 0; i < 3; i++)
    {
      trpy_command->kR[i] = kR_[i];
      trpy_command->kOm[i] = kOm_[i];
    }
  }
  trpy_command->aux.current_yaw = yaw_cur;
  trpy_command->aux.kf_correction = corrections_[0];
  trpy_command->aux.angle_corrections[0] = corrections_[1];
  trpy_command->aux.angle_corrections[1] = corrections_[2];
  trpy_command->aux.enable_motors = enable_motors_;
  trpy_command->aux.use_external_yaw = use_external_yaw_;
  trpy_command_pub_.publish(trpy_command);
}

void SO3TRPYControlNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  des_jrk_ = Eigen::Vector3f(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);
  kx_ = Eigen::Vector3f(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3f(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  //position_cmd_init_ = true;

  publishCommand();
}

void SO3TRPYControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  const Eigen::Vector3f position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_orientation_ = Eigen::Quaternionf(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setCurrentOrientation(current_orientation_);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishCommand();
    position_cmd_updated_ = false;
  }
}

void SO3TRPYControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;

  // Reset integrals when toggling motor state
  yaw_int_ = 0;
  controller_.resetIntegrals();
}

void SO3TRPYControlNodelet::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void SO3TRPYControlNodelet::mass_callback(const std_msgs::Float32::ConstPtr &msg)
{
  mass_ = msg->data;
  controller_.setMass(mass_);
  ROS_INFO("so3_trpy_control now using mass: %2.3f kg", mass_);
}

void SO3TRPYControlNodelet::onInit(void)
{
  ros::NodeHandle n(getNodeHandle());
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  std::string quadrotor_name;
  priv_nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.5);
  mass_ = mass;
  controller_.setMass(mass_);
  controller_.setGravity(g_);

  priv_nh.param("use_external_yaw", use_external_yaw_, true);

  double ki_x, ki_y, ki_z, ki_yaw;
  n.param("gains/ki/x", ki_x, 0.0);
  n.param("gains/ki/y", ki_y, 0.0);
  n.param("gains/ki/z", ki_z, 0.0);
  n.param("gains/ki/yaw", ki_yaw, 0.0);
  ki_[0] = ki_x, ki_[1] = ki_y, ki_[2] = ki_z;
  ki_yaw_ = ki_yaw;

  double kR[3], kOm[3];
  n.param("gains/rot/x", kR[0], 1.5);
  n.param("gains/rot/y", kR[1], 1.5);
  n.param("gains/rot/z", kR[2], 1.0);
  n.param("gains/ang/x", kOm[0], 0.13);
  n.param("gains/ang/y", kOm[1], 0.13);
  n.param("gains/ang/z", kOm[2], 0.1);
  kR_[0] = kR[0], kR_[1] = kR[1], kR_[2] = kR[2];
  kOm_[0] = kOm[0], kOm_[1] = kOm[1], kOm_[2] = kOm[2];

  double corrections[3];
  n.param("corrections/kf", corrections[0], 0.0);
  n.param("corrections/r", corrections[1], 0.0);
  n.param("corrections/p", corrections[2], 0.0);
  corrections_[0] = corrections[0], corrections_[1] = corrections[1], corrections_[2] = corrections[2];

  double max_tilt_angle;
  n.param("max_tilt_angle", max_tilt_angle, M_PI);
  controller_.setMaxTiltAngle(max_tilt_angle);

  trpy_command_pub_ = n.advertise<quadrotor_msgs::TRPYCommand>("trpy_cmd", 10);

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3TRPYControlNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = priv_nh.subscribe("position_cmd", 10, &SO3TRPYControlNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = priv_nh.subscribe("motors", 2, &SO3TRPYControlNodelet::enable_motors_callback, this,
                                   ros::TransportHints().tcpNoDelay());
  corrections_sub_ = priv_nh.subscribe("corrections", 10, &SO3TRPYControlNodelet::corrections_callback, this,
                                 ros::TransportHints().tcpNoDelay());
  mass_sub_ = priv_nh.subscribe("set_mass", 10, &SO3TRPYControlNodelet::mass_callback, this,
                                 ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3TRPYControlNodelet, nodelet::Nodelet);
