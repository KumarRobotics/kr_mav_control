#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <so3_control/SO3Control.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <so3_control/SO3Config.h>

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
      have_odom_(false),
      g_(9.81),
      current_orientation_(Eigen::Quaternionf::Identity())
  {
    controller_.resetIntegrals();
  }

  void onInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command();
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);
  void cfg_callback(so3_control::SO3Config &config, uint32_t level);

  SO3Control controller_;
  ros::Publisher so3_command_pub_, command_viz_pub_;
  ros::Subscriber odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, kx_, kv_, ki_, kib_;
  float des_yaw_, des_yaw_dot_;
  float current_yaw_;
  bool enable_motors_, use_external_yaw_, have_odom_;
  float kR_[3], kOm_[3], corrections_[3];
  float mass_;
  const float g_;
  Eigen::Quaternionf current_orientation_;

  boost::recursive_mutex config_mutex_;
  typedef so3_control::SO3Config Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
};


void SO3ControlNodelet::publishSO3Command()
{
  if (!have_odom_)
  {
    ROS_WARN("No odometry! Not publishing SO3Command.");
    return;
  }

  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f kib = Eigen::Vector3f::Zero();
  if(enable_motors_)
  {
    ki = ki_;
    kib = kib_;
  }

  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &orientation = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  quadrotor_msgs::SO3Command::Ptr so3_command =
      boost::make_shared<quadrotor_msgs::SO3Command>();
  so3_command->header.stamp = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x = force(0);
  so3_command->force.y = force(1);
  so3_command->force.z = force(2);
  so3_command->orientation.x = orientation.x();
  so3_command->orientation.y = orientation.y();
  so3_command->orientation.z = orientation.z();
  so3_command->orientation.w = orientation.w();
  so3_command->angular_velocity.x = ang_vel(0);
  so3_command->angular_velocity.y = ang_vel(1);
  so3_command->angular_velocity.z = ang_vel(2);
  for(int i = 0; i < 3; i++)
  {
    so3_command->kR[i] = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw = current_yaw_;
  so3_command->aux.kf_correction = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors = enable_motors_;
  so3_command->aux.use_external_yaw = use_external_yaw_;
  so3_command_pub_.publish(so3_command);

  geometry_msgs::PoseStamped::Ptr cmd_viz_msg =
      boost::make_shared<geometry_msgs::PoseStamped>();
  cmd_viz_msg->header = so3_command->header;
  cmd_viz_msg->pose.position.x = des_pos_(0);
  cmd_viz_msg->pose.position.y = des_pos_(1);
  cmd_viz_msg->pose.position.z = des_pos_(2);
  cmd_viz_msg->pose.orientation.x = orientation.x();
  cmd_viz_msg->pose.orientation.y = orientation.y();
  cmd_viz_msg->pose.orientation.z = orientation.z();
  cmd_viz_msg->pose.orientation.w = orientation.w();
  command_viz_pub_.publish(cmd_viz_msg);
}

void SO3ControlNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  des_jrk_ = Eigen::Vector3f(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);

  // If we want to set new gains
  if (cmd->use_gains_flag == 2)
  {
    // TODO: Update config server with new config
  }

  // Use these gains temporarily
  if (cmd->use_gains_flag == 1)
  {
    // TODO:
    // kx_ = Eigen::Vector3f(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
    // kv_ = Eigen::Vector3f(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  //position_cmd_init_ = true;

  publishSO3Command();
}

void SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

  frame_id_ = odom->header.frame_id;

  const Eigen::Vector3f position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

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
  // Reset integral when toggling motor state
  controller_.resetIntegrals();
}

void SO3ControlNodelet::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlNodelet::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  std::string quadrotor_name;
  priv_nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  priv_nh.param("mass", mass_, 0.5f);
  controller_.setMass(mass_);
  controller_.setGravity(g_);

  // Dynamic reconfigure struct
  so3_control::SO3Config config;

  priv_nh.param("use_external_yaw", use_external_yaw_, true);

  priv_nh.param("gains/pos/x", kx_[0],  7.4f);
  priv_nh.param("gains/pos/y", kx_[1],  7.4f);
  priv_nh.param("gains/pos/z", kx_[2], 10.4f);
  config.kp_x = kx_[0]; config.kp_y = kx_[1]; config.kp_z = kx_[2];

  priv_nh.param("gains/vel/x", kv_[0],  4.8f);
  priv_nh.param("gains/vel/y", kv_[1],  4.8f);
  priv_nh.param("gains/vel/z", kv_[2],  6.0f);
  config.kd_x = kv_[0]; config.kd_y = kv_[1]; config.kd_z = kv_[2];

  priv_nh.param("gains/ki/x", ki_[0], 0.0f);
  priv_nh.param("gains/ki/y", ki_[1], 0.0f);
  priv_nh.param("gains/ki/z", ki_[2], 0.0f);
  config.ki_x = ki_[0]; config.ki_y = ki_[1]; config.ki_z = ki_[2];

  priv_nh.param("gains/kib/x", kib_[0], 0.0f);
  priv_nh.param("gains/kib/y", kib_[1], 0.0f);
  priv_nh.param("gains/kib/z", kib_[2], 0.0f);
  config.kib_x = kib_[0]; config.kib_y = kib_[1]; config.kib_z = kib_[2];

  priv_nh.param("gains/rot/x", kR_[0], 1.5f);
  priv_nh.param("gains/rot/y", kR_[1], 1.5f);
  priv_nh.param("gains/rot/z", kR_[2], 1.0f);
  priv_nh.param("gains/ang/x", kOm_[0], 0.13f);
  priv_nh.param("gains/ang/y", kOm_[1], 0.13f);
  priv_nh.param("gains/ang/z", kOm_[2], 0.1f);
  config.rot_x = kR_[0];  config.rot_y = kR_[1];  config.rot_z =  kR_[2];
  config.ang_x = kOm_[0]; config.ang_y = kOm_[1]; config.ang_z = kOm_[2];

  priv_nh.param("corrections/kf", corrections_[0], 0.0f);
  priv_nh.param("corrections/r", corrections_[1], 0.0f);
  priv_nh.param("corrections/p", corrections_[2], 0.0f);
  config.kf_correction    = corrections_[0];
  config.roll_correction  = corrections_[1];
  config.pitch_correction = corrections_[2];

  float max_pos_int, max_pos_int_b;
  priv_nh.param("max_pos_int", max_pos_int, 0.5f);
  priv_nh.param("mas_pos_int_b", max_pos_int_b, 0.5f);
  controller_.setMaxIntegral(max_pos_int);
  controller_.setMaxIntegralBody(max_pos_int_b);
  config.max_pos_int = max_pos_int;
  config.max_pos_int_b = max_pos_int_b;

  float max_tilt_angle;
  priv_nh.param("max_tilt_angle", max_tilt_angle, static_cast<float>(M_PI));
  controller_.setMaxTiltAngle(max_tilt_angle);
  config.max_tilt_angle = max_tilt_angle;

  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, priv_nh));
  reconfigure_server_->updateConfig(config);
  ReconfigureServer::CallbackType f = boost::bind(&SO3ControlNodelet::cfg_callback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  so3_command_pub_ = priv_nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  command_viz_pub_ = priv_nh.advertise<geometry_msgs::PoseStamped>("cmd_viz", 10);

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = priv_nh.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = priv_nh.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                                   ros::TransportHints().tcpNoDelay());
  corrections_sub_ = priv_nh.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback, this,
                                 ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
