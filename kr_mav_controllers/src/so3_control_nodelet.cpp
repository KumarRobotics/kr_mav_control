#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <kr_mav_controllers/SO3Config.h>
#include <kr_mav_controllers/SO3Control.h>
#include <kr_mav_msgs/Corrections.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

class SO3ControlNodelet : public nodelet::Nodelet
{
 public:
  SO3ControlNodelet()
      : position_cmd_updated_(false),
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command();
  void position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const kr_mav_msgs::Corrections::ConstPtr &msg);
  void cfg_callback(kr_mav_controllers::SO3Config &config, uint32_t level);

  SO3Control controller_;
  ros::Publisher so3_command_pub_, command_viz_pub_;
  ros::Subscriber odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, config_kx_, config_kv_, config_ki_, config_kib_, kx_, kv_;
  float des_yaw_, des_yaw_dot_;
  float current_yaw_;
  bool enable_motors_, use_external_yaw_, have_odom_;
  float kR_[3], kOm_[3], corrections_[3];
  float mass_;
  const float g_;
  Eigen::Quaternionf current_orientation_;

  boost::recursive_mutex config_mutex_;
  typedef dynamic_reconfigure::Server<kr_mav_controllers::SO3Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
};

void SO3ControlNodelet::publishSO3Command()
{
  if(!have_odom_)
  {
    ROS_WARN("No odometry! Not publishing SO3Command.");
    return;
  }

  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f kib = Eigen::Vector3f::Zero();
  if(enable_motors_)
  {
    ki = config_ki_;
    kib = config_kib_;
  }

  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &orientation = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  kr_mav_msgs::SO3Command::Ptr so3_command = boost::make_shared<kr_mav_msgs::SO3Command>();
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

  geometry_msgs::PoseStamped::Ptr cmd_viz_msg = boost::make_shared<geometry_msgs::PoseStamped>();
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

void SO3ControlNodelet::position_cmd_callback(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3f(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3f(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  des_jrk_ = Eigen::Vector3f(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);

  // Check use_msg_gains_flag to decide whether to use gains from the msg or config
  kx_[0] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_X) ? cmd->kx[0] : config_kx_[0];
  kx_[1] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_Y) ? cmd->kx[1] : config_kx_[1];
  kx_[2] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_POSITION_Z) ? cmd->kx[2] : config_kx_[2];
  kv_[0] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_X) ? cmd->kv[0] : config_kv_[0];
  kv_[1] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_Y) ? cmd->kv[1] : config_kv_[1];
  kv_[2] = (cmd->use_msg_gains_flags & cmd->USE_MSG_GAINS_VELOCITY_Z) ? cmd->kv[2] : config_kv_[2];

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishSO3Command();
}

void SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

  frame_id_ = odom->header.frame_id;

  const Eigen::Vector3f position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  const Eigen::Vector3f velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

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

void SO3ControlNodelet::corrections_callback(const kr_mav_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlNodelet::cfg_callback(kr_mav_controllers::SO3Config &config, uint32_t level)
{
  if(level == 0)
  {
    NODELET_DEBUG_STREAM("Nothing changed. level: " << level);
    return;
  }

  if(level & (1 << 0))
  {
    config_kx_[0] = config.kp_x;
    config_kx_[1] = config.kp_y;
    config_kx_[2] = config.kp_z;

    config_kv_[0] = config.kd_x;
    config_kv_[1] = config.kd_y;
    config_kv_[2] = config.kd_z;

    NODELET_INFO("Position Gains set to kp: {%2.3g, %2.3g, %2.3g}, kd: {%2.3g, %2.3g, %2.3g}", config_kx_[0],
                 config_kx_[1], config_kx_[2], config_kv_[0], config_kv_[1], config_kv_[2]);
  }

  if(level & (1 << 1))
  {
    config_ki_[0] = config.ki_x;
    config_ki_[1] = config.ki_y;
    config_ki_[2] = config.ki_z;

    config_kib_[0] = config.kib_x;
    config_kib_[1] = config.kib_y;
    config_kib_[2] = config.kib_z;

    NODELET_INFO("Integral Gains set to ki: {%2.2g, %2.2g, %2.2g}, kib: {%2.2g, %2.2g, %2.2g}", config_ki_[0],
                 config_ki_[1], config_ki_[2], config_kib_[0], config_kib_[1], config_kib_[2]);
  }

  if(level & (1 << 2))
  {
    kR_[0] = config.rot_x;
    kR_[1] = config.rot_y;
    kR_[2] = config.rot_z;

    kOm_[0] = config.ang_x;
    kOm_[1] = config.ang_y;
    kOm_[2] = config.ang_z;

    NODELET_INFO("Attitude Gains set to kp: {%2.2g, %2.2g, %2.2g}, kd: {%2.2g, %2.2g, %2.2g}", kR_[0], kR_[1], kR_[2],
                 kOm_[0], kOm_[1], kOm_[2]);
  }

  if(level & (1 << 3))
  {
    corrections_[0] = config.kf_correction;
    corrections_[1] = config.roll_correction;
    corrections_[2] = config.pitch_correction;
    NODELET_INFO("Corrections set to kf: %2.2g, roll: %2.2g, pitch: %2.2g", corrections_[0], corrections_[1],
                 corrections_[2]);
  }

  if(level & (1 << 4))
  {
    controller_.setMaxIntegral(config.max_pos_int);
    controller_.setMaxIntegralBody(config.max_pos_int_b);
    controller_.setMaxTiltAngle(config.max_tilt_angle);

    NODELET_INFO("Maxes set to Integral: %2.2g, Integral Body: %2.2g, Tilt Angle (rad): %2.2g", config.max_pos_int,
                 config.max_pos_int_b, config.max_tilt_angle);
  }

  NODELET_WARN_STREAM_COND(level != std::numeric_limits<uint32_t>::max() && (level >= (1 << 5)),
                           "kr_mav_controllers dynamic reconfigure called, but with unknown level: " << level);
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
  kr_mav_controllers::SO3Config config;

  priv_nh.param("use_external_yaw", use_external_yaw_, true);

  priv_nh.param("gains/pos/x", config_kx_[0], 7.4f);
  priv_nh.param("gains/pos/y", config_kx_[1], 7.4f);
  priv_nh.param("gains/pos/z", config_kx_[2], 10.4f);
  kx_[0] = config_kx_[0];
  kx_[1] = config_kx_[1];
  kx_[2] = config_kx_[2];
  config.kp_x = config_kx_[0];
  config.kp_y = config_kx_[1];
  config.kp_z = config_kx_[2];

  priv_nh.param("gains/vel/x", config_kv_[0], 4.8f);
  priv_nh.param("gains/vel/y", config_kv_[1], 4.8f);
  priv_nh.param("gains/vel/z", config_kv_[2], 6.0f);
  kv_[0] = config_kv_[0];
  kv_[1] = config_kv_[1];
  kv_[2] = config_kv_[2];
  config.kd_x = config_kv_[0];
  config.kd_y = config_kv_[1];
  config.kd_z = config_kv_[2];

  priv_nh.param("gains/ki/x", config_ki_[0], 0.0f);
  priv_nh.param("gains/ki/y", config_ki_[1], 0.0f);
  priv_nh.param("gains/ki/z", config_ki_[2], 0.0f);
  config.ki_x = config_ki_[0];
  config.ki_y = config_ki_[1];
  config.ki_z = config_ki_[2];

  priv_nh.param("gains/kib/x", config_kib_[0], 0.0f);
  priv_nh.param("gains/kib/y", config_kib_[1], 0.0f);
  priv_nh.param("gains/kib/z", config_kib_[2], 0.0f);
  config.kib_x = config_kib_[0];
  config.kib_y = config_kib_[1];
  config.kib_z = config_kib_[2];

  priv_nh.param("gains/rot/x", kR_[0], 1.5f);
  priv_nh.param("gains/rot/y", kR_[1], 1.5f);
  priv_nh.param("gains/rot/z", kR_[2], 1.0f);
  priv_nh.param("gains/ang/x", kOm_[0], 0.13f);
  priv_nh.param("gains/ang/y", kOm_[1], 0.13f);
  priv_nh.param("gains/ang/z", kOm_[2], 0.1f);
  config.rot_x = kR_[0];
  config.rot_y = kR_[1];
  config.rot_z = kR_[2];
  config.ang_x = kOm_[0];
  config.ang_y = kOm_[1];
  config.ang_z = kOm_[2];

  priv_nh.param("corrections/kf", corrections_[0], 0.0f);
  priv_nh.param("corrections/r", corrections_[1], 0.0f);
  priv_nh.param("corrections/p", corrections_[2], 0.0f);
  config.kf_correction = corrections_[0];
  config.roll_correction = corrections_[1];
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
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(config_mutex_, priv_nh);
  reconfigure_server_->updateConfig(config);
  reconfigure_server_->setCallback(boost::bind(&SO3ControlNodelet::cfg_callback, this, _1, _2));

  so3_command_pub_ = priv_nh.advertise<kr_mav_msgs::SO3Command>("so3_cmd", 10);
  command_viz_pub_ = priv_nh.advertise<geometry_msgs::PoseStamped>("cmd_viz", 10);

  odom_sub_ =
      priv_nh.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = priv_nh.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback, this,
                                        ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = priv_nh.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                                         ros::TransportHints().tcpNoDelay());
  corrections_sub_ = priv_nh.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback, this,
                                       ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
