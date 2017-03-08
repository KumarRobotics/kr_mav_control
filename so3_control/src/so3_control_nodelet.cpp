#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
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
      current_orientation_(Eigen::Quaternionf::Identity()),
      enable_motors_(false),
      use_external_yaw_(false),
      have_odom_(false),
      g_(9.81)
  {
    controller_.resetIntegrals();
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command(void);
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);
  void mass_callback(const std_msgs::Float32::ConstPtr &msg);
  void cfg_callback(so3_control::SO3Config &config, uint32_t level);

  SO3Control controller_;
  ros::Publisher so3_command_pub_;
  ros::Subscriber odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_, mass_sub_;

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

  // Dynamic Reconfigure Setup
  boost::recursive_mutex config_mutex_;
  typedef so3_control::SO3Config Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
};


void SO3ControlNodelet::publishSO3Command(void)
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
}

void SO3ControlNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
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

  publishSO3Command();
}

void SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

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

void SO3ControlNodelet::mass_callback(const std_msgs::Float32::ConstPtr &msg)
{
  mass_ = msg->data;
  controller_.setMass(mass_);
  ROS_INFO("so3_control now using mass: %2.3f kg", mass_);
}

void SO3ControlNodelet::cfg_callback(so3_control::SO3Config &config, uint32_t level)
{
  ki_[0]  = config.ki_x;
  ki_[1]  = config.ki_y;
  ki_[2]  = config.ki_z;
  controller_.setMaxIntegral(config.max_pos_int);

  kib_[0] = config.kib_x;
  kib_[1] = config.kib_y;
  kib_[2] = config.kib_z;
  controller_.setMaxIntegralBody(config.max_pos_int_b);
  
  controller_.setMaxTiltAngle(config.max_tilt_angle);

  ROS_INFO("\nso3_control reconfigure Request:\n  ki:  {%2.4f, %2.4f, %2.4f}\n  kib: {%2.4f, %2.4f, %2.4f}\n  max_pos_int:   %2.2f\n  max_pos_int_b: %2.2f\n  max_tilt_angle (rad): %2.2f",
      ki_[0], ki_[1], ki_[2],
      kib_[0], kib_[1], kib_[2],
      config.max_pos_int,
      config.max_pos_int_b,
      config.max_tilt_angle);
}

void SO3ControlNodelet::onInit(void)
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

  // Dynamic reconfigure struct
  Config config;

  double ki_x, ki_y, ki_z;
  n.param("gains/ki/x", ki_x, 0.0);
  n.param("gains/ki/y", ki_y, 0.0);
  n.param("gains/ki/z", ki_z, 0.0);
  config.ki_x = ki_x; config.ki_y = ki_y; config.ki_z = ki_z;

  double kib_x, kib_y, kib_z;
  n.param("gains/kib/x", kib_x, 0.0);
  n.param("gains/kib/y", kib_y, 0.0);
  n.param("gains/kib/z", kib_z, 0.0);
  config.kib_x = kib_x; config.kib_y = kib_y; config.kib_z = kib_z;

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

  float max_pos_int, max_pos_int_b;
  n.param("max_pos_int", max_pos_int, 0.0f);
  n.param("max_pos_int_b", max_pos_int_b, 0.0f);
  config.max_pos_int = max_pos_int;
  config.max_pos_int_b = max_pos_int_b;
    
  double max_tilt_angle;
  n.param("max_tilt_angle", max_tilt_angle, M_PI);
  config.max_tilt_angle = max_tilt_angle;
  
  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, priv_nh));
  reconfigure_server_->updateConfig(config);
  ReconfigureServer::CallbackType f = boost::bind(&SO3ControlNodelet::cfg_callback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Publisher
  so3_command_pub_ = priv_nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  // Subscribers
  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = priv_nh.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = priv_nh.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                                   ros::TransportHints().tcpNoDelay());
  corrections_sub_ = priv_nh.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback, this,
                                 ros::TransportHints().tcpNoDelay());
  mass_sub_ = priv_nh.subscribe("set_mass", 10, &SO3ControlNodelet::mass_callback, this,
                                 ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
