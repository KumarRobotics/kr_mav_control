#ifndef QUADROTOR_SIMULATOR_CPP_
#define QUADROTOR_SIMULATOR_CPP_

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <kr_mav_msgs/msg/output_data.hpp>
#include <kr_mav_msgs/msg/so3_command.hpp>
#include <kr_quadrotor_simulator_humble/Quadrotor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h> 

#include <Eigen/Geometry>
#include <memory>
#include <Eigen/Geometry>

namespace QuadrotorSimulator
{
typedef struct _SO3Command
{
  float force[3];
  float qx, qy, qz, qw;
  float angular_velocity[3];
  float kr[3];
  float kom[3];
  float kf_correction;
  float angle_corrections[2];
  bool enable_motors;
} SO3Command;

class QuadrotorSimulator : public rclcpp::Node
{
 public:
  QuadrotorSimulator();
  void run(void);
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Quadrotor quad_;
  SO3Command command_;

 protected:
  typedef struct _ControlInput
  {
    double rpm[4];
  } ControlInput;

  /*
   * The callback called when ROS message arrives and needs to fill in the
   * command_ field that would be passed to the getControl function.
   */
  void cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr cmd);
  void extern_force_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr f_ext);
  void extern_moment_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr m_ext);

  /*
   * Called by the simulator to get the current motor speeds. This is the
   * controller that would be running on the robot.
   * @param[in] quad Quadrotor instance which is being simulated
   * @param[in] cmd The command input which is filled by cmd_callback
   */
  ControlInput getControl(const Quadrotor &quad, const SO3Command &cmd) const;


 private:
  void stateToOdomMsg(const Quadrotor::State &state, nav_msgs::msg::Odometry &odom) const;
  void quadToImuMsg(const Quadrotor &quad, sensor_msgs::msg::Imu &imu) const;
  void tfBroadcast(const nav_msgs::msg::Odometry &odom_msg);

  // ROS1: 
  // ros::Publisher pub_odom_;
  // ros::Publisher pub_imu_;
  // ros::Publisher pub_output_data_;
  // ros::Subscriber sub_cmd_;
  // ros::Subscriber sub_extern_force_;
  // ros::Subscriber sub_extern_moment_;

  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_extern_force_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_extern_moment_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<kr_mav_msgs::msg::OutputData>::SharedPtr pub_output_data_;
  double simulation_rate_;
  double odom_rate_;
  std::string quad_name_;
  std::string world_frame_id_;

};

QuadrotorSimulator::QuadrotorSimulator():Node("quadrotor_simulator_so3")
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
  pub_output_data_ = this->create_publisher<kr_mav_msgs::msg::OutputData>("output_data", 100);

  sub_cmd_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>("cmd", 
      100, std::bind(&QuadrotorSimulator::cmd_callback, this, std::placeholders::_1));
  sub_extern_force_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("extern_force", 
      10, std::bind(&QuadrotorSimulator::extern_force_callback, this, std::placeholders::_1));
  sub_extern_moment_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("extern_moment", 
      10, std::bind(&QuadrotorSimulator::extern_moment_callback, this, std::placeholders::_1));
  // ROS1: 
  // pub_odom_ = n.advertise<nav_msgs::Odometry>("odom", 100);
  // pub_imu_ = n.advertise<sensor_msgs::Imu>("imu", 100);
  // pub_output_data_ = n.advertise<kr_mav_msgs::OutputData>("output_data", 100);
  // sub_cmd_ =
  //     n.subscribe<T>("cmd", 100, &QuadrotorSimulatorBase::cmd_callback, this, ros::TransportHints().tcpNoDelay());
  // sub_extern_force_ = n.subscribe<geometry_msgs::Vector3Stamped>(
  //     "extern_force", 10, &QuadrotorSimulatorBase::extern_force_callback, this, ros::TransportHints().tcpNoDelay());
  // sub_extern_moment_ = n.subscribe<geometry_msgs::Vector3Stamped>(
  //     "extern_moment", 10, &QuadrotorSimulatorBase::extern_moment_callback, this, ros::TransportHints().tcpNoDelay());

  declare_parameter("rate/simulation", 1000.0);
  get_parameter("rate/simulation", simulation_rate_);
  rcpputils::assert_true(simulation_rate_ > 0);

  declare_parameter("rate/odom", 100.0);
  get_parameter("rate/odom", odom_rate_);

  declare_parameter("world_frame_id", std::string("simulator"));
  get_parameter("world_frame_id", world_frame_id_);

  declare_parameter("quadrotor_name", std::string("quadrotor"));
  get_parameter("quadrotor_name", quad_name_);

  // ROS1: 
  // n.param("rate/simulation", simulation_rate_, 1000.0);
  // n.param("rate/odom", odom_rate_, 100.0);
  // n.param("world_frame_id", world_frame_id_, std::string("simulator"));
  // n.param("quadrotor_name", quad_name_, std::string("quadrotor"));



  // TODO: Convert this lambda function. Use list_parameters and search through 
  // list for desired param.
  
  // auto get_param = [&n](const std::string &param_name) {
  //   double param;
  //   if(!n.hasParam(param_name))
  //   {
  //     ROS_WARN("Simulator sleeping to wait for param %s", param_name.c_str());
  //     ros::Duration(0.5).sleep();
  //   }
  //   if(!n.getParam(param_name, param))
  //   {
  //     const std::string error_msg = param_name + " not set";
  //     ROS_FATAL_STREAM(error_msg);
  //     throw std::logic_error(error_msg);
  //   }
  //   return param;
  // };
  // END TODO #################################################################

  quad_.setMass(get_parameter("mass").as_double());
  quad_.setInertia(Eigen::Vector3d(get_parameter("Ixx").as_double(), get_parameter("Iyy").as_double(), get_parameter("Izz").as_double()).asDiagonal());
  quad_.setGravity(get_parameter("gravity").as_double());
  quad_.setPropRadius(get_parameter("prop_radius").as_double());
  quad_.setPropellerThrustCoefficient(get_parameter("thrust_coefficient").as_double());
  quad_.setArmLength(get_parameter("arm_length").as_double());
  quad_.setMotorTimeConstant(get_parameter("motor_time_constant").as_double());
  quad_.setMinRPM(get_parameter("min_rpm").as_double());
  quad_.setMaxRPM(get_parameter("max_rpm").as_double());
  quad_.setDragCoefficient(get_parameter("drag_coefficient").as_double());

  Eigen::Vector3d initial_pos;

  declare_parameter("initial_position/x", 0.0);
  declare_parameter("initial_position/y", 0.0);
  declare_parameter("initial_position/z", 0.0);
  get_parameter("initial_position/x", initial_pos(0));
  get_parameter("initial_position/y", initial_pos(1));
  get_parameter("initial_position/z", initial_pos(2));

  // ROS1:
  // n.param("initial_position/x", initial_pos(0), 0.0);
  // n.param("initial_position/y", initial_pos(1), 0.0);
  // n.param("initial_position/z", initial_pos(2), 0.0);

  Eigen::Quaterniond initial_q;
  declare_parameter("initial_orientation/w", 1.0);
  declare_parameter("initial_orientation/x", 0.0);
  declare_parameter("initial_orientation/y", 0.0);
  declare_parameter("initial_orientation/z", 0.0);
  get_parameter("initial_orientation/w", initial_q.w());
  get_parameter("initial_orientation/x", initial_q.x());
  get_parameter("initial_orientation/y", initial_q.y());
  get_parameter("initial_orientation/z", initial_q.z());

  // ROS1:
  // n.param("initial_orientation/w", initial_q.w(), 1.0);
  // n.param("initial_orientation/x", initial_q.x(), 0.0);
  // n.param("initial_orientation/y", initial_q.y(), 0.0);
  // n.param("initial_orientation/z", initial_q.z(), 0.0);

  initial_q.normalize();
  Quadrotor::State state = quad_.getState();
  state.x(0) = initial_pos(0);
  state.x(1) = initial_pos(1);
  state.x(2) = initial_pos(2);
  state.R = initial_q.matrix();
  quad_.setState(state);
}
 
void QuadrotorSimulator::run(void)
{
  // Call once with empty command to initialize values
  cmd_callback(std::make_shared<kr_mav_msgs::msg::SO3Command>());

  QuadrotorSimulator::ControlInput control;

  nav_msgs::msg::Odometry odom_msg;
  sensor_msgs::msg::Imu imu_msg;
  kr_mav_msgs::msg::OutputData output_data_msg;

  odom_msg.header.frame_id = world_frame_id_;
  odom_msg.child_frame_id = quad_name_;
  imu_msg.header.frame_id = quad_name_;
  output_data_msg.header.frame_id = quad_name_;

  const double simulation_dt = 1 / simulation_rate_;

  // ros::Rate r(simulation_rate_);
  rclcpp::Rate r(simulation_rate_);

  // const ros::Duration odom_pub_duration(1 / odom_rate_);
  const rclcpp::Duration odom_pub_duration(1 / odom_rate_);
  rclcpp::Time next_odom_pub_time = this->now();

  // auto base_node = std::make_shared<this>();
  while(rclcpp::ok())
  {
    rclcpp::spin(shared_from_this());


    control = getControl(quad_, command_);
    quad_.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad_.step(simulation_dt);

    rclcpp::Time tnow = this->now();

    if(tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      const Quadrotor::State &state = quad_.getState();

      stateToOdomMsg(state, odom_msg);
      odom_msg.header.stamp = tnow;
      pub_odom_->publish(odom_msg);
      tfBroadcast(odom_msg);

      quadToImuMsg(quad_, imu_msg);
      imu_msg.header.stamp = tnow;
      pub_imu_->publish(imu_msg);

      // Also publish an OutputData msg
      output_data_msg.header.stamp = tnow;
      output_data_msg.orientation = imu_msg.orientation;
      output_data_msg.angular_velocity = imu_msg.angular_velocity;
      output_data_msg.linear_acceleration = imu_msg.linear_acceleration;
      output_data_msg.motor_rpm[0] = state.motor_rpm(0);
      output_data_msg.motor_rpm[1] = state.motor_rpm(1);
      output_data_msg.motor_rpm[2] = state.motor_rpm(2);
      output_data_msg.motor_rpm[3] = state.motor_rpm(3);
      pub_output_data_->publish(output_data_msg);
    }

    r.sleep();
  }
}

void QuadrotorSimulator::extern_force_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr f_ext)
{
  quad_.setExternalForce(Eigen::Vector3d(f_ext->vector.x, f_ext->vector.y, f_ext->vector.z));
}

void QuadrotorSimulator::extern_moment_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr m_ext)
{
  quad_.setExternalMoment(Eigen::Vector3d(m_ext->vector.x, m_ext->vector.y, m_ext->vector.z));
}

void QuadrotorSimulator::stateToOdomMsg(const Quadrotor::State &state, nav_msgs::msg::Odometry &odom) const
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

void QuadrotorSimulator::quadToImuMsg(const Quadrotor &quad, sensor_msgs::msg::Imu &imu) const
{
  const Quadrotor::State state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  const double kf = quad.getPropellerThrustCoefficient();
  const double m = quad.getMass();
  const Eigen::Vector3d &external_force = quad.getExternalForce();
  const double g = quad.getGravity();
  const double thrust = kf * state.motor_rpm.square().sum();
  Eigen::Vector3d acc;
  if(state.x(2) < 1e-4)
  {
    acc = state.R.transpose() * (external_force / m + Eigen::Vector3d(0, 0, g));
  }
  else
  {
    acc = thrust / m * Eigen::Vector3d(0, 0, 1) + state.R.transpose() * external_force / m;
    if(quad.getDragCoefficient() != 0)
    {
      const double drag_coefficient = quad.getDragCoefficient();
      const double mass = quad.getMass();
      Eigen::Matrix3d P;
      P << 1, 0, 0, 0, 1, 0, 0, 0, 0;
      acc -= drag_coefficient / mass * P * state.R.transpose() * state.v;
    }
  }

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);
}

void QuadrotorSimulator::tfBroadcast(const nav_msgs::msg::Odometry &odom_msg)
{
  geometry_msgs::msg::TransformStamped ts;

  ts.header.stamp = odom_msg.header.stamp;
  ts.header.frame_id = odom_msg.header.frame_id;
  ts.child_frame_id = odom_msg.child_frame_id;

  ts.transform.translation.x = odom_msg.pose.pose.position.x;
  ts.transform.translation.y = odom_msg.pose.pose.position.y;
  ts.transform.translation.z = odom_msg.pose.pose.position.z;

  ts.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(ts);
}
 
void QuadrotorSimulator::cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr cmd)
{
  command_.force[0] = cmd->force.x;
  command_.force[1] = cmd->force.y;
  command_.force[2] = cmd->force.z;
  command_.qx = cmd->orientation.x;
  command_.qy = cmd->orientation.y;
  command_.qz = cmd->orientation.z;
  command_.qw = cmd->orientation.w;
  command_.angular_velocity[0] = cmd->angular_velocity.x;
  command_.angular_velocity[1] = cmd->angular_velocity.y;
  command_.angular_velocity[2] = cmd->angular_velocity.z;
  command_.kr[0] = cmd->kr[0];
  command_.kr[1] = cmd->kr[1];
  command_.kr[2] = cmd->kr[2];
  command_.kom[0] = cmd->kom[0];
  command_.kom[1] = cmd->kom[1];
  command_.kom[2] = cmd->kom[2];
  command_.kf_correction = cmd->aux.kf_correction;
  command_.angle_corrections[0] = cmd->aux.angle_corrections[0];  // Not used yet
  command_.angle_corrections[1] = cmd->aux.angle_corrections[1];  // Not used yet
  command_.enable_motors = cmd->aux.enable_motors;
}
 
QuadrotorSimulator::ControlInput QuadrotorSimulator::getControl(const Quadrotor &quad,
                                                                      const SO3Command &cmd) const
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf - cmd.kf_correction;
  const double km = _km / _kf * kf;

  const double d = quad.getArmLength();
  const Eigen::Matrix3f J = quad.getInertia().cast<float>();
  const float I[3][3] = {{J(0, 0), J(0, 1), J(0, 2)}, {J(1, 0), J(1, 1), J(1, 2)}, {J(2, 0), J(2, 1), J(2, 2)}};
  const Quadrotor::State &state = quad.getState();

  float R11 = state.R(0, 0);
  float R12 = state.R(0, 1);
  float R13 = state.R(0, 2);
  float R21 = state.R(1, 0);
  float R22 = state.R(1, 1);
  float R23 = state.R(1, 2);
  float R31 = state.R(2, 0);
  float R32 = state.R(2, 1);
  float R33 = state.R(2, 2);

  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  float Rd11 = cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 = cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 = cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 + Rd12 * R12 + Rd22 * R22 + Rd32 * R32 + Rd13 * R13 +
                              Rd23 * R23 + Rd33 * R33));

  // TODO: Figure out RCLCPP WARN THROTTLE:
  // if(Psi > 1.0f){  // Position control stability guaranteed only when Psi < 1
  //   //ROS_WARN_THROTTLE(1, "Warning Psi = %f > 1", Psi);
  //   auto clk = get_clock();
  //   RCLCPP_WARN_THROTTLE(get_logger(), *clk, 1, "Warning Psi = %f > 1", Psi);
  // }

  float force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 - R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

  float Omd1 = cmd.angular_velocity[0] * (R11 * Rd11 + R21 * Rd21 + R31 * Rd31) +
               cmd.angular_velocity[1] * (R11 * Rd12 + R21 * Rd22 + R31 * Rd32) +
               cmd.angular_velocity[2] * (R11 * Rd13 + R21 * Rd23 + R31 * Rd33);
  float Omd2 = cmd.angular_velocity[0] * (R12 * Rd11 + R22 * Rd21 + R32 * Rd31) +
               cmd.angular_velocity[1] * (R12 * Rd12 + R22 * Rd22 + R32 * Rd32) +
               cmd.angular_velocity[2] * (R12 * Rd13 + R22 * Rd23 + R32 * Rd33);
  float Omd3 = cmd.angular_velocity[0] * (R13 * Rd11 + R23 * Rd21 + R33 * Rd31) +
               cmd.angular_velocity[1] * (R13 * Rd12 + R23 * Rd22 + R33 * Rd32) +
               cmd.angular_velocity[2] * (R13 * Rd13 + R23 * Rd23 + R33 * Rd33);

  float eOm1 = Om1 - Omd1;
  float eOm2 = Om2 - Omd2;
  float eOm3 = Om3 - Omd3;

  // TODO: Change this to the new term as in http://arxiv.org/abs/1304.6765:
  // Omd^ * J * Omd
  float in1 =
      Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) - Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 =
      Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) - Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 =
      Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) - Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

  float M1 = -cmd.kr[0] * eR1 - cmd.kom[0] * eOm1 + in1;
  float M2 = -cmd.kr[1] * eR2 - cmd.kom[1] * eOm2 + in2;
  float M3 = -cmd.kr[2] * eR3 - cmd.kom[2] * eOm3 + in3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  ControlInput control;
  for(int i = 0; i < 4; i++)
  {
    if(cmd.enable_motors)
    {
      if(w_sq[i] < 0)
        w_sq[i] = 0;

      control.rpm[i] = sqrtf(w_sq[i]);
    }
    else
    {
      control.rpm[i] = 0;
    }
  }
  return control;
}
}  // namespace QuadrotorSimulator

#endif

int main(int argc, char **argv)
{
  std::cout << "Hello Kashy" << std::endl;
//   ros::init(argc, argv, "kr_quadrotor_simulator_so3");
//   ros::NodeHandle nh("~");
//   QuadrotorSimulator::QuadrotorSimulatorSO3 quad_sim(nh);
//   quad_sim.run();
  // auto node = std::make_shared<QuadrotorSimulator::QuadrotorSimulatorSO3>();
  // node->run();
  // rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;



}
