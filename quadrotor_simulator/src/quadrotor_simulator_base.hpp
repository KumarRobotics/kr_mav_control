#ifndef QUADROTOR_SIMULATOR_BASE_HPP_
#define QUADROTOR_SIMULATOR_BASE_HPP_

#include <ros/ros.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>
#include <random>

namespace QuadrotorSimulator
{
template <typename T, typename U>
class QuadrotorSimulatorBase
{
 public:
  QuadrotorSimulatorBase(ros::NodeHandle &n);
  void run(void);
  void extern_force_callback(
      const geometry_msgs::Vector3Stamped::ConstPtr &f_ext);
  void extern_moment_callback(
      const geometry_msgs::Vector3Stamped::ConstPtr &m_ext);

 protected:
  typedef struct _ControlInput
  {
    double rpm[4];
  } ControlInput;

  /*
   * The callback called when ROS message arrives and needs to fill in the
   * command_ field that would be passed to the getControl function.
   */
  virtual void cmd_callback(const typename T::ConstPtr &cmd) = 0;

  /*
   * Called by the simulator to get the current motor speeds. This is the
   * controller that would be running on the robot.
   * @param[in] quad Quadrotor instance which is being simulated
   * @param[in] cmd The command input which is filled by cmd_callback
   */
  virtual ControlInput getControl(const Quadrotor &quad,
                                  const U &cmd) const = 0;

  Quadrotor quad_;
  U command_;

 private:
  void stateToOdomMsg(const Quadrotor::State &state,
                      nav_msgs::Odometry &odom) const;
  void quadToImuMsg(const Quadrotor &quad, sensor_msgs::Imu &imu);
  void tfBroadcast(const nav_msgs::Odometry &odom_msg);

  ros::Publisher pub_odom_;
  ros::Publisher pub_imu_;
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_extern_force_;
  ros::Subscriber sub_extern_moment_;
  double simulation_rate_;
  double odom_rate_;
  std::string quad_name_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // For IMU noise
  Eigen::Vector3d acc_sigma_, angvel_sigma_;
  std::default_random_engine generator_;
};

template <typename T, typename U>
QuadrotorSimulatorBase<T, U>::QuadrotorSimulatorBase(ros::NodeHandle &n)
{
  pub_odom_ = n.advertise<nav_msgs::Odometry>("odom", 100);
  pub_imu_ = n.advertise<sensor_msgs::Imu>("imu", 100);
  sub_cmd_ = n.subscribe<T>("cmd", 100, &QuadrotorSimulatorBase::cmd_callback,
                            this, ros::TransportHints().tcpNoDelay());
  sub_extern_force_ = n.subscribe<geometry_msgs::Vector3Stamped>(
      "extern_force", 10, &QuadrotorSimulatorBase::extern_force_callback, this,
      ros::TransportHints().tcpNoDelay());
  sub_extern_moment_ = n.subscribe<geometry_msgs::Vector3Stamped>(
      "extern_moment", 10, &QuadrotorSimulatorBase::extern_moment_callback,
      this, ros::TransportHints().tcpNoDelay());

  n.param("rate/simulation", simulation_rate_, 1000.0);
  ROS_ASSERT(simulation_rate_ > 0);

  n.param("rate/odom", odom_rate_, 100.0);

  n.param("quadrotor_name", quad_name_, std::string("quadrotor"));

  auto get_param = [&n](const std::string &param_name) {
    double param;
    if(!n.getParam(param_name, param))
    {
      const std::string error_msg = param_name + " not set";
      ROS_FATAL_STREAM(error_msg);
      throw std::logic_error(error_msg);
    }
    return param;
  };

  quad_.setMass(get_param("mass"));
  quad_.setInertia(
      Eigen::Vector3d(get_param("Ixx"), get_param("Iyy"), get_param("Izz"))
          .asDiagonal());
  quad_.setGravity(get_param("gravity"));
  quad_.setPropRadius(get_param("prop_radius"));
  quad_.setPropellerThrustCoefficient(get_param("thrust_coefficient"));
  quad_.setArmLength(get_param("arm_length"));
  quad_.setMotorTimeConstant(get_param("motor_time_constant"));
  quad_.setMinRPM(get_param("min_rpm"));
  quad_.setMaxRPM(get_param("max_rpm"));
  quad_.setDragCoefficient(get_param("drag_coefficient"));

  Eigen::Vector3d initial_pos;
  n.param("initial_position/x", initial_pos(0), 0.0);
  n.param("initial_position/y", initial_pos(1), 0.0);
  n.param("initial_position/z", initial_pos(2), 0.0);

  Eigen::Quaterniond initial_q;
  n.param("initial_orientation/w", initial_q.w(), 1.0);
  n.param("initial_orientation/x", initial_q.x(), 0.0);
  n.param("initial_orientation/y", initial_q.y(), 0.0);
  n.param("initial_orientation/z", initial_q.z(), 0.0);
  initial_q.normalize();

  Quadrotor::State state = quad_.getState();
  state.x(0) = initial_pos(0);
  state.x(1) = initial_pos(1);
  state.x(2) = initial_pos(2);
  state.R = initial_q.matrix();
  quad_.setState(state);

  // IMU noise parameters
  std::vector<double> acc_sigma, angvel_sigma;
  n.param("sigma/accel", acc_sigma, std::vector<double>(3,0));
  n.param("sigma/angvel", angvel_sigma, std::vector<double>(3,0));
  acc_sigma_ = Eigen::Vector3d::Map(acc_sigma.data());
  angvel_sigma_ = Eigen::Vector3d::Map(angvel_sigma.data());
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::run(void)
{
  // initialize command
  typename T::Ptr empty_cmd = boost::make_shared<T>();
  cmd_callback(empty_cmd);

  QuadrotorSimulatorBase::ControlInput control;

  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id = "/" + quad_name_;
  imu_msg.header.frame_id = "/" + quad_name_;

  const double simulation_dt = 1 / simulation_rate_;
  ros::Rate r(simulation_rate_);

  const ros::Duration odom_pub_duration(1 / odom_rate_);
  ros::Time next_odom_pub_time = ros::Time::now();

  while(ros::ok())
  {
    ros::spinOnce();

    control = getControl(quad_, command_);
    quad_.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                   control.rpm[3]);
    quad_.step(simulation_dt);

    ros::Time tnow = ros::Time::now();

    if(tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      const Quadrotor::State &state = quad_.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad_, imu_msg);
      odom_msg.header.stamp = tnow;
      imu_msg.header.stamp = tnow;
      pub_odom_.publish(odom_msg);
      pub_imu_.publish(imu_msg);
      tfBroadcast(odom_msg);
    }

    r.sleep();
  }
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::extern_force_callback(
    const geometry_msgs::Vector3Stamped::ConstPtr &f_ext)
{
  quad_.setExternalForce(
      Eigen::Vector3d(f_ext->vector.x, f_ext->vector.y, f_ext->vector.z));
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::extern_moment_callback(
    const geometry_msgs::Vector3Stamped::ConstPtr &m_ext)
{
  quad_.setExternalMoment(
      Eigen::Vector3d(m_ext->vector.x, m_ext->vector.y, m_ext->vector.z));
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::stateToOdomMsg(
    const Quadrotor::State &state, nav_msgs::Odometry &odom) const
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

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::quadToImuMsg(const Quadrotor &quad,
                                                sensor_msgs::Imu &imu) 
{
  const Quadrotor::State state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  std::normal_distribution<double> p_dist = std::normal_distribution<double>(0,angvel_sigma_[0]);
  std::normal_distribution<double> q_dist = std::normal_distribution<double>(0,angvel_sigma_[1]);
  std::normal_distribution<double> r_dist = std::normal_distribution<double>(0,angvel_sigma_[2]);
  std::normal_distribution<double> ax_dist = std::normal_distribution<double>(0,acc_sigma_[0]);
  std::normal_distribution<double> ay_dist = std::normal_distribution<double>(0,acc_sigma_[1]);
  std::normal_distribution<double> az_dist = std::normal_distribution<double>(0,acc_sigma_[2]);

  imu.angular_velocity.x = state.omega(0) + p_dist(generator_);
  imu.angular_velocity.y = state.omega(1) + q_dist(generator_);
  imu.angular_velocity.z = state.omega(2) + r_dist(generator_);

  const double kf = quad.getPropellerThrustCoefficient();
  const double m = quad.getMass();
  const Eigen::Vector3d &external_force = quad.getExternalForce();
  const double g = quad.getGravity();
  const double thrust = kf * state.motor_rpm.square().sum();
  Eigen::Vector3d acc;
  if(state.x(2) < 1e-4)
  {
    acc = state.R * (external_force / m + Eigen::Vector3d(0, 0, g));
  }
  else
  {
    acc = thrust / m * Eigen::Vector3d(0, 0, 1) +
          state.R.transpose() * external_force / m;
    if(quad.getDragCoefficient() != 0)
    {
      const double drag_coefficient = quad.getDragCoefficient();
      const double mass = quad.getMass();
      Eigen::Matrix3d P;
      P << 1, 0, 0,
           0, 1, 0,
           0, 0, 0;
      acc -= drag_coefficient / mass * P * state.R.transpose() * state.v;
    }
  }


  imu.linear_acceleration.x = acc(0) + ax_dist(generator_);
  imu.linear_acceleration.y = acc(1) + ay_dist(generator_);
  imu.linear_acceleration.z = acc(2) + az_dist(generator_);
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::tfBroadcast(
    const nav_msgs::Odometry &odom_msg)
{
  geometry_msgs::TransformStamped ts;

  ts.header.stamp = odom_msg.header.stamp;
  ts.header.frame_id = odom_msg.header.frame_id;
  ts.child_frame_id = odom_msg.child_frame_id;

  ts.transform.translation.x = odom_msg.pose.pose.position.x;
  ts.transform.translation.y = odom_msg.pose.pose.position.y;
  ts.transform.translation.z = odom_msg.pose.pose.position.z;

  ts.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_.sendTransform(ts);
}
}

#endif
