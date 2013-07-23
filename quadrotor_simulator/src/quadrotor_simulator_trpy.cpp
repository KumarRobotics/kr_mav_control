#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_simulator/Quadrotor.h>

typedef struct _ControlInput
{
  double rpm[4];
} ControlInput;

typedef struct _Command
{
  float thrust;
  float roll, pitch, yaw;
  bool enable_motors;
  float kR[3];
  float kOm[3];
} Command;

static Command command;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state, nav_msgs::Odometry &odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor &quad, sensor_msgs::Imu &imu);


static ControlInput getControl(const QuadrotorSimulator::Quadrotor &quad, const Command &cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf;
  const double km = _km/_kf*kf;

  const double d = quad.getArmLength();
  const Eigen::Matrix3f J = quad.getInertia().cast<float>();
  const float I[3][3] = {{J(0,0), J(0,1), J(0,2)},
                         {J(1,0), J(1,1), J(1,2)},
                         {J(2,0), J(2,1), J(2,2)}};
  const QuadrotorSimulator::Quadrotor::State state = quad.getState();

  float R11 = state.R(0,0);
  float R12 = state.R(0,1);
  float R13 = state.R(0,2);
  float R21 = state.R(1,0);
  float R22 = state.R(1,1);
  float R23 = state.R(1,2);
  float R31 = state.R(2,0);
  float R32 = state.R(2,1);
  float R33 = state.R(2,2);

  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  float Rd11 = cos(cmd.yaw)*cos(cmd.pitch);
  float Rd12 = cos(cmd.yaw)*sin(cmd.pitch)*sin(cmd.roll) - cos(cmd.roll)*sin(cmd.yaw);
  float Rd13 = sin(cmd.yaw)*sin(cmd.roll) + cos(cmd.yaw)*cos(cmd.roll)*sin(cmd.pitch);
  float Rd21 = cos(cmd.pitch)*sin(cmd.yaw);
  float Rd22 = cos(cmd.yaw)*cos(cmd.roll) + sin(cmd.yaw)*sin(cmd.pitch)*sin(cmd.roll);
  float Rd23 = cos(cmd.roll)*sin(cmd.yaw)*sin(cmd.pitch) - cos(cmd.yaw)*sin(cmd.roll);
  float Rd31 = -sin(cmd.pitch);
  float Rd32 = cos(cmd.pitch)*sin(cmd.roll);
  float Rd33 = cos(cmd.pitch)*cos(cmd.roll);

  float Psi = 0.5f*(3.0f - (Rd11*R11 + Rd21*R21 + Rd31*R31 +
                            Rd12*R12 + Rd22*R22 + Rd32*R32 +
                            Rd13*R13 + Rd23*R23 + Rd33*R33));

  float force = 0;
  if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
    force = cmd.thrust;

  float eR1 = 0.5f*(R12*Rd13 - R13*Rd12 + R22*Rd23 - R23*Rd22 + R32*Rd33 - R33*Rd32);
  float eR2 = 0.5f*(R13*Rd11 - R11*Rd13 - R21*Rd23 + R23*Rd21 - R31*Rd33 + R33*Rd31);
  float eR3 = 0.5f*(R11*Rd12 - R12*Rd11 + R21*Rd22 - R22*Rd21 + R31*Rd32 - R32*Rd31);

  float eOm1 = Om1;
  float eOm2 = Om2;
  float eOm3 = Om3;

  float in1 = Om2*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3) - Om3*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3);
  float in2 = Om3*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3) - Om1*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3);
  float in3 = Om1*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3) - Om2*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3);

  float M1 = -cmd.kR[0]*eR1 - cmd.kOm[0]*eOm1 + in1;
  float M2 = -cmd.kR[1]*eR2 - cmd.kOm[1]*eOm2 + in2;
  float M3 = -cmd.kR[2]*eR3 - cmd.kOm[2]*eOm3 + in3;

  float w_sq[4];
  w_sq[0] = force/(4*kf) - M2/(2*d*kf) + M3/(4*km);
  w_sq[1] = force/(4*kf) + M2/(2*d*kf) + M3/(4*km);
  w_sq[2] = force/(4*kf) + M1/(2*d*kf) - M3/(4*km);
  w_sq[3] = force/(4*kf) - M1/(2*d*kf) - M3/(4*km);

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

static void cmd_callback(const quadrotor_msgs::TRPYCommand::ConstPtr &cmd)
{
  command.thrust = cmd->thrust;
  command.roll = cmd->roll;
  command.pitch = cmd->pitch;
  command.yaw = cmd->yaw;
  command.enable_motors = cmd->aux.enable_motors;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_simulator_so3");

  ros::NodeHandle n("~");

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1/odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  double kR[3], kOm[3];
  n.param("gains/rot/x", kR[0], 1.0);
  n.param("gains/rot/y", kR[1], 1.0);
  n.param("gains/rot/z", kR[2], 1.0);
  n.param("gains/ang/x", kOm[0], 0.13);
  n.param("gains/ang/y", kOm[1], 0.13);
  n.param("gains/ang/z", kOm[2], 0.1);
  command.kR[0] = kR[0], command.kR[1] = kR[1], command.kR[2] = kR[2];
  command.kOm[0] = kOm[0], command.kOm[1] = kOm[1], command.kOm[2] = kOm[2];

  QuadrotorSimulator::Quadrotor quad;
  QuadrotorSimulator::Quadrotor::State state = quad.getState();

  ros::Rate r(simulation_rate);
  const double simulation_dt = 1/simulation_rate;

  command.enable_motors = false;

  ControlInput control;

  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id = "/" + quad_name;
  imu_msg.header.frame_id = "/" + quad_name;

  ros::Time next_odom_pub_time = ros::Time::now();
  while(n.ok())
  {
    ros::spinOnce();

    control = getControl(quad, command);
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad.step(simulation_dt);

    ros::Time tnow = ros::Time::now();

    if(tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      state = quad.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu_msg);
      odom_msg.header.stamp = tnow;
      imu_msg.header.stamp = tnow;
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu_msg);
    }

    r.sleep();
  }

  return 0;
}

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state, nav_msgs::Odometry &odom)
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

void quadToImuMsg(const QuadrotorSimulator::Quadrotor &quad, sensor_msgs::Imu &imu)
{
  const QuadrotorSimulator::Quadrotor::State state = quad.getState();
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
  const double thrust = kf*state.motor_rpm.square().sum();
  Eigen::Vector3d acc;
  if(state.x(2) < 1e-4)
  {
    acc = state.R*(external_force/m + Eigen::Vector3d(0,0,g));
  }
  else
  {
    acc = thrust/m*Eigen::Vector3d(0,0,1) + state.R*external_force/m;
  }

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);
}
