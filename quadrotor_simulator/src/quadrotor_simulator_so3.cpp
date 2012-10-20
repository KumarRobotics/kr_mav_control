#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>

typedef struct _Control
{
  double rpm[4];
} Control;

typedef struct _Command
{
  float force[3];
  float qx, qy, qz, qw;
  float kR[3];
  float kOm[3];
  float corrections[3];
} Command;

static Command command;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state,
                    nav_msgs::Odometry &odom);


static Control getControl(const QuadrotorSimulator::Quadrotor &quad,
                          const Command &cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf - cmd.corrections[0];
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

  float Rd11 = cmd.qw*cmd.qw + cmd.qx*cmd.qx - cmd.qy*cmd.qy - cmd.qz*cmd.qz;
  float Rd12 = 2*(cmd.qx*cmd.qy - cmd.qw*cmd.qz);
  float Rd13 = 2*(cmd.qx*cmd.qz + cmd.qw*cmd.qy);
  float Rd21 = 2*(cmd.qx*cmd.qy + cmd.qw*cmd.qz);
  float Rd22 = cmd.qw*cmd.qw - cmd.qx*cmd.qx + cmd.qy*cmd.qy - cmd.qz*cmd.qz;
  float Rd23 = 2*(cmd.qy*cmd.qz - cmd.qw*cmd.qx);
  float Rd31 = 2*(cmd.qx*cmd.qz - cmd.qw*cmd.qy);
  float Rd32 = 2*(cmd.qy*cmd.qz + cmd.qw*cmd.qx);
  float Rd33 = cmd.qw*cmd.qw - cmd.qx*cmd.qx - cmd.qy*cmd.qy + cmd.qz*cmd.qz;

  float Psi = 0.5f*(3.0f - (Rd11*R11 + Rd21*R21 + Rd31*R31 +
                            Rd12*R12 + Rd22*R22 + Rd32*R32 +
                            Rd13*R13 + Rd23*R23 + Rd33*R33));

  float force = 0;
  if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
    force = cmd.force[0]*R13 + cmd.force[1]*R23 + cmd.force[2]*R33;

  float eR1 = 0.5f*(R12*Rd13 - R13*Rd12 + R22*Rd23 - R23*Rd22 + R32*Rd33 - R33*Rd32);
  float eR2 = 0.5f*(R13*Rd11 - R11*Rd13 - R21*Rd23 + R23*Rd21 - R31*Rd33 + R33*Rd31);
  float eR3 = 0.5f*(R11*Rd12 - R12*Rd11 + R21*Rd22 - R22*Rd21 + R31*Rd32 - R32*Rd31);

  float eOm1 = Om1;
  float eOm2 = Om2;
  float eOm3 = Om3;

  float in1 = Om2*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3) -
      Om3*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3);
  float in2 = Om3*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3) -
      Om1*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3);
  float in3 = Om1*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3) -
      Om2*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3);

  float M1 = -cmd.kR[0]*eR1 - cmd.kOm[0]*eOm1 + in1;
  float M2 = -cmd.kR[1]*eR2 - cmd.kOm[1]*eOm2 + in2;
  float M3 = -cmd.kR[2]*eR3 - cmd.kOm[2]*eOm3 + in3;

  float w_sq[4];
  w_sq[0] = force/(4*kf) - M2/(2*d*kf) + M3/(4*km);
  w_sq[1] = force/(4*kf) + M2/(2*d*kf) + M3/(4*km);
  w_sq[2] = force/(4*kf) + M1/(2*d*kf) - M3/(4*km);
  w_sq[3] = force/(4*kf) - M1/(2*d*kf) - M3/(4*km);

  Control control;
  for(int i = 0; i < 4; i++)
  {
    if(w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

static void cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr &cmd)
{
  command.force[0] = cmd->force.x;
  command.force[1] = cmd->force.y;
  command.force[2] = cmd->force.z;
  command.qx = cmd->orientation.x;
  command.qy = cmd->orientation.y;
  command.qz = cmd->orientation.z;
  command.qw = cmd->orientation.w;
  command.kR[0] = cmd->kR[0];
  command.kR[1] = cmd->kR[1];
  command.kR[2] = cmd->kR[2];
  command.kOm[0] = cmd->kOm[0];
  command.kOm[1] = cmd->kOm[1];
  command.kOm[2] = cmd->kOm[2];
  command.corrections[0] = cmd->aux.corrections[0];
  command.corrections[1] = cmd->aux.corrections[1];
  command.corrections[2] = cmd->aux.corrections[2];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_simulator_so3");

  ros::NodeHandle n("~");

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback,
                                        ros::TransportHints().tcpNoDelay());

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1/odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  QuadrotorSimulator::Quadrotor quad;
  QuadrotorSimulator::Quadrotor::State state = quad.getState();

  ros::Rate r(simulation_rate);
  const double dt = 1/simulation_rate;

  Control control;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id = "/" + quad_name;

  /*
  command.force[0] = 0;
  command.force[1] = 0;
  command.force[2] = quad.getMass()*quad.getGravity() + 0.1;
  command.qx = 0;
  command.qy = 0;
  command.qz = 0;
  command.qw = 1;
  command.kR[0] = 2;
  command.kR[1] = 2;
  command.kR[2] = 2;
  command.kOm[0] = 0.15;
  command.kOm[1] = 0.15;
  command.kOm[2] = 0.15;
  */

  ros::Time next_odom_pub_time = ros::Time::now();
  while(n.ok())
  {
    ros::spinOnce();

    control = getControl(quad, command);
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad.step(dt);

    ros::Time tnow = ros::Time::now();

    if(tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state = quad.getState();
      stateToOdomMsg(state, odom_msg);
      odom_pub.publish(odom_msg);
    }

    r.sleep();
  }

  return 0;
}

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state,
                     nav_msgs::Odometry &odom)
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
