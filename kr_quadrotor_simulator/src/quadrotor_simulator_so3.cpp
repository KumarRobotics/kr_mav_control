#include <kr_mav_msgs/SO3Command.h>

#include <Eigen/Geometry>

#include "quadrotor_simulator_base.hpp"

namespace QuadrotorSimulator
{
typedef struct _SO3Command
{
  float force[3];
  float qx, qy, qz, qw;
  float angular_velocity[3];
  float kR[3];
  float kOm[3];
  float kf_correction;
  float angle_corrections[2];
  bool enable_motors;
} SO3Command;

class QuadrotorSimulatorSO3 : public QuadrotorSimulatorBase<kr_mav_msgs::SO3Command, SO3Command>
{
 public:
  QuadrotorSimulatorSO3(ros::NodeHandle &nh) : QuadrotorSimulatorBase(nh) {}

 private:
  virtual void cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &cmd);
  virtual ControlInput getControl(const Quadrotor &quad, const SO3Command &cmd) const;
};

void QuadrotorSimulatorSO3::cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &cmd)
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
  command_.kR[0] = cmd->kR[0];
  command_.kR[1] = cmd->kR[1];
  command_.kR[2] = cmd->kR[2];
  command_.kOm[0] = cmd->kOm[0];
  command_.kOm[1] = cmd->kOm[1];
  command_.kOm[2] = cmd->kOm[2];
  command_.kf_correction = cmd->aux.kf_correction;
  command_.angle_corrections[0] = cmd->aux.angle_corrections[0];  // Not used yet
  command_.angle_corrections[1] = cmd->aux.angle_corrections[1];  // Not used yet
  command_.enable_motors = cmd->aux.enable_motors;
}

QuadrotorSimulatorSO3::ControlInput QuadrotorSimulatorSO3::getControl(const Quadrotor &quad,
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

  if(Psi > 1.0f)  // Position control stability guaranteed only when Psi < 1
    ROS_WARN_THROTTLE(1, "Warning Psi = %f > 1", Psi);

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

  float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1;
  float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2;
  float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kr_quadrotor_simulator_so3");

  ros::NodeHandle nh("~");

  QuadrotorSimulator::QuadrotorSimulatorSO3 quad_sim(nh);

  quad_sim.run();

  return 0;
}
