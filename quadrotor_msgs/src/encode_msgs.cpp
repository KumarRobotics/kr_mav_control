#include "quadrotor_msgs/encode_msgs.h"
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/comm_types.h>

namespace quadrotor_msgs
{

void encodeSO3Command(const quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output)
{
  struct SO3_CMD_INPUT so3_cmd_input;

  so3_cmd_input.force[0] = so3_command.force.x*1e3;
  so3_cmd_input.force[1] = so3_command.force.y*1e3;
  so3_cmd_input.force[2] = so3_command.force.z*1e3;

  so3_cmd_input.des_qx = so3_command.orientation.x*1e4;
  so3_cmd_input.des_qy = so3_command.orientation.y*1e4;
  so3_cmd_input.des_qz = so3_command.orientation.z*1e4;
  so3_cmd_input.des_qw = so3_command.orientation.w*1e4;

  so3_cmd_input.kR[0] = so3_command.kR[0]*1e3;
  so3_cmd_input.kR[1] = so3_command.kR[1]*1e3;
  so3_cmd_input.kR[2] = so3_command.kR[2]*1e3;

  so3_cmd_input.kOm[0] = so3_command.kOm[0]*1e3;
  so3_cmd_input.kOm[1] = so3_command.kOm[1]*1e3;
  so3_cmd_input.kOm[2] = so3_command.kOm[2]*1e3;

  so3_cmd_input.cur_yaw = so3_command.aux.current_yaw*1e4;

  so3_cmd_input.corrections[0] = so3_command.aux.corrections[0]*1e11;
  so3_cmd_input.corrections[1] = so3_command.aux.corrections[1]*1e4;
  so3_cmd_input.corrections[2] = so3_command.aux.corrections[2]*1e4;

  so3_cmd_input.enable_motors = so3_command.aux.enable_motors;
  so3_cmd_input.use_external_yaw = so3_command.aux.use_external_yaw;
  so3_cmd_input.use_angle_corrections = so3_command.aux.use_angle_corrections;

  so3_cmd_input.seq = so3_command.header.seq;

  output.resize(sizeof(so3_cmd_input));
  memcpy(&output[0], &so3_cmd_input, sizeof(so3_cmd_input));
}

}
