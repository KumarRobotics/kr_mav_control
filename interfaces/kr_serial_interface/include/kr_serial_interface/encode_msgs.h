#ifndef QUADROTOR_MSGS_ENCODE_MSGS_H
#define QUADROTOR_MSGS_ENCODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <kr_quadrotor_msgs/SO3Command.h>
#include <kr_quadrotor_msgs/TRPYCommand.h>
#include <kr_quadrotor_msgs/PWMCommand.h>

namespace kr_quadrotor_msgs
{

void encodeSO3Command(const kr_quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const kr_quadrotor_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);
void encodePWMCommand(const kr_quadrotor_msgs::PWMCommand &pwm_command,
                      std::vector<uint8_t> &output);
}

#endif
