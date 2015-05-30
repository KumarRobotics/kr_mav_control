#ifndef QUADROTOR_MSGS_ENCODE_MSGS_H
#define QUADROTOR_MSGS_ENCODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/PWMCommand.h>

namespace quadrotor_msgs
{

void encodeSO3Command(const quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const quadrotor_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);
void encodePWMCommand(const quadrotor_msgs::PWMCommand &pwm_command,
                      std::vector<uint8_t> &output);
}

#endif
