#ifndef QUADROTOR_MSGS_ENCODE_MSGS_H
#define QUADROTOR_MSGS_ENCODE_MSGS_H

#include <kr_mav_msgs/PWMCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <kr_mav_msgs/TRPYCommand.h>
#include <stdint.h>

#include <vector>

namespace kr_mav_msgs
{
void encodeSO3Command(const kr_mav_msgs::SO3Command &so3_command, std::vector<uint8_t> &output);
void encodeTRPYCommand(const kr_mav_msgs::TRPYCommand &trpy_command, std::vector<uint8_t> &output);
void encodePWMCommand(const kr_mav_msgs::PWMCommand &pwm_command, std::vector<uint8_t> &output);
}  // namespace kr_mav_msgs

#endif
