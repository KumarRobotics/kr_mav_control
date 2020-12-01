#ifndef QUADROTOR_MSGS_DECODE_MSGS_H
#define QUADROTOR_MSGS_DECODE_MSGS_H

#include <kr_mav_msgs/OutputData.h>
#include <kr_mav_msgs/StatusData.h>
#include <stdint.h>

#include <vector>

namespace kr_mav_msgs
{
bool decodeOutputData(const std::vector<uint8_t> &data, kr_mav_msgs::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data, kr_mav_msgs::StatusData &status);
}  // namespace kr_mav_msgs

#endif
