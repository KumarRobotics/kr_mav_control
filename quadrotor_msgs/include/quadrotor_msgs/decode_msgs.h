#ifndef QUADROTOR_MSGS_DECODE_MSGS_H
#define QUADROTOR_MSGS_DECODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <quadrotor_msgs/OutputData.h>
#include <quadrotor_msgs/StatusData.h>

namespace quadrotor_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::StatusData &status);
}

#endif
