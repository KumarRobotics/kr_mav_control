#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

#include <stdint.h>
#include <vector>
#include <quadrotor_msgs/OutputData.h>

namespace quadrotor_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::OutputData &output);

}

#endif
