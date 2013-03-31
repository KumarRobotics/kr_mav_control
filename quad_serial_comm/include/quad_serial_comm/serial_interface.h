#ifndef _ENCODE_SERIAL_MSG_H_
#define _ENCODE_SERIAL_MSG_H_

#include <vector>
#include <quadrotor_msgs/Serial.h>
#include <boost/function.hpp>

void encode_serial_msg(const quadrotor_msgs::Serial &msg,
                       std::vector<uint8_t> &serial_data);

void process_serial_data(const uint8_t *data, const uint8_t count,
                         boost::function<void (quadrotor_msgs::Serial)> callback);
#endif
