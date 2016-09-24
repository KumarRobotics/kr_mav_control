#ifndef QUAD_SERIAL_COMM_SERIAL_INTERFACE_H
#define QUAD_SERIAL_COMM_SERIAL_INTERFACE_H

#include <vector>
#include <quadrotor_msgs/Serial.h>
#include <boost/function.hpp>

void encode_serial_msg(const quadrotor_msgs::Serial &msg,
                       std::vector<uint8_t> &serial_data);

void process_serial_data(
    const uint8_t *data, const uint8_t count,
    boost::function<void(quadrotor_msgs::Serial &)> callback);
#endif
