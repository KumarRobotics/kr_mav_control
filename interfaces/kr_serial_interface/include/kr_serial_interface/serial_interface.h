#ifndef QUAD_SERIAL_COMM_SERIAL_INTERFACE_H
#define QUAD_SERIAL_COMM_SERIAL_INTERFACE_H

#include <vector>
#include <kr_quadrotor_msgs/Serial.h>
#include <boost/function.hpp>

void encode_serial_msg(const kr_quadrotor_msgs::Serial &msg,
                       std::vector<uint8_t> &serial_data);

void process_serial_data(
    const uint8_t *data, const size_t count,
    boost::function<void(kr_quadrotor_msgs::Serial &)> callback);
#endif
