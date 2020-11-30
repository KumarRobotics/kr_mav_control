#ifndef QUAD_SERIAL_COMM_SERIAL_INTERFACE_H
#define QUAD_SERIAL_COMM_SERIAL_INTERFACE_H

#include <kr_mav_msgs/Serial.h>

#include <boost/function.hpp>
#include <vector>

void encode_serial_msg(const kr_mav_msgs::Serial &msg, std::vector<uint8_t> &serial_data);

void process_serial_data(const uint8_t *data, const size_t count,
                         boost::function<void(kr_mav_msgs::Serial &)> callback);
#endif
