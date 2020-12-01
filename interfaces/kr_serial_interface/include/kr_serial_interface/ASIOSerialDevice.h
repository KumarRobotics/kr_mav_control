/*
  This file is part of asio_serial_device, a class wrapper to
  use the boost::asio serial functionality.

  asio_serial_device is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Nathan Michael, Aug. 2011
*/

#ifndef ASIOSERIALDEVICE_H
#define ASIOSERIALDEVICE_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <deque>
#include <iostream>

// Class version of example ASIO over serial with boost::asio:
// http://groups.google.com/group/boost-list/browse_thread/thread/5cc7dcc7b90d41fc

#define MAX_READ_LENGTH 16

namespace ba = boost::asio;

class ASIOSerialDevice
{
 public:
  ASIOSerialDevice();
  ASIOSerialDevice(const std::string &device, unsigned int baud);
  ~ASIOSerialDevice();

  void SetReadCallback(const boost::function<void(const unsigned char *, size_t)> &handler);

  void Start();
  void Stop();
  void Close();

  void Read();
  bool Write(const std::vector<unsigned char> &msg);
  void Open(
      const std::string &device_, unsigned int baud_,
      ba::serial_port_base::parity parity = ba::serial_port_base::parity(ba::serial_port_base::parity::none),
      ba::serial_port_base::character_size csize = ba::serial_port_base::character_size(8),
      ba::serial_port_base::flow_control flow =
          ba::serial_port_base::flow_control(ba::serial_port_base::flow_control::none),
      ba::serial_port_base::stop_bits stop = ba::serial_port_base::stop_bits(ba::serial_port_base::stop_bits::one));

  bool Active();

 private:
  void CloseCallback(const boost::system::error_code &error);

  void ReadStart();
  void ReadComplete(const boost::system::error_code &error, size_t bytes_transferred);

  void WriteCallback(const std::vector<unsigned char> &msg);
  void WriteStart();
  void WriteComplete(const boost::system::error_code &error);

  std::string device;
  unsigned int baud;
  bool async_active, open;
  std::deque<std::vector<unsigned char> > write_msgs;

  boost::thread thread;
  ba::io_service io_service;
  ba::serial_port *serial_port;
  boost::function<void(const unsigned char *, size_t)> read_callback;

  unsigned char read_msg[MAX_READ_LENGTH];
};
#endif
