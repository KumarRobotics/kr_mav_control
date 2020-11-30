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

#include <kr_serial_interface/ASIOSerialDevice.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif

using namespace std;

ASIOSerialDevice::ASIOSerialDevice()
{
  async_active = false;
  open = false;
  serial_port = 0;
}

ASIOSerialDevice::ASIOSerialDevice(const string &device, unsigned int baud)
{
  async_active = false;
  open = false;
  serial_port = 0;

  Open(device, baud);
}

ASIOSerialDevice::~ASIOSerialDevice()
{
  if(open)
    Close();

  if(async_active)
    Stop();

  if(serial_port != 0)
    delete serial_port;
}

void ASIOSerialDevice::Open(const string &device_, unsigned int baud_, ba::serial_port_base::parity parity,
                            ba::serial_port_base::character_size csize, ba::serial_port_base::flow_control flow,
                            ba::serial_port_base::stop_bits stop)
{
  device = device_;
  baud = baud_;

  if(!open)
  {
    try
    {
      serial_port = new ba::serial_port(io_service, device);
    }
    catch(const std::exception &e)
    {
      cerr << "Unable to open device: " << device << endl;
      throw;
    }

    if(!serial_port->is_open())
      throw runtime_error("Failed to open serial port");

    ba::serial_port_base::baud_rate baud(baud_);
    serial_port->set_option(baud);

    serial_port->set_option(parity);
    serial_port->set_option(csize);
    serial_port->set_option(flow);
    serial_port->set_option(stop);

#if defined(__linux__)
    int fd = serial_port->native_handle();

    // Enable low latency mode on Linux
    struct serial_struct ser_info;
    ioctl(fd, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ser_info);

#if BOOST_ASIO_VERSION < 101200
    // This is done in Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a
    // bug which doesn't enable relevant code. Fixed by commit:
    // https://github.com/boostorg/asio/commit/619cea4356
    termios tio;
    tcgetattr(fd, &tio);

    // Set serial port to "raw" mode to prevent EOF exit.
    cfmakeraw(&tio);

    // Commit settings
    tcsetattr(fd, TCSANOW, &tio);
#endif
#endif

    open = true;
  }
}

void ASIOSerialDevice::Close()
{
  if(open)
  {
    if(async_active)
      io_service.post(boost::bind(&ASIOSerialDevice::CloseCallback, this, boost::system::error_code()));
    else
      CloseCallback(boost::system::error_code());
  }
}

void ASIOSerialDevice::Start()
{
  if(!open)
    throw runtime_error("Serial port interface not open");

  ReadStart();

  thread = boost::thread(boost::bind(&ba::io_service::run, &io_service));

  async_active = true;

  return;
}

void ASIOSerialDevice::ReadStart()
{
  if(open)
    serial_port->async_read_some(ba::buffer(read_msg, MAX_READ_LENGTH),
                                 boost::bind(&ASIOSerialDevice::ReadComplete, this, ba::placeholders::error,
                                             ba::placeholders::bytes_transferred));
}

void ASIOSerialDevice::ReadComplete(const boost::system::error_code &error, size_t bytes_transferred)
{
  if(!error)
  {
    if(!read_callback.empty())
      read_callback(read_msg, bytes_transferred);
    ReadStart();
  }
  else
    CloseCallback(error);
}

void ASIOSerialDevice::SetReadCallback(const boost::function<void(const unsigned char *, size_t)> &handler)
{
  read_callback = handler;
}

void ASIOSerialDevice::Stop()
{
  thread.join();
  async_active = false;
}

void ASIOSerialDevice::CloseCallback(const boost::system::error_code &error)
{
  if(error && (error != ba::error::operation_aborted))
    cerr << "Error: " << error.message() << endl;

  serial_port->close();
  open = false;
}

bool ASIOSerialDevice::Write(const vector<unsigned char> &msg)
{
  if(!open)
    return false;

  if(async_active)
    // Post for an asynchronous write
    io_service.post(boost::bind(&ASIOSerialDevice::WriteCallback, this, msg));
  else
    // Write synchronously
    ba::write(*serial_port, ba::buffer(&(msg[0]), msg.size()));

  return true;
}

void ASIOSerialDevice::WriteCallback(const vector<unsigned char> &msg)
{
  bool write_in_progress = !write_msgs.empty();
  write_msgs.push_back(msg);
  if(!write_in_progress)
    WriteStart();
}

void ASIOSerialDevice::WriteStart()
{
  ba::async_write(*serial_port, ba::buffer(&(write_msgs.front()[0]), write_msgs.front().size()),
                  boost::bind(&ASIOSerialDevice::WriteComplete, this, ba::placeholders::error));
}

void ASIOSerialDevice::WriteComplete(const boost::system::error_code &error)
{
  if(!error)
  {
    write_msgs.pop_front();
    if(!write_msgs.empty())
      WriteStart();
  }
  else
    CloseCallback(error);
}

bool ASIOSerialDevice::Active()
{
  return async_active;
}

void ASIOSerialDevice::Read()
{
  if(async_active)
  {
    cerr << "ASIOSerialDevice can operate in async or sync modes, not both" << endl;
    return;
  }

  size_t bytes_transferred = serial_port->read_some(ba::buffer(read_msg, MAX_READ_LENGTH));

  if(!read_callback.empty() && (bytes_transferred > 0))
    read_callback(read_msg, bytes_transferred);
}
