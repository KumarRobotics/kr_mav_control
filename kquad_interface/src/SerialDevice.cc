/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
/* Driver for reading data from a serial port

    Aleksandr Kushleyev <kushlik(at)gmail(dot)com>, 2008

*/

#include "SerialDevice.hh"
#include <stdlib.h>
#include <stdio.h>
#include "Timer.hh"
#include <signal.h>
#include <unistd.h>

#include <sstream>

#define SD_VERBOSE
#ifdef SD_VERBOSE
  #define SD_LOG_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg \
    << std::endl << std::flush; std::cout<<msgStream.str(); }
  #define SD_LOG_ERROR( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg \
    << std::endl << std::flush; std::cout<<msgStream.str(); }
#else
  #define SD_LOG_INFO( msg )
  #define SD_LOG_ERROR( msg )
#endif

//constructor
SerialDevice::SerialDevice()
{
  this->fd                = -1;
  this->connected         = false;
  this->block             = -1;
  this->baud              = B2400;

  this->io_mode           = IO_BLOCK_W_TIMEOUT;
  this->delay_us          = 0;
  this->num_term_chars    = 0;
  this->ret_term_sequence = false;
  this->threaded          = 0;
  this->threadRunning     = 0;
  this->pbuf_read         = NULL;
  pthread_mutex_init( &(this->dataMutex) , NULL );
  this->bufsize = 1024;

  this->buf.resize(this->bufsize);
  this->pbuf_head = &(this->buf[0]);
  this->pbuf_now  = this->pbuf_head;
  this->pbuf_tail = this->pbuf_head + this->bufsize;

  this->pbuf_read = this->pbuf_now;
}

//destructor
SerialDevice::~SerialDevice()
{
  Disconnect();
}

//wrapper for ConnectSerial function
int SerialDevice::Connect(const char * device, const int speed, int threaded)
{
  return ConnectSerial(device,speed,threaded);
}

int SerialDevice::Connect(const char * device, const char * speedStr, int threaded)
{
  int speed = strtol(speedStr,NULL,10);
  return ConnectSerial(device,speed,threaded);
}

//connect to the serial device
int SerialDevice::ConnectSerial(const char * device, const int speed, int threaded)
{

  if (this->connected)
  {
    SD_LOG_INFO("Already connected");
    return 0;
  }

  //store the device name
  strncpy(this->device,device,MAX_DEVICE_NAME_LENGTH);

  // Open the device
  //if((this->fd = open(this->device, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
  if((this->fd = open(this->device, O_RDWR | O_NOCTTY)) < 0)
  {
    SD_LOG_ERROR("SerialDevice::ConnectSerial: Error: Unable to open serial port");
    return -1;
  }

  //update the connected flag
  this->connected=true;

  //get the file descriptor information
  fstat(this->fd,&this->fd_stat);

   //set the default IO mode
  if (Set_IO_BLOCK_W_TIMEOUT())
  {
    SD_LOG_ERROR("Unable to set the io mode");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  //set the device type
  this->device_type = DEVICE_TYPE_SERIAL;

  if (speed==0)
    return 0;

  //save current attributes so they can be restored after use
  if( tcgetattr( this->fd, &this->old_term ) < 0 )
  {
   SD_LOG_ERROR("Unable to get old serial port attributes");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  //set up the terminal and set the baud rate
  if (SetBaudRate(speed))
  {
    SD_LOG_ERROR("Unable to set baud rate");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  if (threaded)
  {
    this->_StartThread();
    this->threaded = 1;
  }

  return 0;
}

int SerialDevice::ConnectTCP(const char * device, const int port, const int buff_size, int threaded)
{
  //int buffSize, nonblock=1;
  struct sockaddr_in serv_addr;
  struct hostent *hostptr;

  //check the port
  if ( (port < 0) || (port > 65536) )
  {
    SD_LOG_ERROR("bad port number");
    return -1;
  }

  //get the hostname
  if ((hostptr = gethostbyname(device)) == NULL)
  {
    SD_LOG_ERROR("could not get hostname");
    return -1;
  }

  //Get host info
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy(hostptr->h_addr, (char *) &serv_addr.sin_addr, hostptr->h_length);
  serv_addr.sin_port = htons(port);
  this->port = port;

  if ((this->fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    SD_LOG_ERROR("could not open a socket");
    return -1;
  }

  // Set read buffer size
  if (setsockopt(this->fd, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(int)) < 0)
  {
    SD_LOG_ERROR("could not set receive buffer size");
    close(this->fd);
    return -1;
  }

  //update the connected flag
  this->connected=true;

  //get the file descriptor information
  fstat(this->fd,&this->fd_stat);

  // Set nonblocking I/O so that we can attempt to connect
  if (Set_IO_NONBLOCK_WO_TIMEOUT())
  {
    SD_LOG_ERROR("could not set non-blocking io mode");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  struct timeval selTimeout = {0,DEFAULT_TCP_CONNECT_TIMEOUT_US};

  fd_set wrfds;
  FD_ZERO(&wrfds);
  FD_SET(this->fd, &wrfds);

  connect(this->fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
  int selret = 0;

  selret = select(this->fd+1,NULL,&wrfds,NULL,&selTimeout);


  //make sure that we actually connected by getting the status from getsocketopt
  int option_value;
  int option_length=4;  //need to set this value to 4 to let getsockopt know that we are expecting an int (4 bytes)

  if (getsockopt(this->fd,SOL_SOCKET,SO_ERROR,(char*)&option_value,(socklen_t*)&option_length) != 0)
  {
    SD_LOG_ERROR("could not get socket option");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  //std::cout <<"option length= "<<option_length<<" option value= " << *(int*)option_value<<std::endl;

  if (option_value != 0)
  {
    SD_LOG_ERROR("could not connect to the server:");
    SD_LOG_ERROR(strerror(option_value));

    close(this->fd);
    this->connected=false;
    return -1;
  }

  if (selret <= 0)
  {
    SD_LOG_ERROR("could not connect to the server");
    close(this->fd);
    this->connected=false;
    return -1;
  }


  //set the default IO mode
  if (Set_IO_BLOCK_W_TIMEOUT())
  {
    SD_LOG_ERROR("Unable to set the io mode");
    close(this->fd);
    this->connected=false;
    return -1;
  }

  this->device_type = DEVICE_TYPE_TCP;

  if (threaded)
  {
    this->_StartThread();
    this->threaded = 1;
  }

  return 0;
}


//disconnect from the device
int SerialDevice::Disconnect()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    return 0;
  }

  if (this->device_type == DEVICE_TYPE_SERIAL)
  {
    // Restore old terminal settings
    if(tcsetattr(this->fd,TCSANOW,&this->old_term) < 0)
    {
      SD_LOG_ERROR("Failed to restore attributes!");
    }
  }

  // Actually close the device
  if(close(this->fd) != 0)
  {
    SD_LOG_ERROR("Failed to close device!");
    this->connected=false;
    return -1;
  }

  this->connected=false;

  this->_StopThread();
  return 0;
}

bool SerialDevice::IsConnected()
{
  return this->connected;
}


//convert speed (integer) to baud rate (speed_t)
int SerialDevice::_SpeedToBaud(int speed, speed_t & baud)
{
  switch (speed)
  {
    case 2400:
      baud=B2400;
      return 0;
    case 4800:
      baud=B4800;
      return 0;
    case 9600:
      baud=B9600;
      return 0;
    case 19200:
      baud=B19200;
      return 0;
    case 38400:
      baud=B38400;
      return 0;
    case 57600:
      baud=B57600;
      return 0;
    case 115200:
      baud=B115200;
      return 0;
    case 230400:
      baud=B230400;
      return 0;
#ifndef __APPLE__
    case 460800:
      baud=B460800;
      return 0;
    case 921600:
      baud=B921600;
      return 0;
    case 1000000:
      baud=B1000000;
      return 0;
    case 2000000:
      baud=B2000000;
      return 0;
#endif

    default:
      SD_LOG_ERROR("unknown baud rate");
      return -1;
  }
}

int SerialDevice::SetStandardBaudRate(speed_t baud)
{
  //make sure the custom baud rate flag is off

#ifndef __APPLE__
  //Get serial port info
  if (ioctl(this->fd, TIOCGSERIAL, &(this->serial)) < 0)
  {
    SD_LOG_ERROR("ioctl() failed while trying to get serial port info");
  }
  //make sure that custom speed option is turned off
  else
  {
    this->serial.flags         &= ~ASYNC_SPD_CUST;
    this->serial.flags         |= ASYNC_LOW_LATENCY;  //will set ftdi delay to 1ms
    this->serial.custom_divisor = 0;
    if (ioctl(this->fd, TIOCSSERIAL, &(this->serial)) < 0)
      SD_LOG_ERROR("ioctl() failed while trying to set serial port info");
  }
#endif

  //get current port settings
  if( tcgetattr( this->fd, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to get serial port attributes");
    return -1;
  }

  //cfmakeraw initializes the port to standard configuration. Use this!
  cfmakeraw( &this->new_term );

  //set input baud rate
  if (cfsetispeed( &this->new_term, baud ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set output baud rate
  if (cfsetospeed( &this->new_term, baud ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set new attributes
  if( tcsetattr( this->fd, TCSAFLUSH, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to set serial port attributes");
    return -1;
  }

  //make sure queue is empty
  tcflush(this->fd, TCIOFLUSH);

  //save the baud rate value
  this->baud=baud;

  return 0;
}

#ifndef __APPLE__
int SerialDevice::SetNonStandardBaudRate(int speed)
{
  if (speed < 300)
  {
    SD_LOG_ERROR("bad desired speed");
    return -1;
  }

  printf("setting nonstandard baud rate %d\n",speed);

  //Get serial port info
  if (ioctl(this->fd, TIOCGSERIAL, &(this->serial)) < 0)
  {
    SD_LOG_ERROR("ioctl() failed while trying to get serial port info");
  }
  //make sure that custom speed option is turned off
  else
  {
    this->serial.flags         |= ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
    this->serial.custom_divisor = 24000000/speed;
    if (ioctl(this->fd, TIOCSSERIAL, &(this->serial)) < 0)
      SD_LOG_ERROR("ioctl() failed while trying to set serial port info");
  }

  //get current port settings
  if( tcgetattr( this->fd, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to get serial port attributes");
    return -1;
  }

  //cfmakeraw initializes the port to standard configuration. Use this!
  cfmakeraw( &this->new_term );

  //set input baud rate (B38400 is for custom)
  if (cfsetispeed( &this->new_term, B38400 ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set output baud rate (B38400 is for custom)
  if (cfsetospeed( &this->new_term, B38400 ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set new attributes
  if( tcsetattr( this->fd, TCSAFLUSH, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to set serial port attributes");
    return -1;
  }

  //make sure queue is empty
  tcflush(this->fd, TCIOFLUSH);

  //save the baud rate value
  this->baud=speed;

  return 0;
}

#else
int SerialDevice::SetNonStandardBaudRate(int speed)
{
  if (speed < 300)
  {
    SD_LOG_ERROR("bad desired speed");
    return -1;
  }

  printf("setting nonstandard baud rate %d\n",speed);

  //get current port settings
  if( tcgetattr( this->fd, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to get serial port attributes");
    return -1;
  }

  //cfmakeraw initializes the port to standard configuration. Use this!
  cfmakeraw( &this->new_term );

  //set input baud rate (does not matter since IOSSIOSPEED will override)
  if (cfsetispeed( &this->new_term, B38400 ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set input baud rate (does not matter since IOSSIOSPEED will override)
  if (cfsetospeed( &this->new_term, B38400 ) < 0 )
  {
    SD_LOG_ERROR("Unable to set baud rate");
    return -1;
  }

  //set new attributes
  if( tcsetattr( this->fd, TCSAFLUSH, &this->new_term ) < 0 )
  {
    SD_LOG_ERROR("Unable to set serial port attributes");
    return -1;
  }

  //make sure queue is empty
  tcflush(this->fd, TCIOFLUSH);


  //http://developer.apple.com/library/mac/#samplecode/SerialPortSample/Introduction/Intro.html
  //set the custom speed using ioctl (osx-specifict)
  speed_t spd = speed;
  if (ioctl(this->fd, IOSSIOSPEED, &spd) < 0)
  {
    SD_LOG_ERROR("could not set custom speed");
    return -1;
  }


  //set the custom latency using ioctl (osx-specifict)
  unsigned long mics = 1000;

  if (ioctl(this->fd, IOSSDATALAT, &mics) < 0)
  {
    SD_LOG_ERROR("could not set latency");
    return -1;
  }

  //save the baud rate value
  this->baud=speed;

  return 0;
}
#endif


//set the terminal baud rate. Argument can be either an integer (ex. 115200) or speed_t (ex. B115200)
int SerialDevice::SetBaudRate(const int speed)
{
  speed_t tempBaud;

  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  //convert the integer speed value to speed_t if needed
  int nonStandardBaud = _SpeedToBaud(speed,tempBaud);
  if (nonStandardBaud == 0)
  {
    if (SetStandardBaudRate(tempBaud) != 0)
    {
      SD_LOG_ERROR("could not set standard baud rate");
      return -1;
    }
  }
  else
  {
    if (SetNonStandardBaudRate(speed) != 0)
    {
      SD_LOG_ERROR("could not set non-standard baud rate");
      return -1;
    }
  }

  return 0;
}


int SerialDevice::Set_IO_BLOCK_W_TIMEOUT()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (_SetBlockingIO())
  {
    SD_LOG_ERROR("could not set blocking io");
    return -1;
  }
  this->io_mode=IO_BLOCK_W_TIMEOUT;
  this->delay_us=0;
  this->num_term_chars=0;
  this->ret_term_sequence=false;

  return 0;
}

int SerialDevice::Set_IO_BLOCK_W_TIMEOUT_AT_LEAST_ONE_CHAR()
{
  int ret = this->Set_IO_BLOCK_W_TIMEOUT();
  this->io_mode = IO_BLOCK_W_TIMEOUT_AT_LEAST_ONE_CHAR;

  return ret;
}


int SerialDevice::Set_IO_BLOCK_WO_TIMEOUT()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (_SetBlockingIO())
  {
    SD_LOG_ERROR("could not set blocking io");
    return -1;
  }
  this->io_mode=IO_BLOCK_WO_TIMEOUT;
  this->delay_us=0;
  this->num_term_chars=0;
  this->ret_term_sequence=false;

  return 0;
}

int SerialDevice::Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE(const char * termSequence, int numTermChars, bool retTermSequence)
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (_SetBlockingIO())
  {
    SD_LOG_ERROR("could not set blocking io");
    return -1;
  }

  if (numTermChars < 1 || numTermChars > MAX_NUM_TERM_CHARS)
  {
    SD_LOG_ERROR("bad number of chars: " <<numTermChars);
    return -1;
  }
  this->io_mode=IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE;
  this->num_term_chars=numTermChars;
  this->ret_term_sequence=retTermSequence;
  memcpy(this->term_sequence,termSequence, numTermChars*sizeof(char));
  this->delay_us=0;

  return 0;
}

int SerialDevice::Set_IO_NONBLOCK_WO_TIMEOUT()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (_SetNonBlockingIO())
  {
    SD_LOG_ERROR("could not set non-blocking io");
    return -1;
  }

  this->io_mode=IO_NONBLOCK_WO_TIMEOUT;
  this->delay_us=0;
  this->num_term_chars=0;
  this->ret_term_sequence=false;

  return 0;
}

int SerialDevice::Set_IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT(int delay_us)
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (_SetNonBlockingIO())
  {
    SD_LOG_ERROR("could not set non-blocking io");
    return -1;
  }

  this->io_mode=IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT;
  this->delay_us=delay_us;
  this->num_term_chars=0;
  this->ret_term_sequence=false;

  return 0;
}


//set blocking terminal mode
int SerialDevice::_SetBlockingIO()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  //only set blocking if not already set
  if (this->block != 1)
  {
    // Read the flags
    int flags;
    if((flags = fcntl(this->fd,F_GETFL)) < 0)
    {
      SD_LOG_ERROR("unable to get device flags");
      return -1;
    }

    // Set the new flags
    if(fcntl(this->fd,F_SETFL,flags & (~O_NONBLOCK)) < 0)
    {
      SD_LOG_ERROR("unable to set device flags");
      return -1;
    }

    this->block=1;
  }
  return 0;
}

//set non-blocking terminal mode
int SerialDevice::_SetNonBlockingIO()
{
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  //only set non-blocking if not already set
  if (this->block != 0)
  {
    // Read the flags
    int flags;
    if((flags = fcntl(this->fd,F_GETFL)) < 0)
    {
      SD_LOG_ERROR("unable to retrieve device flags");
      return -1;
    }

    // Set the new flags
    if(fcntl(this->fd,F_SETFL,flags | O_NONBLOCK) < 0)
    {
      SD_LOG_ERROR("unable to set device flags");
      return -1;
    }

    this->block=0;
  }
  return 0;
}

//delete all the data in the input buffer
int SerialDevice::FlushInputBuffer()
{
  char c[1000];
  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  //TODO tcflush for some reason does not work..
  //tcflush(this->fd, TCIFLUSH);

  int block=this->block;
  _SetNonBlockingIO();

  //read off all the chars
  while (read(this->fd,c,1000) > 0){}

  if (block==1)
  {
    _SetBlockingIO();
  }

  return 0;
}

//read characters from device
int SerialDevice::ReadChars(char * data, int byte_count, int timeout_us)
{

  double tEntry  = Timer::GetUnixTime();
  double tWait   = timeout_us / 1000000.0;
  double tFinish = tEntry + tWait;

  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

//TODO: tcp behaves strangely when connection is closed. The check below does not work
/*
  struct stat fd_info;
  if (fstat(this->fd,&fd_info) != 0 )
  {
#ifdef SERIAL_DEVICE_DEBUG
    std::cout << "SerialDevice::ReadChars: Error: file descriptor is not valid" << std::endl;
#endif
    return -1;
  }
*/
  fd_set watched_fds;
  struct timeval timeout, start, end;
  int bytes_read_total = 0;
  int bytes_left = byte_count;
  int retval;
  int bytes_read;
  int charsMatched=0;
  //double waitTime;
  //double timeLeft;

  struct stat fd_stat;
  fstat(this->fd,&fd_stat);

  if (fd_stat.st_mode != this->fd_stat.st_mode)
  {
    SD_LOG_INFO("device disconnected");
    SD_LOG_ERROR("device disconnected???");
    usleep(timeout_us);  //sleep to avoid high cpu usage
    return -1;
  }

  switch (this->io_mode)
  {

    case IO_BLOCK_W_TIMEOUT:
    case IO_BLOCK_W_TIMEOUT_AT_LEAST_ONE_CHAR:

      while (bytes_left > 0)
      {
        //set up for the "select" call
        FD_ZERO(&watched_fds);
        FD_SET(this->fd, &watched_fds);
        double tNow = Timer::GetUnixTime();
        double dt = tFinish - tNow;

        if (dt < 0)
          return bytes_read_total;

        timeout.tv_sec = (int)dt;
        timeout.tv_usec = (dt-timeout.tv_sec)*1000000.0;

        if ((retval = select(this->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1)   //block until at least 1 char is available or timeout
        {                                                                         //error reading chars
          if (retval < 0)
          {
            SD_LOG_ERROR("select call failed");
            ///perror("SerialDevice::ReadChars");
          }
          else                                                                    //timeout
          {
            //SD_LOG_ERROR("timeout. #chars read= "<<bytes_read_total <<", requested= "<< byte_count);
          }
        }
        else
        {
          bytes_read        = read(this->fd, &(data[bytes_read_total]), bytes_left);

          if (bytes_read > 0)
          {
            bytes_read_total += bytes_read;
            bytes_left       -= bytes_read;

            if (this->io_mode == IO_BLOCK_W_TIMEOUT_AT_LEAST_ONE_CHAR)
              return bytes_read_total;
          }
        }
      }
      return bytes_read_total;

    case IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT:
      gettimeofday(&start,NULL);

      while (bytes_left)
      {
        bytes_read = read(this->fd,&(data[bytes_read_total]),bytes_left);
        if ( bytes_read < 1)
        {
          // If a time out then return false
          gettimeofday(&end,NULL);
          if((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec) > timeout_us)
          {
            SD_LOG_ERROR("timeout. #chars read= "<<bytes_read_total <<", requested= "<< byte_count);
            return bytes_read_total;
          }
          usleep(this->delay_us);
          continue;
        }

        bytes_read_total += bytes_read;
        bytes_left       -= bytes_read;
      }
      return bytes_read_total;


    case IO_BLOCK_WO_TIMEOUT:
      while (bytes_left)
      {
        bytes_read = read(this->fd,&(data[bytes_read_total]),bytes_left);
        if (bytes_read < 1)
        {
          return -1;
        }

        bytes_read_total += bytes_read;
        bytes_left       -= bytes_read;
      }
      return bytes_read_total;

    case IO_NONBLOCK_WO_TIMEOUT:
      bytes_read = read(this->fd,&(data[0]),bytes_left);
      if (bytes_read < 0) bytes_read=0;
      return bytes_read;


    case IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE:

      while (bytes_left > 0)
      {
        //set up for the "select" call
        FD_ZERO(&watched_fds);
        FD_SET(this->fd, &watched_fds);
        double tNow = Timer::GetUnixTime();
        double dt = tFinish - tNow;

        if (dt < 0)
          return -1;

        timeout.tv_sec = (int)dt;
        timeout.tv_usec = (dt-timeout.tv_sec)*1000000.0;

        if ((retval = select(this->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1)   //block until at least 1 char is available or timeout
        {                                                                         //error reading chars
          if (retval < 0)
          {
            SD_LOG_ERROR("select call failed");
            //perror("SerialDevice::ReadChars");
          }
          else                                                                    //timeout
          {
            SD_LOG_ERROR("timeout. The terminating sequence has not been read");
          }
        }
        bytes_read = read(this->fd, &(data[bytes_read_total]), 1);

        if (bytes_read==1)
        {
          if (data[bytes_read_total]==this->term_sequence[charsMatched])
          {
            charsMatched++;
          }
          else
          {
            charsMatched=0;
          }

          //std::cout<<data[bytes_read_total];
          bytes_read_total += bytes_read;
          bytes_left       -= bytes_read;

          if (charsMatched==this->num_term_chars)
          {
            if (this->ret_term_sequence)
            {
              return bytes_read_total;
            }

            else
            {
              return bytes_read_total-this->num_term_chars;
            }
          }
        }
      }

      SD_LOG_ERROR("Read too much data. The terminating sequence has not been read");
      return -1;

    default:
      SD_LOG_ERROR("Bad io mode");
      return -1;

  }
}

int SerialDevice::WriteChars(const char * data, int byte_count, int delay_us)
{
  int bytes_written_total=0;
  int bytes_written;
  int bytes_left=byte_count;


  //check whether we are connected to the device
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected to the device");
    return -1;
  }

  if (delay_us==0)
  {
    bytes_written_total=write(this->fd,data,byte_count);
  }

  else
  {
    while (bytes_left)
    {
      bytes_written=write(this->fd,&(data[bytes_written_total]),1);
      if (bytes_written < 0)
      {
        SD_LOG_ERROR("error writing");
        //perror("SerialDevice::ReadChars");
      }
      if (bytes_written < 1)
      {
        SD_LOG_ERROR("could not write a char. #chars written= "<<bytes_written_total <<", requested= "<< byte_count);
        return bytes_written_total;
      }

      bytes_written_total += bytes_written;
      bytes_left       -= bytes_written;
      usleep(delay_us);
    }
  }

  //tcdrain(this->fd);   //wait till all the data written to the file descriptor is transmitted

  return bytes_written_total;
}


int SerialDevice::_StartThread()
{
  if (!this->connected)
  {
    SD_LOG_ERROR("not connected\n");
    return -1;
  }

  if (this->threadRunning) return 0;  //already running

  SD_LOG_INFO("Starting thread...");
  if (pthread_create(&this->thread, NULL, this->ThreadFunc, (void *)this))
  {
    SD_LOG_ERROR("Could not start thread\n");
    return -1;
  }
  SD_LOG_INFO("done\n");

  this->threadRunning=1;

  return 0;
}

int SerialDevice::_StopThread()
{
  if (this->threadRunning)
  {
    SD_LOG_INFO("Stopping thread...");
    pthread_cancel(this->thread);
    pthread_join(this->thread,NULL);
    SD_LOG_INFO("done\n");
    this->threadRunning=false;
  }

  return 0;
}

int SerialDevice::_LockDataMutex()
{
  return pthread_mutex_lock( &this->dataMutex );
}

int SerialDevice::_UnlockDataMutex()
{
  return pthread_mutex_unlock( &this->dataMutex );
}

int SerialDevice::ReadCharsThreaded(char * data, int byte_count, int timeout_us)
{
  //usleep(timeout_us);
  int ret = 0;

  this->_LockDataMutex();
  if (this->pbuf_read < this->pbuf_now)
  {
    int64_t sizea = this->pbuf_now - this->pbuf_read;
    int64_t sizec = byte_count < sizea ? byte_count : sizea;

    //printf("about to copy %d bytes\n",sizec);

    memcpy(data,this->pbuf_read,sizec);
    this->pbuf_read += sizec;
    if (this->pbuf_read >= this->pbuf_tail)
      this->pbuf_read = this->pbuf_head;

    ret = sizec;
  }

  else if (this->pbuf_read > this->pbuf_now)
  {
    //overflow
    int64_t sizea = this->pbuf_tail - this->pbuf_read;
    int64_t sizec = byte_count < sizea ? byte_count : sizea;

    //printf("about to copy %d bytes\n",sizec);

    memcpy(data,this->pbuf_read,sizec);
    this->pbuf_read += sizec;

    if (this->pbuf_read >= this->pbuf_tail)
      this->pbuf_read = this->pbuf_head;

    ret = sizec;
  }


  this->_UnlockDataMutex();

  return ret;
}

void *SerialDevice::ThreadFunc(void * arg_in)
{
  sigset_t sigs;
  sigfillset(&sigs);
  pthread_sigmask(SIG_BLOCK,&sigs,NULL);

  SerialDevice * sd = (SerialDevice *) arg_in;

  int timeoutUs = 1000;

  while(1)
  {
    pthread_testcancel();

    int nmax = sd->pbuf_tail - sd->pbuf_now;
    if (nmax > 256)
      nmax = 256;

    int nchars = sd->ReadChars((char*)sd->pbuf_now,nmax,timeoutUs);

    //printf("nchars = %d\n",nchars);

    if (nchars>0)
    {
      sd->_LockDataMutex();
      sd->pbuf_now+= nchars;
      if (sd->pbuf_now >= sd->pbuf_tail)
      {
        sd->pbuf_now = sd->pbuf_head;
        //printf("overflow\n");
      }
      sd->_UnlockDataMutex();
      //for (int ii=0; ii<nchars; ii++)
      //  printf(".");
    }
    fflush(stdout);
  }

  return NULL;
}

