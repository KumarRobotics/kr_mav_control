/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
/*
  Author: Alex Kushleyev, KMel Robotics,  2012

  LOG:

  11/1/2012
  - added SetMaxQueueLength function, which was not implemented
  - added accessor fucnctions for the data


  11/21/2012
  - added new data type for imu feedback (smaller packet)
  - added yaw wrapping before sending

*/
#include <iostream>
#include <fstream>
#include "kQuadInterface.hh"
#include "kQuadCom.h"
#include "Timer.hh"
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include "kBotPacket2.h"

#define CHECK_CONNECTION if (this->connected != 1) { PRINT_ERROR("NOT CONNECTED!!\n"); return -1; }

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Constructor
//_______________________________________________________
kQuadInterface::kQuadInterface()
{
  this->connected           = 0;
  this->sendThreadRunning   = 0;
  this->recvThreadRunning   = 0;

  this->cmdOutputTimer.Tic();

  kBotPacket2Init(&(this->packet));
  pthread_mutex_init( &(this->dataMutex) , NULL );
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Destructor
//_______________________________________________________
kQuadInterface::~kQuadInterface()
{
  this->StopSendThread();
  this->StopRecvThread();
  this->Disconnect();
  pthread_mutex_destroy(&(this->dataMutex));
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Connect to the serial port
//_______________________________________________________
int kQuadInterface::Connect(string dev,int baud)
{
  if (this->connected)
    return 0;

  //connect to the device
  int ret = this->sd.Connect(dev.c_str(),baud);

  //set the io mode to either block for specified time or return
  //if at least one char is avaialbe when calling ReadChars
  this->sd.Set_IO_BLOCK_W_TIMEOUT_AT_LEAST_ONE_CHAR();

  if    (ret == 0) this->connected = 1;
  else  this->connected = 0;

  return ret;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Disconnect from serial port
//_______________________________________________________
int kQuadInterface::Disconnect()
{
  int ret = this->sd.Disconnect();

  if (ret == 0) this->connected = 0;

  return ret;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Start the internal thread (after connecting)
//_______________________________________________________
int kQuadInterface::StartSendThread()
{
  if (!this->connected)
  {
    PRINT_ERROR("not connected\n");
    return -1;
  }

  if (this->sendThreadRunning)
  {
    printf("send thread is already running\n");
    return 0;  //already running
  }

  PRINT_INFO("Starting send thread...");
  if (pthread_create(&this->sendThread, NULL, this->SendThreadFunc, (void *)this))
  {
    PRINT_ERROR("Could not start thread\n");
    return -1;
  }

  PRINT_INFO("done\n");

  this->sendThreadRunning = 1;
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Start the internal thread (after connecting)
//_______________________________________________________
int kQuadInterface::StartRecvThread()
{
  if (!this->connected)
  {
    PRINT_ERROR("not connected\n");
    return -1;
  }

  if (this->recvThreadRunning)
  {
    PRINT_INFO("receive thread is already running..\n");
    return 0;  //already running
  }

  PRINT_INFO("Starting receive thread...");
  if (pthread_create(&this->recvThread, NULL, this->RecvThreadFunc, (void *)this))
  {
    PRINT_ERROR("Could not start thread\n");
    return -1;
  }

  PRINT_INFO("done\n");

  this->recvThreadRunning = 1;
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Stop the internal thread
//_______________________________________________________
int kQuadInterface::StopSendThread()
{
  if (this->sendThreadRunning)
  {
    PRINT_INFO("Stopping send thread...");
    pthread_cancel(this->sendThread);
    pthread_join(this->sendThread,NULL);

    PRINT_INFO("done\n");
    this->sendThreadRunning = 0;
  }

  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Stop the internal thread
//_______________________________________________________
int kQuadInterface::StopRecvThread()
{
  if (this->recvThreadRunning)
  {
    PRINT_INFO("Stopping receive thread...");
    pthread_cancel(this->recvThread);
    pthread_join(this->recvThread,NULL);

    PRINT_INFO("done\n");
    this->recvThreadRunning = 0;
  }

  return 0;
}

void *kQuadInterface::RecvThreadFunc(void * arg_in)
{
  sigset_t sigs;
  sigfillset(&sigs);
  pthread_sigmask(SIG_BLOCK,&sigs,NULL);

  kQuadInterface * kqi = (kQuadInterface *) arg_in;

  kBotPacket2 ipacket;
  const int timeoutUs = 1000;

  uint8_t buf[1024];

  while(1)
  {
     pthread_testcancel();
     int ret = kqi->kparser.ProcessBuffer(&ipacket);
     while (ret > 0)
     {
       kqi->ProcessIncomingPacket(&ipacket);
       ret = kqi->kparser.ProcessBuffer(&ipacket);
     }

     //read up to bufSize bytes, but returning as soon as at least one
     //char is available, or timeout
     int nchars = kqi->sd.ReadChars((char*)buf,128,timeoutUs);
     if (nchars > 0)
     {
       kqi->kparser.PushData(buf,nchars);
     }
  }

  return NULL;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Internal thread function. Continously handles IO
//_______________________________________________________
void *kQuadInterface::SendThreadFunc(void * arg_in)
{
  sigset_t sigs;
  sigfillset(&sigs);
  pthread_sigmask(SIG_BLOCK,&sigs,NULL);

  kQuadInterface * kqi = (kQuadInterface *) arg_in;

  while(1)
  {
    pthread_testcancel();
    kqi->LockDataMutex();

    int locked = 1;


    //kbee messages have priority
    int nkbee = kqi->kbeeData.size();
    if (nkbee > 0)
    {
      kQuadControlData kb =  kqi->kbeeData.front();
      kqi->kbeeData.pop_front();
      kqi->UnlockDataMutex();
      locked = 0;

      if (kb.size < 120)
      {
        if (kqi->sd.WriteChars((char*)kb.data,kb.size) != kb.size)
        {
          PRINT_ERROR("could not write data\n");
        }
      }
      else
      {
        PRINT_ERROR("data payload too long.. dropping!!\n");
      }
    }



    //now check the command messages
    if (locked == 0)
      kqi->LockDataMutex();

    int nlist = kqi->controlData.size();
    if (nlist > 0)
    {
      //extract the next packet and pop it off
      kQuadControlData cntrl =  kqi->controlData.front();
      kqi->controlData.pop_front();

      kqi->UnlockDataMutex();

      //send the data if the size is ok
      if (cntrl.size < 120)
      {
        if (kqi->sd.WriteChars((char*)cntrl.data,cntrl.size) != cntrl.size)
        {
          PRINT_ERROR("could not write data\n");
        }
        //printf("!"); fflush(stdout);
      }
      else
      {
        PRINT_ERROR("data payload too long.. dropping!!\n");
      }

      //wait for the data to be transmitted by kbee
      usleep(2500);
    }
    else
    {
      kqi->UnlockDataMutex();
      usleep(1000);
    }
  }

  return NULL;
}

int   kQuadInterface::PackQuadCmd1(uint8_t id, uint8_t quadType, uint8_t channel,
                                   float thrust, float roll, float pitch, float yaw,
                                   uint8_t * dst, int dstLen)
{
  float MAX_THRUST_GRAMS, MAX_ROLL, MAX_PITCH, SCALE_THRUST, SCALE_RPY;
  uint8_t msgType;

  if (quadType == KQUAD_TYPE_STANDARD)
  {
    MAX_THRUST_GRAMS = 32760;
    MAX_ROLL         = DegToRad(90.0);
    MAX_PITCH        = DegToRad(90.0);
    SCALE_THRUST     = 1.0;
    SCALE_RPY        = 1000.0;
    msgType          = KQUAD_LL_STD_CMD1;
  }
  else if (quadType == KQUAD_TYPE_NANO)
  {
    MAX_THRUST_GRAMS = 3276;
    MAX_ROLL         = DegToRad(90.0);
    MAX_PITCH        = DegToRad(90.0);
    SCALE_THRUST     = 10.0;
    SCALE_RPY        = 1000.0;
    msgType          = KQUAD_LL_NANO_CMD1;
  }
  else
  {
    PRINT_ERROR("bad quad type:" << quadType<<"\n");
    return -1;
  }

  if ( (roll > MAX_ROLL) || (roll < -MAX_ROLL) || (pitch > MAX_PITCH) || (pitch < -MAX_PITCH) )
  {
    PRINT_ERROR("roll or pitch values are beyond min or max limits\n");
    return -1;
  }

  //check the data
  if (thrust > MAX_THRUST_GRAMS)
  {
    thrust = MAX_THRUST_GRAMS;
    PRINT_ERROR("thrust is above maximum, so it's being capped\n");
  }

  int bufLen = sizeof(QuadCmd1) + KBOT_PACKET2_OVERHEAD;
  if (dstLen < bufLen)
  {
    PRINT_ERROR("not enough space in the provided buffer for the packet\n");
    return -1;
  }

  //wrap the yaw to -pi .. pi
  yaw = Mod2PiF(yaw);

  //pack the data structure
  QuadCmd1 cmd;
  cmd.id     = id;
  cmd.chan   = channel;
  cmd.cntr   = 0;
  cmd.thrust = thrust * SCALE_THRUST;
  cmd.roll   = roll   * SCALE_RPY;
  cmd.pitch  = pitch  * SCALE_RPY;
  cmd.yaw    = yaw    * SCALE_RPY;


  //wrap the data into the packet
  int len = kBotPacket2WrapData(KQUAD_LL_DEVICE_ID, msgType,
                                (uint8_t*)&cmd, sizeof(cmd),
                                dst, dstLen);

  return len;
}



int   kQuadInterface::PackQuadCmd2(uint8_t id, uint8_t quadType, uint8_t channel,
                                   int16_t rpm0, int16_t rpm1,
                                   int16_t rpm2, int16_t rpm3,
                                   uint8_t * dst, int dstLen)
{

  int bufLen = sizeof(QuadCmd2) + KBOT_PACKET2_OVERHEAD;
  if (dstLen < bufLen)
  {
    PRINT_ERROR("not enough space in the provided buffer for the packet\n");
    return -1;
  }

  //check the data
  uint16_t MAX_RPM = 16000;
  if (rpm0 > MAX_RPM) { rpm0 = MAX_RPM; PRINT_ERROR("capping rpm!!\n"); }
  if (rpm1 > MAX_RPM) { rpm1 = MAX_RPM; PRINT_ERROR("capping rpm!!\n"); }
  if (rpm2 > MAX_RPM) { rpm2 = MAX_RPM; PRINT_ERROR("capping rpm!!\n"); }
  if (rpm3 > MAX_RPM) { rpm3 = MAX_RPM; PRINT_ERROR("capping rpm!!\n"); }

  //pack the data structure
  QuadCmd2 cmd;
  cmd.id   = id;
  cmd.chan = channel;
  cmd.cntr = 0;
  cmd.rpm0 = rpm0;
  cmd.rpm1 = rpm1;
  cmd.rpm2 = rpm2;
  cmd.rpm3 = rpm3;

  //assign the message type based on quad type
  uint8_t msgType;
  if      (quadType == KQUAD_TYPE_STANDARD) msgType = KQUAD_LL_STD_CMD2;
  else if (quadType == KQUAD_TYPE_NANO)     msgType = KQUAD_LL_NANO_CMD2;
  else
  {
    PRINT_ERROR("bad quad type:" << quadType<<"\n");
    return -1;
  }

  //wrap the data into the packet
  int len = kBotPacket2WrapData(KQUAD_LL_DEVICE_ID, msgType,
                                (uint8_t*)&cmd, sizeof(cmd),
                                dst, dstLen);

  return len;
}

int   kQuadInterface::PackQuadCmd3(uint8_t id, uint8_t quadType, uint8_t channel, float thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, uint8_t * dst, int dstLen)
{
  //check for the buffer lenght
  int bufLen = sizeof(QuadCmd3) + KBOT_PACKET2_OVERHEAD;
  if (dstLen < bufLen)
  {
    PRINT_ERROR("not enough space in the provided buffer for the packet\n");
    return -1;
  }


  float MAX_THRUST_GRAMS, MAX_ROLL, MAX_PITCH, MAX_RATE_RAD, MAX_GAIN_P, MAX_GAIN_D;
  float SCALE_THRUST, SCALE_RATE, SCALE_GAIN_P, SCALE_GAIN_D, SCALE_RPY;
  uint8_t msgType;

  if (quadType == KQUAD_TYPE_STANDARD)
  {
    MAX_THRUST_GRAMS = 32760;
    MAX_ROLL         = DegToRad(360.0);
    MAX_PITCH        = DegToRad(360.0);
    MAX_RATE_RAD     = 35.0;   //2000 deg/sec
    MAX_GAIN_P       = 5000;
    MAX_GAIN_D       = 250;

    SCALE_THRUST     = 1.0;
    SCALE_RPY        = 1000.0;
    SCALE_RATE       = 500.0;
    SCALE_GAIN_P     = 0.05;
    SCALE_GAIN_D     = 1.0;

    msgType          = KQUAD_LL_STD_CMD3;
  }
  else if (quadType == KQUAD_TYPE_NANO)
  {
    MAX_THRUST_GRAMS = 3276;
    MAX_ROLL         = DegToRad(360.0);
    MAX_PITCH        = DegToRad(360.0);
    MAX_RATE_RAD     = 35.0;  //2000 deg/sec
    MAX_GAIN_P       = 1000;
    MAX_GAIN_D       = 100;

    SCALE_THRUST     = 10.0;
    SCALE_RPY        = 1000.0;
    SCALE_RATE       = 500.0;
    SCALE_GAIN_P     = 0.25;
    SCALE_GAIN_D     = 2.5;

    msgType          = KQUAD_LL_NANO_CMD3;
  }
  else
  {
    PRINT_ERROR("bad quad type:" << quadType<<"\n");
    return -1;
  }

  //sanity checking
  if ( (roll > MAX_ROLL) || (roll < -MAX_ROLL) || (pitch > MAX_PITCH) || (pitch < -MAX_PITCH) )
  {
    PRINT_ERROR("roll or pitch values are beyond min or max limits\n");
    return -1;
  }

  if ((kp_roll <0) || (kp_pitch <0) || (kp_yaw <0) || (kd_roll <0) || (kd_pitch <0) || (kd_yaw <0))
  {
    PRINT_ERROR("one of the gains is negative\n");
    return -1;
  }

  if (thrust > MAX_THRUST_GRAMS)
  {
    thrust = MAX_THRUST_GRAMS;
    PRINT_ERROR("thrust is above 3276, so it's being capped\n");
  }


  if (droll  >  MAX_RATE_RAD)  { droll    =  MAX_RATE_RAD; PRINT_ERROR("droll being capped!!\n");  }
  if (droll  < -MAX_RATE_RAD)  { droll    = -MAX_RATE_RAD; PRINT_ERROR("droll being capped!!\n");  }
  if (dpitch >  MAX_RATE_RAD)  { dpitch   =  MAX_RATE_RAD; PRINT_ERROR("dpitch being capped!!\n"); }
  if (dpitch < -MAX_RATE_RAD)  { dpitch   = -MAX_RATE_RAD; PRINT_ERROR("dpitch being capped!!\n"); }
  if (dyaw   >  MAX_RATE_RAD)  { dyaw     =  MAX_RATE_RAD; PRINT_ERROR("dyaw being capped!!\n");   }
  if (dyaw   < -MAX_RATE_RAD)  { dyaw     = -MAX_RATE_RAD; PRINT_ERROR("dyaw being capped!!\n");   }

  if (kp_roll  >  MAX_GAIN_P)  { kp_roll  =  MAX_GAIN_P; PRINT_ERROR("kp_roll being capped!!\n");  }
  if (kp_roll  < -MAX_GAIN_P)  { kp_roll  = -MAX_GAIN_P; PRINT_ERROR("kp_roll being capped!!\n");  }
  if (kp_pitch >  MAX_GAIN_P)  { kp_pitch =  MAX_GAIN_P; PRINT_ERROR("kp_pitch being capped!!\n"); }
  if (kp_pitch < -MAX_GAIN_P)  { kp_pitch = -MAX_GAIN_P; PRINT_ERROR("kp_pitch being capped!!\n"); }
  if (kp_yaw   >  MAX_GAIN_P)  { kp_yaw   =  MAX_GAIN_P; PRINT_ERROR("kp_yaw being capped!!\n");   }
  if (kp_yaw   < -MAX_GAIN_P)  { kp_yaw   = -MAX_GAIN_P; PRINT_ERROR("kp_yaw being capped!!\n");   }

  if (kd_roll  >  MAX_GAIN_D)  { kd_roll  =  MAX_GAIN_D; PRINT_ERROR("kd_roll being capped!!\n");  }
  if (kd_roll  < -MAX_GAIN_D)  { kd_roll  = -MAX_GAIN_D; PRINT_ERROR("kd_roll being capped!!\n");  }
  if (kd_pitch >  MAX_GAIN_D)  { kd_pitch =  MAX_GAIN_D; PRINT_ERROR("kd_pitch being capped!!\n"); }
  if (kd_pitch < -MAX_GAIN_D)  { kd_pitch = -MAX_GAIN_D; PRINT_ERROR("kd_pitch being capped!!\n"); }
  if (kd_yaw   >  MAX_GAIN_D)  { kd_yaw   =  MAX_GAIN_D; PRINT_ERROR("kd_yaw being capped!!\n");   }
  if (kd_yaw   < -MAX_GAIN_D)  { kd_yaw   = -MAX_GAIN_D; PRINT_ERROR("kd_yaw being capped!!\n");   }

  //wrap the yaw to -pi .. pi
  yaw = Mod2PiF(yaw);

  //get a pointer to a temporary buffer and fill it in
  QuadCmd3 cmd;

  cmd.id       = id;
  cmd.chan     = channel;
  cmd.cntr     = 0; // TODO

  cmd.thrust   = thrust   * SCALE_THRUST;
  cmd.roll     = roll     * SCALE_RPY;
  cmd.pitch    = pitch    * SCALE_RPY;
  cmd.yaw      = yaw      * SCALE_RPY;

  cmd.droll    = droll    * SCALE_RATE;
  cmd.dpitch   = dpitch   * SCALE_RATE;
  cmd.dyaw     = dyaw     * SCALE_RATE;

  cmd.kp_roll  = kp_roll  * SCALE_GAIN_P;
  cmd.kp_pitch = kp_pitch * SCALE_GAIN_P;
  cmd.kp_yaw   = kp_yaw   * SCALE_GAIN_P;

  cmd.kd_roll  = kd_roll  * SCALE_GAIN_D;
  cmd.kd_pitch = kd_pitch * SCALE_GAIN_D;
  cmd.kd_yaw   = kd_yaw   * SCALE_GAIN_D;

  //wrap the data into the packet
  int len = kBotPacket2WrapData(KQUAD_LL_DEVICE_ID, msgType,
                                (uint8_t*)&cmd, sizeof(cmd),
                                dst, dstLen);

  return len;
}

int   kQuadInterface::PackQuadCmd4(uint8_t id, uint8_t quadType, uint8_t channel, float thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, float mx, float my, float mz,
                             uint8_t * dst, int dstLen)
{
    //check for the buffer lenght
  int bufLen = sizeof(QuadCmd4) + KBOT_PACKET2_OVERHEAD;
  if (dstLen < bufLen)
  {
    PRINT_ERROR("not enough space in the provided buffer for the packet\n");
    return -1;
  }


  float MAX_THRUST_GRAMS, MAX_ROLL, MAX_PITCH, MAX_MOMENT_GRAMS, MAX_RATE_RAD, MAX_GAIN_P, MAX_GAIN_D;
  float SCALE_THRUST, SCALE_MOMENT, SCALE_RATE, SCALE_GAIN_P, SCALE_GAIN_D, SCALE_RPY;
  uint8_t msgType;

  if (quadType == KQUAD_TYPE_STANDARD)
  {
    MAX_THRUST_GRAMS = 10000;
    MAX_ROLL         = DegToRad(360.0);
    MAX_PITCH        = DegToRad(360.0);
    MAX_MOMENT_GRAMS = 2500;
    MAX_RATE_RAD     = 35.0;  //2000 deg/sec
    MAX_GAIN_P       = 5000;
    MAX_GAIN_D       = 250;

    SCALE_THRUST     = 1.0;
    SCALE_MOMENT     = 1.0;
    SCALE_RPY        = 1000.0;
    SCALE_RATE       = 500.0;
    SCALE_GAIN_P     = 0.05;
    SCALE_GAIN_D     = 1.0;

    msgType          = KQUAD_LL_STD_CMD4;
  }
  else if (quadType == KQUAD_TYPE_NANO)
  {
    MAX_THRUST_GRAMS = 1000;
    MAX_ROLL         = DegToRad(360.0);
    MAX_PITCH        = DegToRad(360.0);
    MAX_MOMENT_GRAMS = 250;  //2000 deg/sec
    MAX_RATE_RAD     = 35.0;
    MAX_GAIN_P       = 1000;
    MAX_GAIN_D       = 100;

    SCALE_THRUST     = 10.0;
    SCALE_MOMENT     = 10.0;
    SCALE_RPY        = 1000.0;
    SCALE_RATE       = 500.0;
    SCALE_GAIN_P     = 0.25;
    SCALE_GAIN_D     = 2.5;

    msgType          = KQUAD_LL_NANO_CMD4;
  }
  else
  {
    PRINT_ERROR("bad quad type:" << quadType<<"\n");
    return -1;
  }

  //sanity checking
  if ( (roll > MAX_ROLL) || (roll < -MAX_ROLL) || (pitch > MAX_PITCH) || (pitch < -MAX_PITCH) )
  {
    PRINT_ERROR("roll or pitch values are beyond min or max limits\n");
    return -1;
  }

  if ((kp_roll <0) || (kp_pitch <0) || (kp_yaw <0) || (kd_roll <0) || (kd_pitch <0) || (kd_yaw <0))
  {
    PRINT_ERROR("one of the gains is negative\n");
    return -1;
  }

  if (thrust > MAX_THRUST_GRAMS)
  {
    thrust = MAX_THRUST_GRAMS;
    PRINT_ERROR("thrust is above 3276, so it's being capped\n");
  }


  if (droll  >  MAX_RATE_RAD)  { droll    =  MAX_RATE_RAD; PRINT_ERROR("droll being capped!!\n");  }
  if (droll  < -MAX_RATE_RAD)  { droll    = -MAX_RATE_RAD; PRINT_ERROR("droll being capped!!\n");  }
  if (dpitch >  MAX_RATE_RAD)  { dpitch   =  MAX_RATE_RAD; PRINT_ERROR("dpitch being capped!!\n"); }
  if (dpitch < -MAX_RATE_RAD)  { dpitch   = -MAX_RATE_RAD; PRINT_ERROR("dpitch being capped!!\n"); }
  if (dyaw   >  MAX_RATE_RAD)  { dyaw     =  MAX_RATE_RAD; PRINT_ERROR("dyaw being capped!!\n");   }
  if (dyaw   < -MAX_RATE_RAD)  { dyaw     = -MAX_RATE_RAD; PRINT_ERROR("dyaw being capped!!\n");   }

  if (kp_roll  >  MAX_GAIN_P)  { kp_roll  =  MAX_GAIN_P; PRINT_ERROR("kp_roll being capped!!\n");  }
  if (kp_roll  < -MAX_GAIN_P)  { kp_roll  = -MAX_GAIN_P; PRINT_ERROR("kp_roll being capped!!\n");  }
  if (kp_pitch >  MAX_GAIN_P)  { kp_pitch =  MAX_GAIN_P; PRINT_ERROR("kp_pitch being capped!!\n"); }
  if (kp_pitch < -MAX_GAIN_P)  { kp_pitch = -MAX_GAIN_P; PRINT_ERROR("kp_pitch being capped!!\n"); }
  if (kp_yaw   >  MAX_GAIN_P)  { kp_yaw   =  MAX_GAIN_P; PRINT_ERROR("kp_yaw being capped!!\n");   }
  if (kp_yaw   < -MAX_GAIN_P)  { kp_yaw   = -MAX_GAIN_P; PRINT_ERROR("kp_yaw being capped!!\n");   }

  if (kd_roll  >  MAX_GAIN_D)  { kd_roll  =  MAX_GAIN_D; PRINT_ERROR("kd_roll being capped!!\n");  }
  if (kd_roll  < -MAX_GAIN_D)  { kd_roll  = -MAX_GAIN_D; PRINT_ERROR("kd_roll being capped!!\n");  }
  if (kd_pitch >  MAX_GAIN_D)  { kd_pitch =  MAX_GAIN_D; PRINT_ERROR("kd_pitch being capped!!\n"); }
  if (kd_pitch < -MAX_GAIN_D)  { kd_pitch = -MAX_GAIN_D; PRINT_ERROR("kd_pitch being capped!!\n"); }
  if (kd_yaw   >  MAX_GAIN_D)  { kd_yaw   =  MAX_GAIN_D; PRINT_ERROR("kd_yaw being capped!!\n");   }
  if (kd_yaw   < -MAX_GAIN_D)  { kd_yaw   = -MAX_GAIN_D; PRINT_ERROR("kd_yaw being capped!!\n");   }

  if (mx >  MAX_MOMENT_GRAMS)   { mx       =  MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}
  if (mx < -MAX_MOMENT_GRAMS)   { mx       = -MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}
  if (my >  MAX_MOMENT_GRAMS)   { my       =  MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}
  if (my < -MAX_MOMENT_GRAMS)   { my       = -MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}
  if (mz >  MAX_MOMENT_GRAMS)   { mz       =  MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}
  if (mz < -MAX_MOMENT_GRAMS)   { mz       = -MAX_MOMENT_GRAMS; PRINT_ERROR("mx being capped!!\n");}

  //wrap the yaw to -pi .. pi
  yaw = Mod2PiF(yaw);

  //get a pointer to a temporary buffer and fill it in
  QuadCmd4 cmd;

  cmd.id       = id;
  cmd.chan     = channel;
  cmd.cntr     = 0; // TODO

  cmd.thrust   = thrust   * SCALE_THRUST;
  cmd.roll     = roll     * SCALE_RPY;
  cmd.pitch    = pitch    * SCALE_RPY;
  cmd.yaw      = yaw      * SCALE_RPY;

  cmd.droll    = droll    * SCALE_RATE;
  cmd.dpitch   = dpitch   * SCALE_RATE;
  cmd.dyaw     = dyaw     * SCALE_RATE;

  cmd.kp_roll  = kp_roll  * SCALE_GAIN_P;
  cmd.kp_pitch = kp_pitch * SCALE_GAIN_P;
  cmd.kp_yaw   = kp_yaw   * SCALE_GAIN_P;

  cmd.kd_roll  = kd_roll  * SCALE_GAIN_D;
  cmd.kd_pitch = kd_pitch * SCALE_GAIN_D;
  cmd.kd_yaw   = kd_yaw   * SCALE_GAIN_D;

  cmd.mx       = mx       * SCALE_MOMENT;
  cmd.my       = my       * SCALE_MOMENT;
  cmd.mz       = mz       * SCALE_MOMENT;

  //wrap the data into the packet
  int len = kBotPacket2WrapData(KQUAD_LL_DEVICE_ID, msgType,
                                (uint8_t*)&cmd, sizeof(cmd),
                                dst, dstLen);

  return len;
}


int kQuadInterface::PackKbeeChannelCmd(int id, int channel, uint8_t * dst, int dstLen)
{
  int len;

  if (id == 1)
  {
    uint8_t data[2];
    data[0] = id;
    data[1] = channel;

    //wrap the data into the packet
    len = kBotPacket2WrapData(KQUAD_ZIGBEE_DEVICE_ID, KQUAD_ZIGBEE_SET_CHANNEL,
                                 data, 2,dst, dstLen);
  }
  else if (id == 2)
  {
    uint8_t data[2];
    data[0] = id;
    data[1] = channel;

    //wrap the data into the packet
    len = kBotPacket2WrapData(KQUAD_ZIGBEE_DEVICE_ID, KQUAD_ZIGBEE_SET_CHANNEL,
                                 data, 2, dst, dstLen);
  }
  else
  {
    PRINT_ERROR("bad kbee id :"<<id<<"\n");
    return -1;
  }

  return len;
}

int kQuadInterface::PackKbeeModeCmd(int id, int mode, uint8_t * dst, int dstLen)
{
  if ( (id != 1) && (id != 2) )
  {
     PRINT_ERROR("bad kbee id :"<<id<<"\n");
     return -1;
  }

  if ( (mode != 0) && (mode != 1) && (mode != 2))
  {
     PRINT_ERROR("bad kbee mode :"<<mode<<"\n");
     return -1;
  }

  uint8_t data[2];
  data[0] = id;
  data[1] = mode;

  //wrap the data into the packet
  int len = kBotPacket2WrapData(KQUAD_ZIGBEE_DEVICE_ID, KQUAD_ZIGBEE_SET_MODE,
                            data, 2, dst, dstLen);

  return len;
}

int kQuadInterface::SendKbeeModeCmd(int id, int mode)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackKbeeModeCmd(id, mode,data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->LockDataMutex();
  this->kbeeData.push_back(data);
  this->UnlockDataMutex();

  return 0;
}

int kQuadInterface::SendKbeeChannelCmd(int id, int channel)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackKbeeChannelCmd(id, channel, data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->LockDataMutex();
  this->kbeeData.push_back(data);
  this->UnlockDataMutex();

  return 0;
}


int kQuadInterface::SendQuadCmd1(uint8_t id, uint8_t quadType, uint8_t channel, float thrust,
                             float roll, float pitch, float yaw)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackQuadCmd1(id, quadType, channel, thrust, roll, pitch, yaw, data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->AddDataToQueue(&data);

  return 0;
}

int kQuadInterface::SendQuadCmd2(uint8_t id, uint8_t quadType, uint8_t channel, int16_t rpm0,
                             int16_t rpm1, int16_t rpm2, int16_t rpm3)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackQuadCmd2(id, quadType, channel, rpm0, rpm1, rpm2, rpm3, data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->AddDataToQueue(&data);

  return 0;
}

int kQuadInterface::SendQuadCmd3(uint8_t id, uint8_t quadType, uint8_t channel, float thrust,
                             float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackQuadCmd3(id, quadType, channel, thrust, roll, pitch, yaw, droll, dpitch, dyaw,
                         kp_roll, kp_pitch, kp_yaw, kd_roll, kd_pitch, kd_yaw, data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->AddDataToQueue(&data);

  return 0;
}

int kQuadInterface::SendQuadCmd4(uint8_t id, uint8_t quadType, uint8_t channel, float
                             thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, float mx, float my, float mz)
{
  CHECK_CONNECTION;

  kQuadControlData data;
  data.id = id;
  data.t  = Timer::GetUnixTime();

  int len = PackQuadCmd4(id, quadType, channel, thrust, roll, pitch, yaw, droll, dpitch, dyaw,
                         kp_roll, kp_pitch, kp_yaw, kd_roll, kd_pitch, kd_yaw, mx, my, mz, data.data, 256);

  if (len < 0)
    return -1;

  data.size = len;

  this->AddDataToQueue(&data);

  return 0;
}



int kQuadInterface::AddDataToQueue(kQuadControlData * data)
{
  this->LockDataMutex();

  int nlist = this->controlData.size();
  int foundDup = 0;

  if (nlist > 0)
  {
    //make sure that there is no command already pending for the same id
    list<kQuadControlData>::iterator it = this->controlData.begin();

    while( it != this->controlData.end() )
    {
      if ((*it).id == data->id)   //id already exists in the queue
      {
        if (foundDup == 0)
        {
          *it++ = *data;
          foundDup = 1;  //mark it
          //printf("replaced data with %d\n",data.id);
        }
        else  //remove any additional packets with the same id (too old). This should never happen
        {
          it = this->controlData.erase(it);  //erase the current one. returns next item: dont increment!
          //printf("erased data with %d\n",data.id);
        }
      }
      else //id does not match
      {
        it++;
      }
    }
  }

  if ((nlist == 0) || (foundDup == 0)) //did not find a duplicate, so need to insert the data
  {
    this->controlData.push_back(*data);
    //printf("pushed data with %d\n",data.id);
  }

  this->UnlockDataMutex();

  return 0;
}

int kQuadInterface::ProcessIncomingPacket(kBotPacket2 * packet)
{
  uint8_t id     = kBotPacket2GetId(packet);
  uint8_t type   = kBotPacket2GetType(packet);
  uint8_t len    = kBotPacket2GetPayloadSize(packet);
  uint8_t * data = kBotPacket2GetData(packet);

  //printf("got packet with id(%d), type(%d), length(%d)\n",id,type,len);

  if (id == KQUAD_LL_DEVICE_ID)
  {
    switch (type)
    {
      case 0  : imuRawDataC.PushData(data,len);         break;
      //case 1  : imuFiltDataC.PushData(data,len,1);     break;
      case 5  : rcDataC.PushData(data,len);             break;
      case 27 : quadStatusDataC.PushData(data,len);     break;
      case 31 : gpsNmeaDataC.PushData(data,len);        break;
      case 32 : gpsUbloxDataC.PushData(data,len);       break;
      case 33 : pressMagDataC.PushData(data,len,33);    break;
      case 34 : imuFiltDataC.PushData(data,len,34);     break;
      case 35 : pressMagDataC.PushData(data,len,35);    break;
      case 37 : motorStatusDataC.PushData(data,len,37); break;
    }
  }
  else if (id == 5)
  {
    if (type == 3)
      this->zrxStatusDataC.PushData(data,len,3);
  }
  return 0;
}

int kQuadInterface::SetMaxQueueLength(uint32_t maxLen)
{
  imuRawDataC.SetQueueLength(maxLen);
  imuFiltDataC.SetQueueLength(maxLen);
  battDataC.SetQueueLength(maxLen);
  rcDataC.SetQueueLength(maxLen);
  gpsNmeaDataC.SetQueueLength(maxLen);
  gpsUbloxDataC.SetQueueLength(maxLen);
  quadStatusDataC.SetQueueLength(maxLen);
  servoDataC.SetQueueLength(maxLen);
  zrxStatusDataC.SetQueueLength(maxLen);
  motorStatusDataC.SetQueueLength(maxLen);

  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Get the name of the state in the state machine
//_______________________________________________________
int kQuadInterface::GetStateName(int state, string & name)
{
  switch (state)
  {
    case STATE_OFF:
      name = string("Off");
      break;

    case STATE_INITIALIZED:
      name = string("Initialized");
      break;

    case STATE_WAITING_FOR_RC:
      name = string("Waiting for RC");
      break;

    case STATE_READY_TO_START:
      name = string("Ready to start");
      break;

    case STATE_MOTORS_STARTING:
      name = string("Motors starting");
      break;

    case STATE_MOTORS_STARTED:
      name = string("Motors spinning");
      break;

    case STATE_MANUAL_FLIGHT:
      name = string("Manual flight");
      break;

    case STATE_AUTO_MOTORS_STARTED:
      name = string("Auto motors spinning");
      break;

    case STATE_AUTO_FLIGHT:
      name = string("Auto flight");
      break;

    case STATE_EMERGENCY_LANDING:
      name = string("Emergency landing");
      break;

    case STATE_EMERGENCY_KILLED:
      name = string("Emergency stopped");
      break;

    case STATE_UNKNOWN:
    default:
      name = string("Unknown");
      break;
  }

  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Accessor functions for the feedback data
//_______________________________________________________
int kQuadInterface::GetImuFiltData    ( list<ImuFiltData>     & dataList, uint32_t sizer)
{
  return imuFiltDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetImuRawData     ( list<ImuRawData>      & dataList, uint32_t sizer)
{
  return imuRawDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetBatteryData    ( list<BatteryData>     & dataList, uint32_t sizer)
{
  return battDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetRcData         ( list<RcData>          & dataList, uint32_t sizer)
{
  return rcDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetGpsNmeaData    ( list<GpsNmeaData>     & dataList, uint32_t sizer)
{
  return gpsNmeaDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetGpsUbloxData  ( list<GpsUbloxData>   & dataList, uint32_t sizer)
{
  return gpsUbloxDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetQuadStatusData ( list<QuadStatusData>  & dataList, uint32_t sizer)
{
  return quadStatusDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetServoData      ( list<ServoData>       & dataList, uint32_t sizer)
{
  return servoDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetPressMagData   ( list<PressureMagData> & dataList, uint32_t sizer)
{
  return pressMagDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetZigRxStatData  ( list<ZigBeeRxStatus>  & dataList, uint32_t sizer)
{
  return zrxStatusDataC.GetData(dataList,sizer);
}

int kQuadInterface::GetMotorStatusData  ( list<MotorStatusData>  & dataList, uint32_t sizer)
{
  return motorStatusDataC.GetData(dataList,sizer);
}

void kQuadInterface::PrintImuFiltData ( list<ImuFiltData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<ImuFiltData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}


void kQuadInterface::PrintImuRawData ( list<ImuRawData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<ImuRawData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}

void kQuadInterface::PrintQuadStatusData( list<QuadStatusData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<QuadStatusData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}

void kQuadInterface::PrintRcData ( list<RcData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<RcData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}

void kQuadInterface::PrintPressMagData    ( list<PressureMagData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<PressureMagData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}

void kQuadInterface::PrintZigRxStatData   ( list<ZigBeeRxStatus>  & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<ZigBeeRxStatus>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}

void kQuadInterface::PrintMotorStatusData ( list<MotorStatusData> & dataList)
{
  int size = dataList.size();
  if (size < 1) return;
  list<MotorStatusData>::iterator it = dataList.begin();
  while (size--) (*(it++)).Print();
}
