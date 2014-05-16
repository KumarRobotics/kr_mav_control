/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#ifndef KQUAD_INTERFACE_HH
#define KQUAD_INTERFACE_HH

#include <string>
#include <stdint.h>
#include <pthread.h>
#include <list>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>


#include "SerialDevice.hh"
#include "kQuadDataTypes2.h"
#include "kQuadCom.h"
#include "kBotPacket2.h"
#include "kBotPacketParser.hh"
#include "kQuadInterfaceDataTypes.hh"
#include "mathMacros.h"

/*
#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__PRETTY_FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
*/

#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }


using namespace std;

struct kQuadControlData
{
  double   t;
  uint32_t id;
  int32_t size;
  uint8_t  data[256];
};

enum
{
  KQUAD_TYPE_STANDARD,
  KQUAD_TYPE_NANO
};

enum
{
  STATE_UNKNOWN,
  STATE_OFF,
  STATE_INITIALIZED,
  STATE_WAITING_FOR_RC,
  STATE_READY_TO_START,
  STATE_MOTORS_STARTING,
  STATE_MOTORS_STARTED,
  STATE_MANUAL_FLIGHT,
  STATE_AUTO_MOTORS_STARTED,
  STATE_AUTO_FLIGHT,
  STATE_EMERGENCY_LANDING,
  STATE_EMERGENCY_KILLED
};

class kQuadInterface
{
  public:
    kQuadInterface();
   ~kQuadInterface();

    int   Connect(string dev,int baud);
    int   Disconnect();
    int   StartSendThread();
    int   StopSendThread();
    int   StartRecvThread();
    int   StopRecvThread();

    inline bool IsConnected() { return this->connected; }


    int   PackQuadCmd1(uint8_t id, uint8_t quadType, uint8_t channel, float thrust,
                             float roll, float pitch, float yaw, uint8_t * dst, int len);

    int   PackQuadCmd2(uint8_t id, uint8_t quadType, uint8_t channel, int16_t rpm0,
                             int16_t rpm1, int16_t rpm2, int16_t rpm3, uint8_t * dst, int len);

    int   PackQuadCmd3(uint8_t id, uint8_t quadType, uint8_t channel, float thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, uint8_t * dst, int len);

    int   PackQuadCmd4(uint8_t id, uint8_t quadType, uint8_t channel, float thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, float mx, float my, float mz,
                             uint8_t * dst, int len);

    int   SendQuadCmd1(uint8_t id, uint8_t quadType, uint8_t channel, float thrust,
                             float roll, float pitch, float yaw);

    int   SendQuadCmd2(uint8_t id, uint8_t quadType, uint8_t channel, int16_t rpm0,
                             int16_t rpm1, int16_t rpm2, int16_t rpm3);

    int   SendQuadCmd3(uint8_t id, uint8_t quadType, uint8_t channel, float thrust,
                             float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw);

    int   SendQuadCmd4(uint8_t id, uint8_t quadType, uint8_t channel, float
                             thrust, float roll, float pitch, float yaw,
                             float droll, float dpitch, float dyaw, float kp_roll, float kp_pitch, float kp_yaw,
                             float kd_roll, float kd_pitch, float kd_yaw, float mx, float my, float mz);


    int   PackKbeeModeCmd(int id, int mode, uint8_t * dst, int dstLen);
    int   PackKbeeChannelCmd(int id, int channel, uint8_t * dst, int dstLen);
    int   SendKbeeModeCmd(int id, int mode);
    int   SendKbeeChannelCmd(int id, int channel);
    int   AddDataToQueue(kQuadControlData * data);
    int   ProcessIncomingPacket(kBotPacket2 * packet);
    int   GetStateName(int state, string & name);


    int   SetMaxQueueLength(uint32_t maxLen);

    //data accessors
    int  GetImuFiltData       ( list<ImuFiltData>     & dataList, uint32_t sizer = 0 );
    int  GetImuRawData        ( list<ImuRawData>      & dataList, uint32_t sizer = 0 );
    int  GetBatteryData       ( list<BatteryData>     & dataList, uint32_t sizer = 0 );
    int  GetRcData            ( list<RcData>          & dataList, uint32_t sizer = 0 );
    int  GetGpsNmeaData       ( list<GpsNmeaData>     & dataList, uint32_t sizer = 0 );
    int  GetGpsUbloxData      ( list<GpsUbloxData>    & dataList, uint32_t sizer = 0 );
    int  GetQuadStatusData    ( list<QuadStatusData>  & dataList, uint32_t sizer = 0 );
    int  GetServoData         ( list<ServoData>       & dataList, uint32_t sizer = 0 );
    int  GetPressMagData      ( list<PressureMagData> & dataList, uint32_t sizer = 0 );
    int  GetZigRxStatData     ( list<ZigBeeRxStatus>  & dataList, uint32_t sizer = 0 );
    int  GetMotorStatusData   ( list<MotorStatusData> & dataList, uint32_t sizer = 0 );

    void PrintImuFiltData     ( list<ImuFiltData>     & dataList);
    void PrintImuRawData      ( list<ImuRawData>      & dataList);
    void PrintBatteryData     ( list<BatteryData>     & dataList);
    void PrintRcData          ( list<RcData>          & dataList);
    void PrintGpsNmeaData     ( list<GpsNmeaData>     & dataList);
    void PrintGpsUbloxData    ( list<GpsUbloxData>    & dataList);
    void PrintQuadStatusData  ( list<QuadStatusData>  & dataList);
    void PrintServoData       ( list<ServoData>       & dataList);
    void PrintPressMagData    ( list<PressureMagData> & dataList);
    void PrintZigRxStatData   ( list<ZigBeeRxStatus>  & dataList);
    void PrintMotorStatusData ( list<MotorStatusData> & dataList);

    int  SetEmergencyThrust(uint16_t id, float thrust);

  protected:
    inline int LockDataMutex()   { return pthread_mutex_lock( &this->dataMutex );  }
    inline int UnlockDataMutex() { return pthread_mutex_unlock( &this->dataMutex );}

    inline float DegToRad(float deg) { return deg*M_PI/180.0; }
    inline float RadToDeg(float rad) { return rad*180.0/M_PI; }

    SerialDevice sd;
    pthread_t sendThread;
    pthread_t recvThread;

    pthread_mutex_t dataMutex;
    static void *SendThreadFunc(void * input);
    static void *RecvThreadFunc(void * input);

    int connected;
    int sendThreadRunning;
    int recvThreadRunning;

    kBotPacket2 packet;
    Timer cmdOutputTimer;

    list<kQuadControlData> controlData;
    list<kQuadControlData> kbeeData;

    kBotPacketParser kparser;

    kDataContainer<ImuRawData>      imuRawDataC;
    kDataContainer<ImuFiltData>     imuFiltDataC;
    kDataContainer<BatteryData>     battDataC;
    kDataContainer<RcData>          rcDataC;
    kDataContainer<GpsNmeaData>     gpsNmeaDataC;
    kDataContainer<GpsUbloxData>    gpsUbloxDataC;
    kDataContainer<QuadStatusData>  quadStatusDataC;
    kDataContainer<ServoData>       servoDataC;
    kDataContainer<PressureMagData> pressMagDataC;
    kDataContainer<ZigBeeRxStatus>  zrxStatusDataC;
    kDataContainer<MotorStatusData> motorStatusDataC;
};
#endif //KQUAD_INTERFACE_HH

