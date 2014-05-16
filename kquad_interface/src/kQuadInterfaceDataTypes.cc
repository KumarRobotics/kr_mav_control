/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "kQuadInterfaceDataTypes.hh"
#include "Timer.hh"
#include <list>
#include <vector>
#include <stdint.h>
#include <pthread.h>
#include <string.h>

using namespace std;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Raw IMU
//_______________________________________________________
int ImuRawData::Parse(uint8_t * raw, int len, uint8_t type)
{
  int32_t * idata = (int32_t*)raw;

  tpc   = Timer::GetUnixTime();
  wxr1  = *idata++; wyr1  = *idata++; wzr1  = *idata++;
  wxr2  = *idata++; wyr2  = *idata++; wzr2  = *idata++;
  axr   = *idata++; ayr   = *idata++; azr   = *idata++;
  wxtr1 = *idata++; wytr1 = *idata++; wztr1 = *idata++;
  atr   = *idata++; idata++; //skip counter
  tuc   = *((uint32_t*)idata++);

  //Print();
  return 0;
}

int ImuRawData::Print()
{
  PRINT_INFO("IMU RAW: ("<<tpc<<" "<<tuc<<") wxyz=("<<wxr1<<" "<<wyr1<<" "<<wzr1<<") "
                    <<"wxyz2=("<<wxr2<<" "<<wyr2<<" "<<wzr2<<") "
                    <<"axyz=("<<axr<<" "<<ayr<<" "<<azr<<") "
                    <<"temps=("<<wxtr1<<" "<<wytr1<<" "<<wztr1<<" "<<atr<<")\n");

  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Filtered IMU
//_______________________________________________________
int ImuFiltData::Parse(uint8_t * raw, int len, uint8_t type)
{
  if (type == 1)
  {
    float * fdata = (float *)raw;

    tpc    = Timer::GetUnixTime();        tuc    = *((uint32_t*)fdata++);
    roll   = *fdata++; pitch  = *fdata++; yaw    = *fdata++;
    wroll  = *fdata++; wpitch = *fdata++; wyaw   = *fdata++;
    ax     = *fdata++; ay     = *fdata++; az     = *fdata++;
  }
  else if (type == 34)
  {
    int16_t * idata = (int16_t*)raw;
    tpc    = Timer::GetUnixTime();        tuc    = *((uint32_t*)idata++); idata++;
    id = *reinterpret_cast<uint8_t*>(idata);
    idata++; //id, cntr

    roll  = ((float)(*idata++) / 5000.0); pitch  = ((float)(*idata++) / 5000.0);  yaw = ((float)(*idata++)  / 5000.0);
    wroll = ((float)(*idata++) / 500.0);  wpitch = ((float)(*idata++) / 500.0);  wyaw = ((float)(*idata++)  / 500.0);
    ax    = ((float)(*idata++) / 5000.0); ay     = ((float)(*idata++) / 5000.0);  az  = ((float)(*idata++)  / 5000.0);
  }

  //Print();
  return 0;
}


int ImuFiltData::Print()
{
  PRINT_INFO("IMU: ("<<tpc<<" "<<tuc<<") rpy=("<<roll<<" "<<pitch<<" "<<yaw<<"), wrpy("
                     <<wroll<<" "<<wpitch<<" "<<wyaw<<"), acc("
                     <<ax<<" "<<ay<<" "<<az<<")\n");
  return 0;
}



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Battery
//_______________________________________________________
int BatteryData::Parse(uint8_t * raw, int len, uint8_t type)
{
  return 0;
}
int BatteryData::Print()
{
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// RC data
//_______________________________________________________
int RcData::Parse(uint8_t * raw, int len, uint8_t type)
{
  uint16_t * data2 = (uint16_t *)raw;
  uint32_t * t    = (uint32_t*)(&data2[8]);
  uint8_t  * id2  = (uint8_t*)(&data2[10]);

  for (int ii=0; ii<8; ii++)
    data[ii] = data2[ii];

  tpc     = Timer::GetUnixTime();
  tuc     = *t;
  id      = *id2;

  //Print();
  return 0;
}
int RcData::Print()
{
  PRINT_INFO("RC ("<<id<<", "<<tuc<<"): " << data[0] <<" "<< data[1] <<" "<< data[2] <<
                 " "<< data[3] <<" "<< data[4] <<" "<< data[5]<<"\n");
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// GPS NMEA
//_______________________________________________________
int GpsNmeaData::Parse(uint8_t * raw, int len, uint8_t type)
{
  return 0;
}
int GpsNmeaData::Print()
{
  PRINT_INFO("GPS: ("<<(int)id<<","<<tuc<<","<<tpc<<"): "<<((char*)data));
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// GPS Ublox
//_______________________________________________________
int GpsUbloxData::Parse(uint8_t * raw, int len, uint8_t type)
{
  tpc     = Timer::GetUnixTime();
  tuc     = *((uint32_t*)(raw+len-4));
  ubclass = raw[2];
  ubid    = raw[3];
  ublen   = raw[4]; ublen |= ((uint16_t)raw[5]) << 8;

  if (ublen > 512)
    return -1;

  memcpy(ubdata,raw+6,ublen);

  printf("got ublox packet class %d, id %d, length %d, time %d\n",ubclass,ubid,ublen,tuc);

  return 0;
}
int GpsUbloxData::Print()
{
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Quad Status
//_______________________________________________________
int QuadStatusData::Parse(uint8_t * raw, int len, uint8_t type)
{


  tpc = Timer::GetUnixTime();
  tuc = *((uint32_t*)raw); raw+=4;

  uint16_t * data2 = (uint16_t *)raw;
  voltage = *data2++ / 1000.0f;
  current = *data2++ / 1000.0f;

  raw+=4;
  id        = *raw++;
  state     = *raw++;
  autoCntr  = *raw++;
  rcCntr    = *raw++;
  nerr      = *raw++;
  lastError = *raw++;
  sigstren  = *raw++;

  //Print();

  return 0;
}
int QuadStatusData::Print()
{
  PRINT_INFO("Quad Status: t:" <<tuc<<", id: "<<id<<", state: "<<state<<", voltage: "<<voltage<<
           ", current: "<<current<<", autoCntr: "<<autoCntr<<", rcCntr "<<rcCntr<<", nerr: "<<nerr<<", last error: "<<lastError<<", signal str "<<sigstren<<"\n");
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Servo Data
//_______________________________________________________
int ServoData::Parse(uint8_t * raw, int len, uint8_t type)
{
  return 0;
}

int ServoData::Print()
{
  PRINT_INFO("Servo Data: " << cntr << " "<<tuc<<" "<<angle<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Pressure Mag Data
//_______________________________________________________
int PressureMagData::Parse(uint8_t * raw, int len, uint8_t type)
{
  //printf("got pressure mag data!!!\n");

  if (type == 33)
  {
    tpc = Timer::GetUnixTime();
    id = *raw++;

    uint32_t * pdata = (uint32_t*)raw;

    tucp0 = *pdata++;
    tucp1 = *pdata++;
    tucm  = *pdata++;

    pressure[0] = *pdata++;
    pressure[1] = *pdata++;

    int16_t * pdata2 = (int16_t*)pdata;

    temperature[0] = *pdata2++;
    temperature[1] = *pdata2++;

    mx = *pdata2++;
    my = *pdata2++;
    mz = *pdata2++;
  }
  else if (type == 35)
  {
    tpc = Timer::GetUnixTime();
    id = *raw++;

    uint32_t * pdata = (uint32_t*)raw;

    tucp0 = *pdata++;
    tucp1 = tucp0;
    tucm  = tucp0;

    int16_t * pdata2 = (int16_t*)pdata;

    pressure[0] = *pdata2++ + 100000;
    pressure[1] = *pdata2++ + 100000;

    temperature[0] = *pdata2++;
    temperature[1] = *pdata2++;

    mx = *pdata2++;
    my = *pdata2++;
    mz = *pdata2++;
  }

  Print();
  return 0;
}

int PressureMagData::Print()
{
  PRINT_INFO("Pressure :"<<"("<<id<<") "<<tucp0<<" "<<tucp1<<" "<<pressure[0]<<" "<<pressure[1]<<" "<<temperature[0]<<
  " "<<temperature[1]);
  PRINT_INFO("|| Mag Data: " <<tucm<<" "<<mx<<" "<<my<<" "<<mz<<" "<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Zigbee Rx status
//_______________________________________________________
int ZigBeeRxStatus::Parse(uint8_t * raw, int len, uint8_t type)
{
  //printf("got zigbee rx status!!\r\n");

  tpc = Timer::GetUnixTime();
  uint32_t * pdata = (uint32_t*)raw;
  tuc = *pdata++;
  err = *pdata++;

  raw = (uint8_t*)pdata;

  kid = *raw++;
  len = *raw++;
  lqi = *raw++;
  ed  = *raw++;
  status = *raw++;
  chan = *raw++;

  //Print();
  return 0;
}

int ZigBeeRxStatus::Print()
{
  PRINT_INFO("ZigbeeRxStatus : tuc: "<<tuc<<", ed: "<<(int)ed<<", chan: "<<(int)chan<<", err: "<<(int)err<<", kid: "<<(int)kid<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Motor status
//_______________________________________________________
int MotorStatusData::Parse(uint8_t * raw, int len, uint8_t type)
{
  tpc = Timer::GetUnixTime();
  uint32_t * pdata = (uint32_t*)raw;
  tuc = *pdata++;

  uint8_t * pdata2 = (uint8_t*)pdata;
  id = *pdata2++;

  uint16_t * pdata3 = (uint16_t*)pdata2;

  memcpy(crpm,pdata3,4*sizeof(uint16_t)); pdata3 += 4;
  memcpy(arpm,pdata3,4*sizeof(uint16_t)); pdata3 += 4;

  pdata2 = (uint8_t*)pdata3;
  memcpy(mstat,pdata2,4);

  //Print();
  return 0;
}

int MotorStatusData::Print()
{
  PRINT_INFO("MotorStatus : tuc: "<<tuc<<", commanded: ("<<(int)crpm[0]<<" "<<(int)crpm[1]<<" "<<(int)crpm[2]<<" "<<(int)crpm[3]<<" )"<<
                                         ", actual: ("<<(int)arpm[0]<<" "<<(int)arpm[1]<<" "<<(int)arpm[2]<<" "<<(int)arpm[3]<<" )"<<
                                         ", state: ("<<(int)mstat[0]<<" "<<(int)mstat[1]<<" "<<(int)mstat[2]<<" "<<(int)mstat[3]<<" )\n");
  return 0;
}
