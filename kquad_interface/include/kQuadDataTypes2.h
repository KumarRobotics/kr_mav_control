/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#ifndef KQUAD_DATA_TYPES2
#define KQUAD_DATA_TYPES2

typedef struct
{
  uint8_t  id;
  uint8_t  chan;
  uint16_t cntr;

  int16_t  thrust;
  int16_t  roll;
  int16_t  pitch;
  int16_t  yaw;
} __attribute__ ((__packed__)) QuadCmd1;

typedef struct
{
  uint8_t  id;
  uint8_t  chan;
  uint16_t cntr;

  int16_t  rpm0;
  int16_t  rpm1;
  int16_t  rpm2;
  int16_t  rpm3;
} __attribute__ ((__packed__)) QuadCmd2;

typedef struct
{
  uint8_t  id;
  uint8_t  chan;
  uint16_t cntr;

  int16_t  thrust;
  int16_t  roll;
  int16_t  pitch;
  int16_t  yaw;

  int16_t droll;    //0.002 rad  //maximum 65.536 radians/sec = 3755 deg/sec
  int16_t dpitch;
  int16_t dyaw;

  uint8_t kp_roll;  //scale = 2  for nano???
  uint8_t kp_pitch;
  uint8_t kp_yaw;

  uint8_t kd_roll;  //scale = 0.1  for nano???
  uint8_t kd_pitch;
  uint8_t kd_yaw;
} __attribute__ ((__packed__)) QuadCmd3;

typedef struct
{
  uint8_t  id;
  uint8_t  chan;
  uint16_t cntr;

  int16_t  thrust;
  int16_t  roll;
  int16_t  pitch;
  int16_t  yaw;

  int16_t droll;    //0.002 rad  //maximum 65.536 radians/sec = 3755 deg/sec
  int16_t dpitch;
  int16_t dyaw;

  uint8_t kp_roll;  //scale = 2  for nano???
  uint8_t kp_pitch;
  uint8_t kp_yaw;

  uint8_t kd_roll;  //scale = 0.1  for nano???
  uint8_t kd_pitch;
  uint8_t kd_yaw;

  int16_t mx;   //0.1 gram
  int16_t my;
  int16_t mz;

} __attribute__ ((__packed__)) QuadCmd4;


typedef struct
{
  uint32_t cntr;
  float roll;
  float pitch;
  float yaw;
  float wroll;
  float wpitch;
  float wyaw;
  float accx;
  float accy;
  float accz;
} __attribute__ ((__packed__)) FilteredImuData;


typedef struct
{
  uint32_t tuc;
  uint16_t voltage;
  uint16_t current;
  uint8_t  id;
  uint8_t  state;
  uint8_t  autoCmdCntr;
  uint8_t  rcCmdCntr;
  uint8_t  checkErrorCntr;
  uint8_t  sigstren;
} __attribute__ ((__packed__)) QuadStatus;

typedef struct
{
  uint8_t  id;

  uint32_t tucp1;
  uint32_t tucp2;
  uint32_t tucm;

  uint32_t pressure[2];
  uint16_t temperature[2];
  uint16_t mxyz[3];
} __attribute__ ((__packed__)) PressureMagData1;

#endif
