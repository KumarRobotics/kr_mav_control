#ifndef __QUADROTOR_MSGS_COMM_TYPES_H__
#define __QUADROTOR_MSGS_COMM_TYPES_H__

#define TYPE_SO3_CMD 's'
struct SO3_CMD_INPUT
{
  int16_t force[3];
  int16_t des_qx, des_qy, des_qz, des_qw;
  int16_t kR[3];
  int16_t kOm[3];
  int16_t cur_yaw;
  int16_t corrections[3]; // thrust, roll, pitch
  int8_t enable_motors;
  int8_t use_external_yaw;
  int8_t use_angle_corrections;
  int8_t seq;
};

#define TYPE_OUTPUT_DATA 'd'
struct OUTPUT_DATA
{
  uint16_t cpu_load;
  uint16_t voltage;
  int16_t roll, pitch, yaw;
  int16_t acc[3];
  int16_t ang_vel[3];
  int16_t dheight;
  int32_t height;
  int16_t mag[3];
  uint8_t radio[8];
};

#endif
