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
  uint8_t enable_motors:1;
  uint8_t use_external_yaw:1;
  uint8_t use_angle_corrections:1;
  uint8_t seq;
};

// TODO: Change to:
/*
struct SO3_CMD_INPUT
{
  // Scaling factors
  int16_t force[3]; // /500
  int8_t des_qx, des_qy, des_qz, des_qw; // /125
  uint8_t kR[3]; // /50
  uint8_t kOm[3]; // /100
  int16_t cur_yaw; // /1e4
  int16_t kf_correction; // /1e11;
  uint8_t enable_motors:1;
  uint8_t use_external_yaw:1;
  uint8_t seq;
};
*/

#define TYPE_OUTPUT_DATA 'd'
struct OUTPUT_DATA
{
  uint16_t cpu_load;
  uint16_t voltage;
  int16_t roll, pitch, yaw;
  int16_t ang_vel[3];
  int16_t acc[3];
  int16_t dheight;
  int32_t height;
  int16_t mag[3];
  uint8_t radio[8];
  uint8_t rpm[4];
};

#endif
