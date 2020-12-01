#ifndef QUADROTOR_MSGS_COMM_TYPES_H
#define QUADROTOR_MSGS_COMM_TYPES_H

#include <stdint.h>

#define TYPE_SO3_CMD 's'
struct SO3_CMD_INPUT
{
  // Scaling factors when decoding
  int16_t force[3];                       // /500 (range: +-65 N on each axis)
  int8_t des_qx, des_qy, des_qz, des_qw;  // /125
  int16_t angvel_x, angvel_y, angvel_z;   // /1000 (range: +-32 rad/s)
  uint8_t kR[3];                          // /50 (range: 0-5.1)
  uint8_t kOm[3];                         // /100 (range: 0-2.5)
  int16_t cur_yaw;                        // /1e4
  int16_t kf_correction;                  // /1e11 (range: +-3.2e-7)
  int8_t angle_corrections[2];            // roll,pitch /2500 (range: +-0.05 rad)
  uint8_t enable_motors : 1;
  uint8_t use_external_yaw : 1;
  uint8_t seq;
};

#define TYPE_STATUS_DATA 'c'
struct STATUS_DATA
{
  uint16_t loop_rate;
  uint16_t voltage;
  uint8_t seq;
};

#define TYPE_OUTPUT_DATA 'd'
struct OUTPUT_DATA
{
  uint16_t loop_rate;
  uint16_t voltage;
  int16_t roll, pitch, yaw;
  int16_t ang_vel[3];
  int16_t acc[3];
  int16_t dheight;
  int32_t height;
  int16_t mag[3];
  uint8_t radio[8];
  uint8_t rpm[4];
  uint8_t seq;
};

#define TYPE_TRPY_CMD 'p'
struct TRPY_CMD
{
  int16_t thrust;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t current_yaw;
  uint8_t enable_motors : 1;
  uint8_t use_external_yaw : 1;
};

#define TYPE_PWM_CMD 'w'
struct PWM_CMD_INPUT
{
  // Scaling factors when decoding
  uint8_t pwm[2];  // /255
};

#endif
