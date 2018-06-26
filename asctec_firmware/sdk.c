/*

AscTec AutoPilot HL SDK v2.0

Copyright (c) 2011, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */

#include <string.h>
#include <math.h>
#include "main.h"
#include "sdk.h"
#include "uart.h"
#include "LL_HL_comm.h"
#include "pelican_ptu.h"

#define HUMMINGBIRD
//#define PELICAN
#define GYRO_900_DEG 0

static const float DEG2RADCONV = M_PI/180.0/1000;
#if GYRO_900_DEG
static const float DEGSEC2RADSECCONV = 0.0154*3.0*M_PI/180.0; // AscTec LL sends output/3 for high rate gyros
#else
static const float DEGSEC2RADSECCONV = 0.0154*M_PI/180.0;
#endif

#if defined(HUMMINGBIRD)
static const float LENGTH = 0.171;
//static const float KTHRUST = 8.0e-8; // Black props
static const float KTHRUST = 6e-8; // Gray props
//static const float KTHRUST = 6.16e-8; // AR Drone props
static const float PROP_RADIUS = 0.099;

static const float I[3][3] = { {2.64e-03, 0, 0},
                               {0, 2.64e-03, 0},
                               {0, 0, 4.96e-03} };
#elif defined(PELICAN)
static const float LENGTH = 0.21;
static const float KTHRUST = 1.63e-7; // APC 10x4.7
static const float PROP_RADIUS = 0.127;

static const float I[3][3] = { {1.0e-2, 0, 0},
                               {0, 1.0e-2, 0},
                               {0, 0, 2.0e-2} };
#endif

// http://wiki.asctec.de/display/AR/List+of+all+predefined+variables%2C+commands+and+parameters
static const float RPMSCALE = 0.026578073; // 1/37.625
static const int MIN_RPM = 1075;

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct RO_ALL_DATA RO_ALL_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;
struct SO3_CMD_INPUT SO3_cmd_input_tmp;
struct PWM_CMD_INPUT PWM_cmd_input_tmp;
struct OUTPUT_DATA Output_Data;
struct STATUS_DATA Status_Data;

static float normalize(float angle);

/* SDK_mainloop(void) is triggered @ 1kHz.
 *
 * RO_(Read Only) data is updated before entering this function
 * and can be read to obtain information for supervision or control
 *
 * WO_(Write Only) data is written to the LL processor after
 * execution of this function.
 *
 * WO_ and RO_ structs are defined in sdk.h
 *
 * The struct RO_ALL_Data (defined in sdk.h)
 * is used to read all sensor data, results of the data fusion
 * and R/C inputs transmitted from the LL-processor. This struct is
 * automatically updated at 1 kHz.
 * */

void SDK_mainloop(void)
{
  static struct SO3_CMD_INPUT SO3_cmd_input;
  static uint8_t command_initialized = 0;

  static struct PWM_CMD_INPUT PWM_cmd_input;
  static uint8_t pwm_initialized = 0;

  if(Ctrl_Input_updated)
  {
    memcpy(&SO3_cmd_input, &SO3_cmd_input_tmp, sizeof(SO3_cmd_input));
    Ctrl_Input_updated = 0;
    command_initialized = 1;

#if 0 // Loopback (for debugging)
    if((sizeof(SO3_cmd_input)) < ringbuffer(RBFREE, 0, 0))
      UART_SendPacket(&SO3_cmd_input, sizeof(SO3_cmd_input), TYPE_SO3_CMD);
#endif
  }

  if(PWM_Input_updated)
  {
    memcpy(&PWM_cmd_input, &PWM_cmd_input_tmp, sizeof(PWM_cmd_input));
    PWM_Input_updated = 0;
    pwm_initialized = 1;
  }

  uint8_t motors_des = 0;
  int u[4] = {0, 0, 0, 0};

  if(command_initialized)
  {
    const float fx = 2e-3f*SO3_cmd_input.force[0];
    const float fy = 2e-3f*SO3_cmd_input.force[1];
    const float fz = 2e-3f*SO3_cmd_input.force[2];

    float qw = 8e-3f*SO3_cmd_input.des_qw;
    float qx = 8e-3f*SO3_cmd_input.des_qx;
    float qy = 8e-3f*SO3_cmd_input.des_qy;
    float qz = 8e-3f*SO3_cmd_input.des_qz;
    float q_norm = sqrtf(qw*qw+qx*qx+qy*qy+qz*qz);
    qw = qw/q_norm;
    qx = qx/q_norm;
    qy = qy/q_norm;
    qz = qz/q_norm;

    const float des_angvel_x = 1e-3f*SO3_cmd_input.angvel_x;
    const float des_angvel_y = 1e-3f*SO3_cmd_input.angvel_y;
    const float des_angvel_z = 1e-3f*SO3_cmd_input.angvel_z;

    const float Rd11 = qw*qw + qx*qx - qy*qy - qz*qz;
    const float Rd12 = 2.0f*(qx*qy - qw*qz);
    const float Rd13 = 2.0f*(qx*qz + qw*qy);
    const float Rd21 = 2.0f*(qx*qy + qw*qz);
    const float Rd22 = qw*qw - qx*qx + qy*qy - qz*qz;
    const float Rd23 = 2.0f*(qy*qz - qw*qx);
    const float Rd31 = 2.0f*(qx*qz - qw*qy);
    const float Rd32 = 2.0f*(qy*qz + qw*qx);
    const float Rd33 = qw*qw - qx*qx - qy*qy + qz*qz;

    const float kR1 = 2e-2f*SO3_cmd_input.kR[0];
    const float kR2 = 2e-2f*SO3_cmd_input.kR[1];
    const float kR3 = 2e-2f*SO3_cmd_input.kR[2];

    const float kOm1 = 1e-2f*SO3_cmd_input.kOm[0];
    const float kOm2 = 1e-2f*SO3_cmd_input.kOm[1];
    const float kOm3 = 1e-2f*SO3_cmd_input.kOm[2];

    const float kf = (KTHRUST - 1e-11f*SO3_cmd_input.kf_correction);
    // km = (Cq/Ct)*Dia*kf
    // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
    const float km = 0.14f*PROP_RADIUS*kf;

    const float r_correction = 4e-4f*SO3_cmd_input.angle_corrections[0];
    const float p_correction = 4e-4f*SO3_cmd_input.angle_corrections[1];

    motors_des = SO3_cmd_input.enable_motors;

    // Switch from X front, Y right, Z down to X front, Y left, Z up
    const float rad_r = normalize(DEG2RADCONV*RO_ALL_Data.angle_roll + r_correction);
    const float rad_p = normalize(-DEG2RADCONV*RO_ALL_Data.angle_pitch + p_correction);

    const float rad_y = SO3_cmd_input.use_external_yaw ? normalize(1e-4f*SO3_cmd_input.cur_yaw) :
      normalize(-DEG2RADCONV*RO_ALL_Data.angle_yaw);

    const float Om1 = DEGSEC2RADSECCONV*RO_ALL_Data.angvel_roll;
    const float Om2 = -DEGSEC2RADSECCONV*RO_ALL_Data.angvel_pitch;
    const float Om3 = -DEGSEC2RADSECCONV*RO_ALL_Data.angvel_yaw;

    const float cr = cosf(rad_r);
    const float sr = sinf(rad_r);
    const float cp = cosf(rad_p);
    const float sp = sinf(rad_p);
    const float cy = cosf(rad_y);
    const float sy = sinf(rad_y);

    // LL sends euler angles in Z-Y-X convention
    const float R11 = cy*cp;
    const float R12 = cy*sp*sr - cr*sy;
    const float R13 = sy*sr + cy*cr*sp;
    const float R21 = cp*sy;
    const float R22 = cy*cr + sy*sp*sr;
    const float R23 = cr*sy*sp - cy*sr;
    const float R31 = -sp;
    const float R32 = cp*sr;
    const float R33 = cp*cr;

#if 0
    float tr = sqrtf(1.0f +
        R11*Rd11 + R12*Rd12 + R13*Rd13 +
        R21*Rd21 + R22*Rd22 + R23*Rd23 +
        R31*Rd31 + R32*Rd32 + R33*Rd33);
    float Psi = 2.0f - tr;

    float force = 0.0;
    if (Psi < 1) // Position control stability guaranteed only when Psi < 1
      force = fx*R13 + fy*R23 + fz*R33;

    float tri = 1/tr;
    float eR1 =
      0.5f*tri*(-R13*Rd12 + R12*Rd13 - R23*Rd22 + R22*Rd23 - R33*Rd32 + R32*Rd33);
    float eR2 =
      0.5f*tri*(R13*Rd11 - R11*Rd13 + R23*Rd21 - R21*Rd23 + R33*Rd31 - R31*Rd33);
    float eR3 =
      0.5f*tri*(-R12*Rd11 + R11*Rd12 - R22*Rd21 + R21*Rd22 - R32*Rd31 + R31*Rd32);
#else
    const float Psi = 0.5f*(3.0f - (Rd11*R11 + Rd21*R21 + Rd31*R31 +
          Rd12*R12 + Rd22*R22 + Rd32*R32 + Rd13*R13 + Rd23*R23 + Rd33*R33));

    float force = 0.0f;
    if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
      force = fx*R13 + fy*R23 + fz*R33;

    const float eR1 = 0.5f*(R12*Rd13 - R13*Rd12 + R22*Rd23 - R23*Rd22 + R32*Rd33 - R33*Rd32);
    const float eR2 = 0.5f*(R13*Rd11 - R11*Rd13 - R21*Rd23 + R23*Rd21 - R31*Rd33 + R33*Rd31);
    const float eR3 = 0.5f*(R11*Rd12 - R12*Rd11 + R21*Rd22 - R22*Rd21 + R31*Rd32 - R32*Rd31);
#endif

    // Omd = R.transpose()*Rd*des_ang_vel;
    const float Omd1 = des_angvel_x*(R11*Rd11 + R21*Rd21 + R31*Rd31) + des_angvel_y*(R11*Rd12 + R21*Rd22 + R31*Rd32) +
      des_angvel_z*(R11*Rd13 + R21*Rd23 + R31*Rd33);
    const float Omd2 = des_angvel_x*(R12*Rd11 + R22*Rd21 + R32*Rd31) + des_angvel_y*(R12*Rd12 + R22*Rd22 + R32*Rd32) +
      des_angvel_z*(R12*Rd13 + R22*Rd23 + R32*Rd33);
    const float Omd3 = des_angvel_x*(R13*Rd11 + R23*Rd21 + R33*Rd31) + des_angvel_y*(R13*Rd12 + R23*Rd22 + R33*Rd32) +
      des_angvel_z*(R13*Rd13 + R23*Rd23 + R33*Rd33);

    const float eOm1 = Om1 - Omd1;
    const float eOm2 = Om2 - Omd2;
    const float eOm3 = Om3 - Omd3;

    const float in1 = Om2*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3) - Om3*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3);
    const float in2 = Om3*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3) - Om1*(I[2][0]*Om1 + I[2][1]*Om2 + I[2][2]*Om3);
    const float in3 = Om1*(I[1][0]*Om1 + I[1][1]*Om2 + I[1][2]*Om3) - Om2*(I[0][0]*Om1 + I[0][1]*Om2 + I[0][2]*Om3);

    const float uF = force;
    const float uM1 = -kR1*eR1 - kOm1*eOm1 + in1;
    const float uM2 = -kR2*eR2 - kOm2*eOm2 + in2;
    const float uM3 = -kR3*eR3 - kOm3*eOm3 + in3;

    const float kf_inv = 1/kf;
    const float kfl_inv = kf_inv*(1/LENGTH);
    const float km_inv = 1/km;

    // Rotor numbering is:
    //   *1*    Front
    // 3     4
    //    2
    float w_sq[4];
    w_sq[0] = (uF*kf_inv - 2*uM2*kfl_inv + uM3*km_inv)/4;
    w_sq[1] = (uF*kf_inv + 2*uM2*kfl_inv + uM3*km_inv)/4;
    w_sq[2] = (uF*kf_inv + 2*uM1*kfl_inv - uM3*km_inv)/4;
    w_sq[3] = (uF*kf_inv - 2*uM1*kfl_inv - uM3*km_inv)/4;

    float w[4] = {0,0,0,0};
    for(int i = 0; i < 4; i++)
    {
      if(w_sq[i] > 0)
        w[i] = sqrtf(w_sq[i]);
    }

    for(int i = 0; i < 4; i++)
    {
      u[i] = lrintf(RPMSCALE*(w[i] - MIN_RPM));

      // Input range is 1-200, 0 turns off motor
      if(u[i] < 1)
        u[i] = 1;
      else if(u[i] > 200)
        u[i] = 200;
    }
  }
#if 1
  //uint8_t motors_on = (LL_1khz_attitude_data.status2 & 0x01);
  const uint8_t motors_on = (RO_ALL_Data.motor_rpm[0] > 0 ||
                             RO_ALL_Data.motor_rpm[1] > 0 ||
                             RO_ALL_Data.motor_rpm[2] > 0 ||
                             RO_ALL_Data.motor_rpm[3] > 0);
  static unsigned int counter = 0;
  if(motors_des != motors_on)
  {
    WO_SDK.ctrl_mode = 0x02; // 0x02: attitude and throttle control: commands
    // are input for standard attitude controller
    WO_SDK.ctrl_enabled = 1; // enable control by HL processor
    WO_SDK.disable_motor_onoff_by_stick = 0; // allow RC control
    WO_CTRL_Input.ctrl = 0x0C; // enable throttle control and yaw control
    WO_CTRL_Input.thrust = 0;
    WO_CTRL_Input.yaw = -2047;
    counter = 0;
  }
  else
  {
    if(counter < 50) // Delay between turning motors on and sending command
      counter++;
    else
    {
      WO_SDK.ctrl_mode = 0x00; // direct individual motor control
      WO_SDK.ctrl_enabled = 1; // enable control by HL processor
      WO_SDK.disable_motor_onoff_by_stick = 0; // allow RC control

      WO_Direct_Individual_Motor_Control.motor[0] = u[0];
      WO_Direct_Individual_Motor_Control.motor[1] = u[1];
      WO_Direct_Individual_Motor_Control.motor[2] = u[2];
      WO_Direct_Individual_Motor_Control.motor[3] = u[3];
      WO_Direct_Individual_Motor_Control.motor[4] = 0;
      WO_Direct_Individual_Motor_Control.motor[5] = 0;
    }
  }
#else
  // Just for testing
  WO_SDK.ctrl_mode = 0x00; // direct individual motor control
  WO_SDK.ctrl_enabled = 1; // enable control by HL processor
  WO_SDK.disable_motor_onoff_by_stick = 0; // allow RC control

  WO_Direct_Individual_Motor_Control.motor[0] = (RO_ALL_Data.channel[6]/4000)*100 + 50;
  WO_Direct_Individual_Motor_Control.motor[1] = 0;//RO_ALL_Data.channel[2]/21;
  WO_Direct_Individual_Motor_Control.motor[2] = 0;//RO_ALL_Data.channel[2]/21;
  WO_Direct_Individual_Motor_Control.motor[3] = 0;//RO_ALL_Data.channel[2]/21;
  WO_Direct_Individual_Motor_Control.motor[4] = 0;
  WO_Direct_Individual_Motor_Control.motor[5] = 0;
#endif

  if (pwm_initialized)
  {
    // The lower limit of servo_val is 6000 and the upper limit is 12000
    // for a valid pwm signal (i.e. 1ms to 2ms pulse width)
    int servo_val = PWM_cmd_input.pwm[0] * 6000u / 255u + 6000u;
    PTU_update(servo_val);
  }
}

static float normalize(float angle)
{
  const float pi = M_PI;
  while (angle > pi)
    angle -= 2*pi;

  while (angle < -pi)
    angle += 2*pi;

  return angle;
}
