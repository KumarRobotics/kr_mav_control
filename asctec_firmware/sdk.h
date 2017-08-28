/*

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

 ** Checked for Kumar Lab by Andrew Block 5/20/16 **

 */

#ifndef SDK_
#define SDK_

#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void SDK_mainloop(void);

#define TYPE_SO3_CMD 's'
struct SO3_CMD_INPUT
{
  // Scaling factors when decoding
  int16_t force[3]; // /500 (range: +-65 N on each axis)
  int8_t des_qx, des_qy, des_qz, des_qw; // /125
  int16_t angvel_x, angvel_y, angvel_z; // /1000 (range: +-32 rad/s)
  uint8_t kR[3]; // /50 (range: 0-5.1)
  uint8_t kOm[3]; // /100 (range: 0-2.5)
  int16_t cur_yaw; // /1e4
  int16_t kf_correction; // /1e11 (range: +-3.2e-7)
  int8_t angle_corrections[2]; // roll,pitch /2500 (range: +-0.05 rad)
  uint8_t enable_motors:1;
  uint8_t use_external_yaw:1;
  uint8_t seq;
};
extern struct SO3_CMD_INPUT SO3_cmd_input_tmp;

#define TYPE_PWM_CMD 'w'
struct PWM_CMD_INPUT
{
  // Scaling factors when decoding
  uint8_t pwm[2]; // pwm1,pwm2 *255
};
extern struct PWM_CMD_INPUT PWM_cmd_input_tmp;

#define TYPE_STATUS_DATA 'c'
struct STATUS_DATA
{
  uint16_t loop_rate;
  uint16_t voltage;
  uint8_t seq;
};
extern struct STATUS_DATA Status_Data;

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
  //uint8_t rpm[4];
  uint8_t seq;
};
extern struct OUTPUT_DATA Output_Data;

//--- general commands -----------------------------------------------------------------------------------------------------------------------------------------------
struct WO_SDK_STRUCT {

	uint8_t ctrl_mode;

	uint8_t ctrl_enabled; //0x00: control commands are ignored by LL processor
								//0x01: control commands are accepted by LL processor

	uint8_t disable_motor_onoff_by_stick; // if true, "minimum thrust + full yaw" command will not start/stop motors
};
extern struct WO_SDK_STRUCT WO_SDK;

//--- read sensor data -----------------------------------------------------------------------------------------------------------------------------------------------
struct RO_ALL_DATA {

	//remote control data
		uint16_t channel[8];
		/*
		 * channel[0]: pitch
		 * channel[1]: roll
		 * channel[2]: thrust
		 * channel[3]: yaw
		 * channel[4]: serial interface enable/disable (>2048 enabled)
		 * channel[5]: manual / height control / GPS + height control (0 -> manual mode; 2048 -> height mode; 4095 -> GPS mode)
		 *
		 * range of each channel: 0..4095
		 */

	//angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
	    int angle_pitch;
	    int angle_roll;
	    int angle_yaw;

	//angular velocities, bias free, in 0.0154 deg/s (=> 64.8 = 1 deg/s)
	    int angvel_pitch;
	    int angvel_roll;
	    int angvel_yaw;

	//acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g, body frame coordinate system
	    int16_t acc_x;
	    int16_t acc_y;
	    int16_t acc_z;

	//magnetic field sensors output, offset free and scaled to +-2500 = +- earth field strength;
	    int Hx;
	    int Hy;
	    int Hz;

	//RPM measurements (0..200)
	/*
	 * Quadcopter (AscTec Hummingbird, AscTec Pelican)
	 *
	 * motor[0]: front
	 * motor[1]: rear
	 * motor[2]: left
	 * motor[3]: right
	 *
	 */

	/*
	 * Hexcopter (AscTec Firefly)
	 *
	 * motor[0]: front-left
	 * motor[1]: left
	 * motor[2]: rear-left
	 * motor[3]: rear-right
	 * motor[4]: right
	 * motor[5]: front-right
	 *
	 */
	    uint8_t motor_rpm[6];

	//latitude/longitude in degrees * 10^7
		int GPS_latitude;
		int GPS_longitude;

	//GPS height in mm
		int GPS_height;

	//speed in x (E/W) and y(N/S) in mm/s
		int GPS_speed_x;
		int GPS_speed_y;

	//GPS heading in deg * 1000
		int GPS_heading;

	//accuracy estimates in mm and mm/s
		unsigned int GPS_position_accuracy;
		unsigned int GPS_height_accuracy;
		unsigned int GPS_speed_accuracy;

	//number of satellites used in NAV solution
		unsigned int GPS_sat_num;

	// GPS status information: Bit7...Bit3: 0; Bit 2: longitude direction; Bit1: latitude direction; Bit 0: GPS lock
		int GPS_status;

		unsigned int GPS_time_of_week;	//[ms] (1 week = 604,800 s)
		uint16_t GPS_week;		// starts from beginning of year 1980

	//height in mm (after data fusion)
		int fusion_height;

	//diff. height in mm/s (after data fusion)
		int fusion_dheight;

	//GPS data fused with all other sensors (best estimations)
		int fusion_latitude; 	//Fused latitude in degrees * 10^7
		int fusion_longitude;	//Fused longitude in degrees * 10^7

		int16_t fusion_speed_x; 	//[mm/s]
		int16_t fusion_speed_y;	//[mm/s]

}; //************************************************************
extern struct RO_ALL_DATA RO_ALL_Data;


struct RO_RC_DATA {

	uint16_t channel[8];
	/*
	 * channel[0]: pitch
	 * channel[1]: roll
	 * channel[2]: thrust
	 * channel[3]: yaw
	 * channel[4]: serial interface enable/disable (>2048 enabled)
	 * channel[5]: manual / height control / GPS + height control (0-manual-1500-height-2500-GPS-4095)
	 *
	 * range of each channel: 0..4095
	 */
};
extern struct RO_RC_DATA RO_RC_Data;



//--- send commands -----------------------------------------------------------------------------------------------------------------------------------------------
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL
{
	uint8_t motor[8];

	/*
	 * commands will be directly interpreted by each motor individually
	 *
	 * range: 0..200 = 0..100 %; 0 = motor off! Please check for command != 0 during flight, as a motor restart might take > 1s!
	 */

	/*
	 * Quadcopter (AscTec Hummingbird, AscTec Pelican)
	 *
	 * motor[0]: front
	 * motor[1]: rear
	 * motor[2]: left
	 * motor[3]: right
	 *
	 */

	/*
	 * Hexcopter (AscTec Firefly)
	 *
	 * motor[0]: front-left
	 * motor[1]: left
	 * motor[2]: rear-left
	 * motor[3]: rear-right
	 * motor[4]: right
	 * motor[5]: front-right
	 *
	 */

};
extern struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;

struct WO_DIRECT_MOTOR_CONTROL //direct motor commands with standard output mapping
{
	uint8_t pitch;
	uint8_t roll;
	uint8_t yaw;
	uint8_t thrust;

	/*
	 * commands will be directly interpreted by the mixer
	 * running on each of the motor controllers
	 *
	 * range (pitch, roll, yaw commands): 0..200 = - 100..+100 %
	 * range of thrust command: 0..200 = 0..100 %
	 */

};
extern struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;


//attitude commands

struct WO_CTRL_INPUT {

	int16_t pitch;	//pitch input: -2047..+2047 (0=neutral)
	int16_t roll;		//roll input: -2047..+2047	(0=neutral)
	int16_t yaw;		//(=R/C Stick input) -2047..+2047 (0=neutral)
	int16_t thrust;	//collective: 0..4095 = 0..100%
	int16_t ctrl;				/*control byte:
							bit 0: pitch control enabled
							bit 1: roll control enabled
							bit 2: yaw control enabled
							bit 3: thrust control enabled
							bit 4: height control enabled
							bit 5: GPS position control enabled
							*/

	//max. pitch/roll (+-2047) equals 51.2 deg angle if GPS position control is disabled
	//max. pitch/roll (+-2047) equals approx. 3 m/s if GPS position control is active (GPS signal needed!)

};
extern struct WO_CTRL_INPUT WO_CTRL_Input;


//waypoint commands

struct WAYPOINT { //waypoint definition
//always set to 1
  unsigned int wp_activated;

//use WPPROP_* defines to set waypoint properties
  uint8_t properties;

//max. speed to travel to waypoint in % (default 100)
  uint8_t max_speed;

//time to stay at a waypoint (XYZ) in 1/100 s
  uint16_t time;

//position accuracy to consider a waypoint reached in mm (recommended: 3000 (= 3.0 m))
  uint16_t pos_acc;

//chksum = 0xAAAA + wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number;
  int16_t chksum;

 //relative waypoint coordinates in mm 	// longitude in abs coords e.g. 113647430 (= 11.3647430 deg; angle in degrees * 10^7)
  int X;
 //relative waypoint coordinates in mm  	// latitude in abs coords e.g. 480950480 (= 48.0950480 deg; angle in degrees * 10^7)
  int Y;

//yaw angle
  int yaw; // 1/1000 deg

//height over 0 reference in mm
  int height;

};
extern struct WAYPOINT wpToLL;

//set waypoint properties with WPPROP_* defines (wpToLL.properties)
#define WPPROP_ABSCOORDS 			0x01	//if set waypoint is interpreted as absolute coordinates, else relative coordinates
#define WPPROP_HEIGHTENABLED 		0x02  	//set new height at waypoint
#define WPPROP_YAWENABLED 			0x04	//set new yaw-angle at waypoint
#define WPPROP_AUTOMATICGOTO		0x10 	//if set, vehicle will not wait for a goto command, but goto this waypoint directly

extern uint8_t wpCtrlWpCmd; //choose actual waypoint command from WP_CMD_* defines
#define WP_CMD_SINGLE_WP 	0x01 //fly to single waypoint
#define WP_CMD_LAUNCH		0x02 //launch to 10m at current position
#define WP_CMD_LAND			0x03 //land at current position
#define WP_CMD_GOHOME		0x04 //come home at current height. Home position is set when motors are started (valid GPS signal mandatory!) or with WP_CMD_SETHOME
#define WP_CMD_SETHOME		0x05 //save current vehicle position as home position
#define WP_CMD_ABORT		0x06 //abort navigation (stops current waypoint flying)

extern uint8_t wpCtrlWpCmdUpdated; //send current waypoint command to LL
extern uint8_t wpCtrlAckTrigger; //acknowledge from LL processor that waypoint was accepted

//Waypoint navigation status
extern uint16_t wpCtrlNavStatus; //check navigation status with WP_NAVSTAT_* defines
#define WP_NAVSTAT_REACHED_POS 			0x01	//vehicle has entered a radius of WAYPOINT.pos_acc and time to stay is not necessarily over
#define WP_NAVSTAT_REACHED_POS_TIME		0x02 	//vehicle is within a radius of WAYPOINT.pos_acc and time to stay is over
#define WP_NAVSTAT_20M					0x04 	//vehicle within a 20m radius of the waypoint
#define WP_NAVSTAT_PILOT_ABORT			0x08	//waypoint navigation aborted by safety pilot (any stick was moved)

extern uint16_t wpCtrlDistToWp; //current distance to the current waypoint in dm (=10 cm)

#endif /*SDK_*/
