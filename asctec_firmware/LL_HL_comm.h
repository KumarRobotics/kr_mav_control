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

 */

#ifndef LL_HL_COMM_
#define LL_HL_COMM_

//system flags
#define SF_PAGE_BIT1 						0x01
#define SF_PAGE_BIT2 						0x02
#define SF_SSP_ACK							0x04
#define SF_GPS_NEW							0x08
#define SF_HL_CONTROL_ENABLED 				0x10
#define SF_DIRECT_MOTOR_CONTROL 			0x20
#define SF_WAYPOINT_MODE					0x40
#define SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL	0x80
#define SF_SDK_DISABLE_MOTORONOFF_BY_STICK	0x100
#define SF_NEW_SDK							0x8000


//ctrl_flags
//scientific control
#define HL_CTRL_PITCH			0x01
#define HL_CTRL_ROLL			0x02
#define HL_CTRL_YAW 			0x04
#define HL_CTRL_THRUST			0x08
#define HL_CTRL_HEIGHT_ENABLED	0x10
#define HL_CTRL_GPS_ENABLED		0x20

//direct motor control
#define HL_CTRL_MOTORS_ONOFF_BY_RC	0x01


#define WP_CMD_SINGLE_WP_PART1 	0x81 //internal use!
#define WP_CMD_SINGLE_WP_PART2 	0x82 //internal use!

//Slow Data Channel defines
//declination data [short]
#define SDC_DECLINATION 		0x01
//declination data [short]
#define SDC_INCLINATION 		0x12

//Slow Data Up Channel defines
#define SUDC_NONE				0x00
//flight time. flighttime in slowDataUpChannelShort
#define SUDC_FLIGHTTIME			0x01
//set camera type. Cameratype in slowDataUpChannelShort
#define SUDC_SETCAMERA			0x02
//set payload options. payloadoptions in slowDataUpChannelShort
#define SUDC_SETPAYLOADOPTIONS	0x03
//set camera mount roll angle calibration
#define SUDC_SETCAMERAROLLANGLECALIB 0x04
//set camera mount roll angle calibration and store to eeprom
#define SUDC_SETCAMERAROLLANGLECALIBANDSTORE 0x05
//navigation status
#define SUDC_NAVSTATUS 0x06
//distance to waypoint
#define SUDC_DISTTOWP  0x07
//waypoint ack trigger
#define SUDC_WPACKTRIGGER 0x08

void LL_write_ctrl_data(uint8_t);
int HL2LL_write_cycle(void);
void SSP_rx_handler_HL(uint8_t);
void SSP_data_distribution_HL(void);
struct LL_ATTITUDE_DATA
{
	uint16_t system_flags;	//GPS data acknowledge, etc.

	int16_t angle_pitch;	//angles [deg*100]
	int16_t angle_roll;
	uint16_t angle_yaw;

	int16_t angvel_pitch;	//angular velocities; bias-free [0.015 deg/s]
	int16_t angvel_roll;
	int16_t angvel_yaw;

				//<-- 14 bytes @ 1kHz
				//--> 3x 26 bytes @ 333 Hz
				//=> total = 40 bytes @ 1 kHz
//-----------------------------PAGE0
	uint8_t RC_data[10];	//8 channels @ 10 bit

	int latitude_best_estimate;	//GPS data fused with all other sensors
	int longitude_best_estimate;
	int16_t acc_x;		//accelerations [mg]
	int16_t acc_y;
	int16_t acc_z;

	uint16_t temp_gyro;
//-----------------------------PAGE1
	uint8_t motor_data[16];	//PWM 0..7, speed 8..15

	int16_t speed_x_best_estimate;
	int16_t speed_y_best_estimate;
	int height;		//height [mm]
	int16_t dheight;		//differentiated height[mm/s]
//------------------------------PAGE2
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;

	int16_t cam_angle_pitch;
	int16_t cam_angle_roll;
	int16_t cam_status;

	int16_t battery_voltage1;
	int16_t battery_voltage2;
	int16_t flightMode;
	int16_t slowDataUpChannelDataShort; //former flight_time
	int16_t cpu_load;
	int16_t status;
	int16_t status2; //Bits 7..1: slowDataUpChannelSelect (7bit) Bit0:flying Bit15..8:active Motors

};

extern struct LL_ATTITUDE_DATA LL_1khz_attitude_data;

struct LL_CONTROL_INPUT
{
	uint16_t system_flags;
							//bit 0: page_select
							//bit 1: reserved (page_select)
							//bit 2: SSP_ack
							//bit 3: GPS new
							//bit 4: HL controller enabled
							//bit 5: 0 -> "scientific" commands
							//       1 -> direct motor commands
							//bit 6: waypoint mode

	uint16_t ctrl_flags;
							//bit 0..3:
							// pitch, roll, yaw, thrust enable bits
							//bit 4: height control enabled
							//bit 5: GPS_control enabled
							//bit 8..15: waypoint command if waypoint mode is active or POI options if POI mode is active

	int16_t pitch, roll, yaw, thrust; 		//"scientific interface"
	uint8_t direct_motor_control[8];		//direct motor commands: pitch, roll, yaw, throttle, 4xDNC
							//or motor 0..7 (Falcon)

							//<-- 20 bytes @ 1kHz
							//--> 2x18 bytes @ 500 Hz
							//=> total = 38 bytes @ 1kHz

	int latitude;					//data received from GPS-unit
	int longitude;
	int height;
	int16_t speed_x;
	int16_t speed_y;
	int16_t status;
//-----------------------------

	uint16_t hor_accuracy;
	uint16_t vert_accuracy;
	uint16_t speed_accuracy;
	uint16_t numSV;
	uint16_t heading;
	int16_t battery_voltage_1, battery_voltage_2; 	//battery voltage read by HL-ADC [mV]
	uint8_t slowDataChannelSelect; // these three vars define a slow data transfer channel. the select byte defines which data is in the data channel
	uint8_t slowDataChannelDataChar;
	int16_t slowDataChannelDataShort;
};
extern struct LL_CONTROL_INPUT LL_1khz_control_input;


#endif /*LL_HL_COMM_*/
