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

//flight modes / error flags
#define FM_ACC 						0x01
#define FM_HEIGHT   				0x02
#define FM_POS 						0x04
#define FM_SPEED   					0x08
#define FM_COMPASS_FAILURE			0x10
#define FM_SCIENTIFIC 				0x20
#define FM_SCIENTIFIC_ACTIVE		0x40
#define FM_EMERGENCY				0x80
#define FM_CALIBRATION_ERROR		0x100
#define FM_CALIBRATION_ERROR_GYROS 	0x200
#define FM_CALIBRATION_ERROR_ACC   	0x400
#define FM_ADC_STARTUP_ERROR        0x800
#define FM_FLYING_BLOCKED	        0x1000
#define FM_MOTORSTART_WAS_BLOCKED   0x2000
#define FM_MAG_FIELD_STRENGTH_ERROR 0x4000
#define FM_MAG_INCLINATION_ERROR	0x8000

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

//flags used to detect if vehicle is a quad or hex
#define OM_QUAD 0	//quadrocopter mixer
#define OM_HEX 	1	//hex mixer

//Firefly LED
extern unsigned char fireflyLedEnabled;

//Slow Data Channel defines
//declination data [short]
#define SDC_DECLINATION 		0x01
//declination data [short]
#define SDC_INCLINATION 		0x12
//build info define
#define SDC_BUILDINFO 			0x05
//emergency mode
#define SDC_EM_MODE				0x18

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
#define SUDC_NAVSTATUS 			0x06
//distance to waypoint
#define SUDC_DISTTOWP  			0x07
//waypoint ack trigger
#define SUDC_WPACKTRIGGER 		0x08
//send output mixer type
#define SUDC_SENDOMTYPE 		0x0C
//send jeti key value
#define SUDC_JETIKEYVAL 		0x0D
//emergency mode (R/C link loss)
#define SUDC_EM_MODE			0x0E


extern volatile unsigned char transmitBuildInfoTrigger;
void LL_write_ctrl_data(char);
int HL2LL_write_cycle(void);
inline void SSP_rx_handler_HL(unsigned char);
inline void SSP_data_distribution_HL(void);
struct LL_ATTITUDE_DATA
{
	unsigned short system_flags;	//GPS data acknowledge, etc.

	short angle_pitch;	//angles [deg*100]
	short angle_roll;
	unsigned short angle_yaw;

	short angvel_pitch;	//angular velocities; bias-free [0.015°/s]
	short angvel_roll;
	short angvel_yaw;

				//<-- 14 bytes @ 1kHz
				//--> 3x 26 bytes @ 333 Hz
				//=> total = 40 bytes @ 1 kHz
//-----------------------------PAGE0
	unsigned char RC_data[10];	//8 channels @ 10 bit

	int latitude_best_estimate;	//GPS data fused with all other sensors
	int longitude_best_estimate;
	short acc_x;		//accelerations [mg]
	short acc_y;
	short acc_z;

	unsigned short temp_gyro;
//-----------------------------PAGE1
	unsigned char motor_data[16];	//PWM 0..7, speed 8..15

	short speed_x_best_estimate;
	short speed_y_best_estimate;
	int height;		//height [mm]
	short dheight;		//differentiated height[mm/s]
//------------------------------PAGE2
	short mag_x;
	short mag_y;
	short mag_z;

	short cam_angle_pitch;
	short cam_angle_roll;
	short cam_status;

	short battery_voltage1;
	short battery_voltage2;
	short flightMode;
	short slowDataUpChannelDataShort; //former flight_time
	short cpu_load;
	short status;
	short status2; //Bits 7..1: slowDataUpChannelSelect (7bit) Bit0:flying Bit15..8:active Motors

};

extern struct LL_ATTITUDE_DATA LL_1khz_attitude_data;

struct LL_CONTROL_INPUT
{
	unsigned short system_flags;
							//bit 0: page_select
							//bit 1: reserved (page_select)
							//bit 2: SSP_ack
							//bit 3: GPS new
							//bit 4: HL controller enabled
							//bit 5: 0 -> "scientific" commands
							//       1 -> direct motor commands
							//bit 6: waypoint mode

	unsigned short ctrl_flags;
							//bit 0..3:
							// pitch, roll, yaw, thrust enable bits
							//bit 4: height control enabled
							//bit 5: GPS_control enabled
							//bit 8..15: waypoint command if waypoint mode is active or POI options if POI mode is active or motor invert bits in individual direct motor command mode

	short pitch, roll, yaw, thrust; 		//"scientific interface"
	unsigned char direct_motor_control[8];		//direct motor commands: pitch, roll, yaw, throttle, 4xDNC
							//or motor 0..7 (Falcon)

							//<-- 20 bytes @ 1kHz
							//--> 2x18 bytes @ 500 Hz
							//=> total = 38 bytes @ 1kHz

	int latitude;					//data received from GPS-unit
	int longitude;
	int height;
	short speed_x;
	short speed_y;
	short status;
//-----------------------------

	unsigned short hor_accuracy;
	unsigned short vert_accuracy;
	unsigned short speed_accuracy;
	unsigned short numSV;
	unsigned short heading;
	short battery_voltage_1, battery_voltage_2; 	//battery voltage read by HL-ADC [mV]
	unsigned char slowDataChannelSelect; // these three vars define a slow data transfer channel. the select byte defines which data is in the data channel
	unsigned char slowDataChannelDataChar;
	short slowDataChannelDataShort;
};
extern struct LL_CONTROL_INPUT LL_1khz_control_input;



#endif /*LL_HL_COMM_*/
