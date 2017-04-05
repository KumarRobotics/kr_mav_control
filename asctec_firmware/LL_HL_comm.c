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

#include "main.h"
#include "LL_HL_comm.h"
#include "system.h"
#include "gpsmath.h"
#include "sdk.h"
#include "declination.h"
#include "jetiTelemetry.h"
#include "sdk_telemetry.h"
#include "buildInfoSetup.h"
#include <string.h>

unsigned short SSP_ack = 0;
extern char SPIWRData[128];
extern char data_sent_to_HL;
extern char data_sent_to_LL;
extern unsigned int SPIWR_num_bytes;

struct LL_ATTITUDE_DATA LL_1khz_attitude_data;
struct LL_CONTROL_INPUT LL_1khz_control_input;

unsigned char wpCtrlWpCmd = 0;
unsigned char wpCtrlWpCmdUpdated = 0;

unsigned char wpCtrlAckTrigger = 0;

unsigned short wpCtrlNavStatus = 0;
unsigned short wpCtrlDistToWp = 0;

struct WAYPOINT wpToLL;

volatile unsigned char transmitBuildInfoTrigger = 0;

void SSP_data_distribution_HL(void) {
	unsigned char i;
	unsigned char current_page = LL_1khz_attitude_data.system_flags & 0x03;
	static unsigned char oldKey = 0;

	if (LL_1khz_attitude_data.system_flags & SF_GPS_NEW)
		gpsDataOkTrigger = 0;

	IMU_CalcData.angle_nick = LL_1khz_attitude_data.angle_pitch * 10;
	IMU_CalcData.angle_roll = LL_1khz_attitude_data.angle_roll * 10;
	IMU_CalcData.angle_yaw = LL_1khz_attitude_data.angle_yaw * 10;

	IMU_CalcData.angvel_nick = LL_1khz_attitude_data.angvel_pitch;
	IMU_CalcData.angvel_roll = LL_1khz_attitude_data.angvel_roll;
	IMU_CalcData.angvel_yaw = LL_1khz_attitude_data.angvel_yaw;

	RO_ALL_Data.angle_pitch = IMU_CalcData.angle_nick;
	RO_ALL_Data.angle_roll = IMU_CalcData.angle_roll;
	RO_ALL_Data.angle_yaw = IMU_CalcData.angle_yaw;

	RO_ALL_Data.angvel_pitch = LL_1khz_attitude_data.angvel_pitch;
	RO_ALL_Data.angvel_roll = LL_1khz_attitude_data.angvel_roll;
	RO_ALL_Data.angvel_yaw = LL_1khz_attitude_data.angvel_yaw;

	if (!current_page) //page 0
	{
		for (i = 0; i < 8; i++) {
			RO_RC_Data.channel[i] = LL_1khz_attitude_data.RC_data[i] * 16;
			RO_ALL_Data.channel[i] = RO_RC_Data.channel[i];
		}
		IMU_CalcData.acc_x_calib = LL_1khz_attitude_data.acc_x * 10;
		IMU_CalcData.acc_y_calib = LL_1khz_attitude_data.acc_y * 10;
		IMU_CalcData.acc_z_calib = LL_1khz_attitude_data.acc_z * 10;

		//system is initialized as soon as values differ from 0
		if (IMU_CalcData.acc_z_calib && (SYSTEM_initialized < 10))
			SYSTEM_initialized++;

		RO_ALL_Data.acc_x = IMU_CalcData.acc_x_calib;
		RO_ALL_Data.acc_y = IMU_CalcData.acc_y_calib;
		RO_ALL_Data.acc_z = IMU_CalcData.acc_z_calib;

		RO_ALL_Data.fusion_latitude
				= LL_1khz_attitude_data.latitude_best_estimate;
		RO_ALL_Data.fusion_longitude
				= LL_1khz_attitude_data.longitude_best_estimate;

	} else if (current_page == 1) //page 1
	{
		IMU_CalcData.height = LL_1khz_attitude_data.height;
		IMU_CalcData.dheight = LL_1khz_attitude_data.dheight;

		RO_ALL_Data.fusion_height = LL_1khz_attitude_data.height;
		RO_ALL_Data.fusion_dheight = LL_1khz_attitude_data.dheight;

		RO_ALL_Data.fusion_speed_x
				= LL_1khz_attitude_data.speed_x_best_estimate;
		RO_ALL_Data.fusion_speed_y
				= LL_1khz_attitude_data.speed_y_best_estimate;
		for (i = 0; i < 6; i++) {
			RO_ALL_Data.motor_rpm[i] = LL_1khz_attitude_data.motor_data[i + 8];
		}
	} else if (current_page == 2) {
		IMU_CalcData.Hx = LL_1khz_attitude_data.mag_x;
		IMU_CalcData.Hy = LL_1khz_attitude_data.mag_y;
		IMU_CalcData.Hz = LL_1khz_attitude_data.mag_z;

		RO_ALL_Data.Hx = LL_1khz_attitude_data.mag_x;
		RO_ALL_Data.Hy = LL_1khz_attitude_data.mag_y;
		RO_ALL_Data.Hz = LL_1khz_attitude_data.mag_z;

		RO_ALL_Data.UAV_status = LL_1khz_attitude_data.flightMode;
		RO_ALL_Data.battery_voltage = HL_Status.battery_voltage_1;
		RO_ALL_Data.flight_time = HL_Status.flight_time;
		RO_ALL_Data.HL_cpu_load = HL_Status.cpu_load;
		RO_ALL_Data.HL_up_time = HL_Status.up_time;

		if (LL_1khz_attitude_data.status2 & 0x01)
			RO_ALL_Data.flying = 1;
		else
			RO_ALL_Data.flying = 0;

		unsigned char slowDataUpChannelSelect = (LL_1khz_attitude_data.status2
				>> 1) & 0x7F;
		switch (slowDataUpChannelSelect) {
		case SUDC_FLIGHTTIME:

			HL_Status.flight_time
					= LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;
		case SUDC_NAVSTATUS:
			wpCtrlNavStatus = LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;
		case SUDC_DISTTOWP:
			wpCtrlDistToWp = LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;
		case SUDC_WPACKTRIGGER:
			wpCtrlAckTrigger = LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;
		case SUDC_SENDOMTYPE:
			if ((LL_1khz_attitude_data.slowDataUpChannelDataShort == OM_HEX)
					&& (fireflyLedEnabled == 0))
				fireflyLedEnabled = 1;
			break;
		case SUDC_JETIKEYVAL:
			if (oldKey != LL_1khz_attitude_data.slowDataUpChannelDataShort)
				jetiSetKeyChanged(
						LL_1khz_attitude_data.slowDataUpChannelDataShort);
			oldKey = LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;
		case SUDC_EM_MODE:
			if (emergencyModeUpdate == 2) {
				if (LL_1khz_attitude_data.slowDataUpChannelDataShort
						!= emergencyMode)
					emergencyModeUpdate = 1;
				else
					emergencyModeUpdate = 0;
			} else
				emergencyMode
						= LL_1khz_attitude_data.slowDataUpChannelDataShort;
			break;

		}
	}
}

int HL2LL_write_cycle(void) //write data to low-level processor
{
	static char pageselect = 0;
	unsigned char i;
	static unsigned char jetiValuePartialSyncPending = 0;
	static unsigned char * transmitPtr;
	static unsigned short transmitCnt = 0;

	if (!data_sent_to_LL)
		return (0);

	//update 1kHz data
	LL_1khz_control_input.system_flags = 0 | pageselect;
	//SSP_ack=0;	//reset ack

	if (gpsDataOkTrigger)
		LL_1khz_control_input.system_flags |= SF_GPS_NEW;

	if (WO_SDK.ctrl_enabled)
		LL_1khz_control_input.system_flags |= SF_HL_CONTROL_ENABLED
				| SF_NEW_SDK;
	else
		LL_1khz_control_input.system_flags &= ~(SF_HL_CONTROL_ENABLED
				| SF_NEW_SDK);

	if (WO_SDK.disable_motor_onoff_by_stick)
		LL_1khz_control_input.system_flags
				|= SF_SDK_DISABLE_MOTORONOFF_BY_STICK;
	else
		LL_1khz_control_input.system_flags
				&= ~SF_SDK_DISABLE_MOTORONOFF_BY_STICK;

	if (WO_SDK.ctrl_mode == 0x00) //direct individual motor control
	{
		LL_1khz_control_input.system_flags
				|= SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL;
		for (i = 0; i < 8; i++) {
			LL_1khz_control_input.direct_motor_control[i]
					= WO_Direct_Individual_Motor_Control.motor[i];
		LL_1khz_control_input.ctrl_flags=WO_Direct_Individual_Motor_Control.motorReverseMask<<8;
		}
	} else if (WO_SDK.ctrl_mode == 0x01) //direct motor control with standard output mapping
	{
		LL_1khz_control_input.system_flags |= SF_DIRECT_MOTOR_CONTROL;
		LL_1khz_control_input.direct_motor_control[0]
				= WO_Direct_Motor_Control.pitch;
		LL_1khz_control_input.direct_motor_control[1]
				= WO_Direct_Motor_Control.roll;
		LL_1khz_control_input.direct_motor_control[2]
				= WO_Direct_Motor_Control.yaw;
		LL_1khz_control_input.direct_motor_control[3]
				= WO_Direct_Motor_Control.thrust;
	} else if (WO_SDK.ctrl_mode == 0x02) //attitude control
	{
		LL_1khz_control_input.system_flags &= ~(SF_DIRECT_MOTOR_CONTROL
				| SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE); //no additional system flag => attitude control is "standard mode"
		LL_1khz_control_input.ctrl_flags = WO_CTRL_Input.ctrl;
		LL_1khz_control_input.pitch = WO_CTRL_Input.pitch;
		LL_1khz_control_input.roll = WO_CTRL_Input.roll;
		LL_1khz_control_input.yaw = WO_CTRL_Input.yaw;
		LL_1khz_control_input.thrust = WO_CTRL_Input.thrust;
	} else if (WO_SDK.ctrl_mode == 0x03) //gps waypoint control
	{
		LL_1khz_control_input.system_flags |= SF_WAYPOINT_MODE;

		//check if new command should be send

		if (wpCtrlWpCmdUpdated) {

			if (wpCtrlWpCmd == WP_CMD_SINGLE_WP) {
				if (wpCtrlWpCmdUpdated == 1) {
					LL_1khz_control_input.ctrl_flags &= 0x00FF;
					LL_1khz_control_input.ctrl_flags |= WP_CMD_SINGLE_WP_PART1
							<< 8;

					LL_1khz_control_input.pitch = wpToLL.X & 0xFFFF;
					LL_1khz_control_input.roll = wpToLL.X >> 16;
					LL_1khz_control_input.thrust = wpToLL.Y & 0xFFFF;
					LL_1khz_control_input.yaw = wpToLL.Y >> 16;
					LL_1khz_control_input.direct_motor_control[0]
							= wpToLL.height;
					LL_1khz_control_input.direct_motor_control[1]
							= wpToLL.height >> 8;
					LL_1khz_control_input.direct_motor_control[2]
							= wpToLL.height >> 16;
					LL_1khz_control_input.direct_motor_control[3]
							= wpToLL.height >> 24;
					LL_1khz_control_input.direct_motor_control[4] = wpToLL.yaw;
					LL_1khz_control_input.direct_motor_control[5] = wpToLL.yaw
							>> 8;
					LL_1khz_control_input.direct_motor_control[6] = wpToLL.yaw
							>> 16;
					LL_1khz_control_input.direct_motor_control[7] = wpToLL.yaw
							>> 24;

					wpCtrlWpCmdUpdated++;
				} else if (wpCtrlWpCmdUpdated == 2) {
					LL_1khz_control_input.ctrl_flags &= 0x00FF;
					LL_1khz_control_input.ctrl_flags |= WP_CMD_SINGLE_WP_PART2
							<< 8;

					LL_1khz_control_input.pitch = wpToLL.time;
					LL_1khz_control_input.roll = 0; //wpToLL.cam_angle_pitch;
					LL_1khz_control_input.thrust = wpToLL.pos_acc;
					LL_1khz_control_input.yaw = wpToLL.chksum;
					LL_1khz_control_input.direct_motor_control[0] = 0; //wpToLL.cam_angle_roll;
					LL_1khz_control_input.direct_motor_control[1]
							= wpToLL.max_speed;
					LL_1khz_control_input.direct_motor_control[2]
							= wpToLL.properties;
					LL_1khz_control_input.direct_motor_control[3]
							= wpToLL.wp_activated;
					LL_1khz_control_input.direct_motor_control[4] = 0;
					LL_1khz_control_input.direct_motor_control[5] = 0;
					LL_1khz_control_input.direct_motor_control[6] = 0;
					LL_1khz_control_input.direct_motor_control[7] = 0;
					wpCtrlWpCmdUpdated = 0;
					wpCtrlNavStatus = 0;
				}
			} else {
				LL_1khz_control_input.ctrl_flags &= 0x00FF;
				LL_1khz_control_input.ctrl_flags |= wpCtrlWpCmd << 8;
				wpCtrlWpCmdUpdated = 0;
			}
		} else {
			LL_1khz_control_input.ctrl_flags &= 0x00FF;
		}
	} else
		LL_1khz_control_input.system_flags &= ~(SF_DIRECT_MOTOR_CONTROL
				| SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE);

	if (pageselect == 0) {
		if (LL_1khz_control_input.system_flags & SF_GPS_NEW) {
			//fill struct with 500Hz data
			LL_1khz_control_input.latitude = GPS_Data.latitude;
			LL_1khz_control_input.longitude = GPS_Data.longitude;
			LL_1khz_control_input.height = GPS_Data.height;
			LL_1khz_control_input.speed_x = GPS_Data.speed_x;
			LL_1khz_control_input.speed_y = GPS_Data.speed_y;
			LL_1khz_control_input.heading = GPS_Data.heading;
			LL_1khz_control_input.status = GPS_Data.status;
		} else {
			//default is no command
			LL_1khz_control_input.status = 0;

			if (LL_1khz_control_input.system_flags & SF_GPS_NEW) {
				//fill struct with 500Hz data
				LL_1khz_control_input.latitude = GPS_Data.latitude;
				LL_1khz_control_input.longitude = GPS_Data.longitude;
				LL_1khz_control_input.height = GPS_Data.height;
				LL_1khz_control_input.speed_x = GPS_Data.speed_x;
				LL_1khz_control_input.speed_y = GPS_Data.speed_y;
				LL_1khz_control_input.heading = GPS_Data.heading;
				LL_1khz_control_input.status = GPS_Data.status;
			} else {
				static unsigned char jetiSyncState = 0;
				static unsigned char jetiSensorCnt = 0;
				static unsigned char jetiSensorValCnt = 0;
				static unsigned char jetiSensorValUpdateTimeout = 0;

				//default is no command
				LL_1khz_control_input.status = 0;

				//data fields are used for jeti commands
				if (jetiValuePartialSyncPending == 1) {
					jetiValuePartialSyncPending = 0;
					LL_1khz_control_input.status = PD_JETI_SETSENSOR2;

					LL_1khz_control_input.height
							= jetiValues[jetiSensorCnt].active;
					memcpy(&LL_1khz_control_input.latitude,
							&jetiValues[jetiSensorCnt].unit[0], 4);
					LL_1khz_control_input.speed_x
							= jetiValues[jetiSensorCnt].unit[4];
					LL_1khz_control_input.speed_y
							= jetiValues[jetiSensorCnt].varType;
					LL_1khz_control_input.longitude
							= jetiValues[jetiSensorCnt].value;
					jetiSensorCnt++;
					if (jetiSensorCnt == 15) {
						jetiSensorCnt = 0;
					}
					jetiSyncState++;
					jetiSensorValCnt = 0;
					jetiSensorValUpdateTimeout = 0;

				} else {
					switch (jetiSyncState) {
					case 0:
						//sync name
						LL_1khz_control_input.status = PD_JETI_SETNAME;
						memcpy(&LL_1khz_control_input.latitude, &jetiName[0], 4);
						memcpy(&LL_1khz_control_input.longitude, &jetiName[4],
								4);
						memcpy(&LL_1khz_control_input.speed_x, &jetiName[8], 2);

						jetiSyncState++;
						break;
					case 1:
						//sync alarm
						LL_1khz_control_input.status = PD_JETI_SETALARM;
						LL_1khz_control_input.speed_x = jetiAlarm;
						LL_1khz_control_input.speed_y = jetiAlarmType;
						jetiSyncState++;
						break;
					case 2:
						//sync complete sensor fields
						if (jetiValues[jetiSensorCnt].active) {
							LL_1khz_control_input.status = PD_JETI_SETSENSOR;
							LL_1khz_control_input.height
									= jetiValues[jetiSensorCnt].active;
							memcpy(&LL_1khz_control_input.latitude,
									&jetiValues[jetiSensorCnt].name[0], 4);
							memcpy(&LL_1khz_control_input.longitude,
									&jetiValues[jetiSensorCnt].name[4], 4);
							memcpy(&LL_1khz_control_input.speed_x,
									&jetiValues[jetiSensorCnt].name[8], 2);
							LL_1khz_control_input.speed_y
									= jetiValues[jetiSensorCnt].decPoint;
							jetiValuePartialSyncPending = 1;
						} else {
							jetiSensorCnt++;
							if (jetiSensorCnt == 15) {
								jetiSensorCnt = 0;
							}

							jetiSyncState++;
							jetiSensorValCnt = 0;
							jetiSensorValUpdateTimeout = 0;
						}
						break;

					case 3:
						//sync text
						LL_1khz_control_input.status = PD_JETI_SETTEXT;
						memcpy(&LL_1khz_control_input.latitude,
								&jetiDisplayText[0], 4);
						memcpy(&LL_1khz_control_input.longitude,
								&jetiDisplayText[4], 4);
						memcpy(&LL_1khz_control_input.height,
								&jetiDisplayText[8], 4);
						memcpy(&LL_1khz_control_input.speed_x,
								&jetiDisplayText[12], 2);
						memcpy(&LL_1khz_control_input.speed_y,
								&jetiDisplayText[14], 2);
						jetiSyncState++;
						break;

					case 4:
						//sync text2
						LL_1khz_control_input.status = PD_JETI_SETTEXT2;

						memcpy(&LL_1khz_control_input.latitude,
								&jetiDisplayText[16], 4);
						memcpy(&LL_1khz_control_input.longitude,
								&jetiDisplayText[20], 4);
						memcpy(&LL_1khz_control_input.height,
								&jetiDisplayText[24], 4);
						memcpy(&LL_1khz_control_input.speed_x,
								&jetiDisplayText[28], 2);
						memcpy(&LL_1khz_control_input.speed_y,
								&jetiDisplayText[30], 2);
						jetiSyncState++;
						break;

					case 5:
						//sync all sensor values for 75ms

						LL_1khz_control_input.status = PD_JETI_UPDATESDATA;
						LL_1khz_control_input.height = jetiSensorValCnt;
						LL_1khz_control_input.latitude = jetiValues[3
								* jetiSensorValCnt].value;
						LL_1khz_control_input.longitude = jetiValues[3
								* jetiSensorValCnt + 1].value;
						memcpy(&LL_1khz_control_input.speed_y,
								(void *) (((unsigned char *) (&jetiValues[3
										* jetiSensorValCnt + 2].value)) + 2), 2);
						memcpy(&LL_1khz_control_input.speed_x, &jetiValues[3
								* jetiSensorValCnt + 2].value, 2);

						jetiSensorValCnt++;
						if (jetiSensorValCnt == 5)
							jetiSensorValCnt = 0;

						if (jetiTriggerTextSync) {
							jetiSyncState = 3; //trigger text resync
							jetiTriggerTextSync = 0;
						}

						jetiSensorValUpdateTimeout++;
						if (jetiSensorValUpdateTimeout == 75) {
							jetiSyncState = 0;
						}
						break;
					}
				}
			}
		}

		//write data
		LL_write_ctrl_data(pageselect);
		//set pageselect to other page for next cycle
		pageselect = 1;
	} else //pageselect=1
	{
		//fill struct with 500Hz data
		LL_1khz_control_input.hor_accuracy = GPS_Data.horizontal_accuracy;
		LL_1khz_control_input.vert_accuracy = GPS_Data.vertical_accuracy;
		LL_1khz_control_input.speed_accuracy = GPS_Data.speed_accuracy;
		LL_1khz_control_input.numSV = GPS_Data.numSV;
		LL_1khz_control_input.battery_voltage_1 = HL_Status.battery_voltage_1;
		LL_1khz_control_input.battery_voltage_2 = HL_Status.battery_voltage_2;
		if (declinationAvailable == 1) {
			declinationAvailable = 2;
			LL_1khz_control_input.slowDataChannelSelect = SDC_DECLINATION;
			LL_1khz_control_input.slowDataChannelDataShort
					= estimatedDeclination;

		} else if (declinationAvailable == 2) {
			declinationAvailable = 3;
			LL_1khz_control_input.slowDataChannelSelect = SDC_INCLINATION;
			LL_1khz_control_input.slowDataChannelDataShort
					= estimatedInclination;

		} else if (transmitBuildInfoTrigger == 1) {
			transmitBuildInfoTrigger = 2;
			transmitCnt = 0;
			transmitPtr = (unsigned char *) &buildInfo.version_major;
			LL_1khz_control_input.slowDataChannelSelect = SDC_BUILDINFO;
			LL_1khz_control_input.slowDataChannelDataShort = transmitCnt;
			LL_1khz_control_input.slowDataChannelDataChar = *transmitPtr;
			transmitPtr++;
			transmitCnt++;

		} else if (transmitBuildInfoTrigger == 2) {
			LL_1khz_control_input.slowDataChannelSelect = SDC_BUILDINFO;
			LL_1khz_control_input.slowDataChannelDataShort = transmitCnt;
			LL_1khz_control_input.slowDataChannelDataChar = *transmitPtr;
			transmitPtr++;
			transmitCnt++;

			if (transmitCnt == sizeof(buildInfo))
				transmitBuildInfoTrigger = 3;
		} else if (emergencyModeUpdate) {
			LL_1khz_control_input.slowDataChannelSelect = SDC_EM_MODE;
			LL_1khz_control_input.slowDataChannelDataShort = emergencyMode;
			//verify mode
			emergencyModeUpdate = 2;
		} else {
			LL_1khz_control_input.slowDataChannelDataShort = 0;
			LL_1khz_control_input.slowDataChannelSelect = 0;
			LL_1khz_control_input.slowDataChannelDataChar = 0;
		}

		//write data
		LL_write_ctrl_data(pageselect);
		//set pageselect to other page for next cycle
		pageselect = 0;
	}
	return (1);
}

void LL_write_ctrl_data(char page) {
	unsigned int i;
	unsigned char *dataptr;
	static volatile short spi_chksum;

	dataptr = (unsigned char *) &LL_1khz_control_input;

	//initialize syncbytes
	SPIWRData[0] = '>';
	SPIWRData[1] = '*';

	spi_chksum = 0xAAAA;

	if (!page) {
		for (i = 2; i < 40; i++) {
			SPIWRData[i] = *dataptr++;
			spi_chksum += SPIWRData[i];
		}
	} else {
		for (i = 2; i < 22; i++) {
			SPIWRData[i] = *dataptr++;
			spi_chksum += SPIWRData[i];
		}
		dataptr += 18;
		for (i = 22; i < 40; i++) {
			SPIWRData[i] = *dataptr++;
			spi_chksum += SPIWRData[i];
		}
	}

	SPIWRData[40] = spi_chksum; //chksum LSB
	SPIWRData[41] = (spi_chksum >> 8); //chksum MSB

	SPIWR_num_bytes = 42;
	data_sent_to_LL = 0;
}

inline void SSP_rx_handler_HL(unsigned char SPI_rxdata) //rx_handler @ high-level processor
{
	static volatile unsigned char SPI_syncstate = 0;
	static volatile unsigned char SPI_rxcount = 0;
	static volatile unsigned char *SPI_rxptr;
	static volatile unsigned char incoming_page;

	//receive handler
	if (SPI_syncstate == 0) {
		if (SPI_rxdata == '>')
			SPI_syncstate++;
		else
			SPI_syncstate = 0;
	} else if (SPI_syncstate == 1) {
		if (SPI_rxdata == '*') {
			SPI_syncstate++;
			SPI_rxptr = (unsigned char *) &LL_1khz_attitude_data;
			SPI_rxcount = 40;
		} else
			SPI_syncstate = 0;
	} else if (SPI_syncstate == 2) {
		if (SPI_rxcount == 26) //14 bytes transmitted => select 500Hz page
		{
			incoming_page = LL_1khz_attitude_data.system_flags & 0x03; //system flags were already received
			if (incoming_page == 1)
				SPI_rxptr += 26;
			else if (incoming_page == 2)
				SPI_rxptr += 52;
		}
		SPI_rxcount--;
		*SPI_rxptr = SPI_rxdata;
		SPI_rxptr++;
		if (SPI_rxcount == 0) {
			SPI_syncstate++;
		}
	} else if (SPI_syncstate == 3) {
		if (SPI_rxdata == '<') //last byte ok => data should be valid
		{
			SSP_data_distribution_HL(); //only distribute data to other structs, if it was received correctly
			//ack data receiption
		}
		SPI_syncstate = 0;
	} else
		SPI_syncstate = 0;
}

