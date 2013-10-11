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

#include "main.h"
#include "sdk.h"
#include "LL_HL_comm.h"
#include "gpsmath.h"

#if 1
struct CTRL_INPUT Ctrl_Input, Ctrl_Input_tmp;
struct OUTPUT_DATA Output_Data, Output_Data_tmp;
#endif

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct RO_ALL_DATA RO_ALL_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;
struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_Direct_Individual_Motor_Control;

void SDK_EXAMPLE_direct_individual_motor_commands(void);
void SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping(void);
void SDK_EXAMPLE_attitude_commands(void);
void SDK_EXAMPLE_gps_waypoint_control(void);
int SDK_EXAMPLE_turn_motors_on(void);
int SDK_EXAMPLE_turn_motors_off(void);

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

/* How to flash the high level processor
 *
 * The easiest way to get your code on the high level processor is to use the JTAG-adapter.
 *
 * It needs three steps to get your code on the high level processor.
 * 1. Build your code ("Build Project")
 * 2. Connect your JTAG adapter and start the JTAG connection ("OpenOCD Asctec-JTAG")
 * 3. Flash the processor ("Asctec JTAG Debug")
 *
 * In the menu "Run -> External Tools -> External Tools Configuration..." you
 * will find "OpenOCD Asctec-JTAG". If the JTAG connection was activated
 * correctly, the console will show only the following line:
 * "Info:    openocd.c:92 main(): Open On-Chip Debugger (2007-08-10 22:30 CEST)"
 *
 * Do not launch more than ONE JTAG-connection at the same time!
 *
 * In the menu "Run -> Debug Configurations..." you will find "Asctec JTAG Debug"
 * If the code was successfully flashed on the processor, the program will switch
 * to the Debug window.
 *************
 * If you want to flash the high level processor using a serial interface (AscTec USB adapter)
 * and bootloader software like "Flash Magic", you need to change
 * the following in the "makefile" (line 113):
 *
 * FORMAT = ihex
 * #FORMAT = binary
 *
 * After buidling your code you will find the main.hex in your workspace folder.
 *************
 * */

/* After flashing the HL, your code can be debugged online. The ARM7 supports ONE hardware breakpoint.
 * You can monitor the CPU-load by looking at HL_Status.cpu_load. As long as this value is 1000 your
 * code in SDK_mainloop() is executed at 1 kHz.
 *
 * To activate the SDK controls, the serial interface switch on your R/C (channel 5) needs to be in ON position.
 *
 * If your project needs communication over a serial link, you find an example of how to do so in main.c, line 226.
 * All sample functions for transmitting and receiving data over the UART (HL_serial_0) are in uart.c - you can use
 * these examples to code your own communication state machine.
 */


void SDK_mainloop(void) //write your own code within this function
{


	//you can select an example by using ONE of the functions below.
	//CAUTION! Read the code of the examples before you test them on your UAV!

	//SDK_EXAMPLE_direct_individual_motor_commands();
	//SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping();
	//SDK_EXAMPLE_attitude_commands();
	//SDK_EXAMPLE_gps_waypoint_control();

	//example to turn motors on and off every 2 seconds
	
 	static int timer=0;
	if(++timer<1000) SDK_EXAMPLE_turn_motors_on();
	else if(timer<2000) SDK_EXAMPLE_turn_motors_on();
	else timer=0;
	
}

/* the following example shows the direct motor command usage by mapping the stick directly to the motor outputs (do NOT try to fly ;-) )
 *
 */
void SDK_EXAMPLE_direct_individual_motor_commands(void)
{

	WO_SDK.ctrl_mode=0x00;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0;

	unsigned int i;

	//scale throttle stick to [0..200] and map it to all motors
	WO_Direct_Individual_Motor_Control.motor[0]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[1]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[2]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[3]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[4]=RO_ALL_Data.channel[2]/21;
	WO_Direct_Individual_Motor_Control.motor[5]=RO_ALL_Data.channel[2]/21;

	//make sure commands are never 0 so that motors will always keep spinning
    //also make sure that commands stay within range
    for(i=0;i<6;i++)
    {
    	if(WO_Direct_Individual_Motor_Control.motor[i] == 0)
        WO_Direct_Individual_Motor_Control.motor[i] = 0;
    	else if(WO_Direct_Individual_Motor_Control.motor[i] > 200)
        WO_Direct_Individual_Motor_Control.motor[i] = 200;
    }
}


void SDK_EXAMPLE_direct_motor_commands_with_standard_output_mapping(void)
{
	WO_SDK.ctrl_mode=0x01;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	/*
	 *  Stick commands directly mapped to motors, NO attitude control! Do NOT try to fly!
	 * */

	WO_Direct_Motor_Control.pitch=(4095-RO_ALL_Data.channel[0])/21;
	WO_Direct_Motor_Control.roll=RO_ALL_Data.channel[1]/21;
	WO_Direct_Motor_Control.thrust=RO_ALL_Data.channel[2]/21;
	WO_Direct_Motor_Control.yaw=(4095-RO_ALL_Data.channel[3])/21;

}


void SDK_EXAMPLE_attitude_commands(void)
{
	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	//with this example the UAV will go to ~10% throttle when SDK control is activated
	WO_CTRL_Input.ctrl=0x08;	//0x08: enable throttle control by HL. Height control and GPS are deactivated!!
								//pitch, roll and yaw are still commanded via the remote control

	WO_CTRL_Input.thrust=400;	//10% throttle command


}



/* This function demonstrates a simple waypoint command generation. Switch on Channel 7 is used
 * to activate a 15m by 15m square. Therefore a waypoint is calculated from the current position and
 * height and is transmitted to the low level processor. The waypoint status is monitored to switch to
 * the next waypoint after the current one is reached.
 *
 * wpCtrlWpCmd is used to send a command to the low level processor. Different options like waypoint, launch, land, come home, set home
 * are available. See LL_HL_comm.h for WP_CMD_* defines
 *
 * wpCtrlWpCmdUpdated has to be set to 1 to send the command. When the cmd is sent it is set back to 0 automatically
 *
 * wpCtrlAckTrigger is set to 1 when the LL accepts the waypoint
 *
 * wpCtrlNavStatus gives you a navigation status. See WP_NAVSTAT_* defines in SDK.h for options
 *
 * wpCtrlDistToWp gives you the current distance to the current waypoint in dm (= 10 cm)
 */
void SDK_EXAMPLE_gps_waypoint_control()
{
	static uint8_t wpExampleState=0;
	static double originLat,originLon;


	WO_SDK.ctrl_mode=0x03;

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	switch (wpExampleState)
	{
		//prior to start, the lever on channel 7 has to be in "OFF" position
		case 0:
		if (RO_RC_Data.channel[6]<1600)
			wpExampleState=1;
		break;

		case 1:
		if (RO_RC_Data.channel[6]>2400)
		{
			double lat,lon;
			//lever was set to "ON" state -> calculate and send first waypoint and switch state

			//fill waypoint structure
			wpToLL.max_speed=100;
			wpToLL.pos_acc=3000; 	//3m accuracy
			wpToLL.time=400; 		//4 seconds waiting time at each waypoint
			wpToLL.wp_activated=1;

			//see LL_HL_comm.h for WPPROP defines
			wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

			//use current height and yaw
			wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
			wpToLL.height=IMU_CalcData.height; //use current height

			originLat=(double)GPS_Data.latitude/10000000.0;
			originLon=(double)GPS_Data.longitude/10000000.0;

			//calculate a position 15m north of us
			xy2latlon(originLat,originLon,0.0,15.0,&lat,&lon);

			wpToLL.X=lon*10000000;
			wpToLL.Y=lat*10000000;

			//calc chksum
			wpToLL.chksum = 0xAAAA
									+ wpToLL.yaw
									+ wpToLL.height
									+ wpToLL.time
									+ wpToLL.X
									+ wpToLL.Y
									+ wpToLL.max_speed
									+ wpToLL.pos_acc
									+ wpToLL.properties
									+ wpToLL.wp_activated;

			//send waypoint
			wpCtrlAckTrigger=0;
			wpCtrlWpCmd=WP_CMD_SINGLE_WP;
			wpCtrlWpCmdUpdated=1;

			wpExampleState=2;

		}
		break;

		case 2:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{
					//new waypoint
					double lat,lon;

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

					//calculate a position 15m north and 15m east of origin
					xy2latlon(originLat,originLon,15.0,15.0,&lat,&lon);

					wpToLL.X=lon*10000000;
					wpToLL.Y=lat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;
					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;

					wpExampleState=3;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
						wpExampleState=0;
		break;

		case 3:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{
					//new waypoint
					double lat,lon;

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

					//calculate a position 15m east of origin
					xy2latlon(originLat,originLon,15.0,0.0,&lat,&lon);

					wpToLL.X=lon*10000000;
					wpToLL.Y=lat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;

					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;

					wpExampleState=4;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
						wpExampleState=0;
		break;

		case 4:
			//wait until cmd is processed and sent to LL processor
			if ((wpCtrlWpCmdUpdated==0) && (wpCtrlAckTrigger))
			{
				//check if waypoint was reached and wait time is over
				if (wpCtrlNavStatus&(WP_NAVSTAT_REACHED_POS_TIME))
				{

					//fill waypoint structure
					wpToLL.max_speed=100;
					wpToLL.pos_acc=3000; //3m accuracy
					wpToLL.time=400; //4 seconds wait time
					wpToLL.wp_activated=1;

					//see LL_HL_comm.h for WPPROP defines
					wpToLL.properties=WPPROP_ABSCOORDS|WPPROP_AUTOMATICGOTO|WPPROP_HEIGHTENABLED|WPPROP_YAWENABLED;

					//use current height and yaw
					wpToLL.yaw=IMU_CalcData.angle_yaw; //use current yaw
					wpToLL.height=IMU_CalcData.height; //use current height

					//go to the beginning

					wpToLL.X=originLon*10000000;
					wpToLL.Y=originLat*10000000;

					//calc chksum
					wpToLL.chksum = 0xAAAA
											+ wpToLL.yaw
											+ wpToLL.height
											+ wpToLL.time
											+ wpToLL.X
											+ wpToLL.Y
											+ wpToLL.max_speed
											+ wpToLL.pos_acc
											+ wpToLL.properties
											+ wpToLL.wp_activated;

					//send waypoint
					wpCtrlAckTrigger=0;
					wpCtrlWpCmd=WP_CMD_SINGLE_WP;
					wpCtrlWpCmdUpdated=1;

					wpExampleState=0;
				}

				if (wpCtrlNavStatus&WP_NAVSTAT_PILOT_ABORT)
					wpExampleState=0;


			}
			if (RO_RC_Data.channel[6]<1600)
						wpExampleState=0;
		break;

		default:
			wpExampleState=0;
		break;
	}

}

int SDK_EXAMPLE_turn_motors_on(void) //hold throttle stick down and yaw stick fully left to turn motors on
{
	static int timeout=0;

	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0; //make sure stick command is accepted

	if(++timeout>=1000)
	{
		timeout=0;
		return(1); //1 => start sequence completed => motors running => user can stop calling this function
	}
	else if(timeout>500) //neutral stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=-2047;
		return(0);
	}

}

int SDK_EXAMPLE_turn_motors_off(void) //hold throttle stick down and yaw stick fully right to turn motors off
{
	static int timeout=0;

	WO_SDK.ctrl_mode=0x02;	//0x00: direct individual motor control: individual commands for motors 0..3
							//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw and thrust inputs; no attitude controller active
							//0x02: attitude and throttle control: commands are input for standard attitude controller
							//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;  //0: disable control by HL processor
							//1: enable control by HL processor

	WO_SDK.disable_motor_onoff_by_stick=0; //make sure stick command is accepted

	if(++timeout>=1000)
	{
		timeout=0;
		return(1); //1 => stop sequence completed => motors turned off => user can stop calling this function
	}
	else if(timeout>500) //neutral stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=0;
		return(0);
	}
	else //hold stick command for 500 ms
	{
		WO_CTRL_Input.ctrl=0x0C;	//0x0C: enable throttle control and yaw control
		WO_CTRL_Input.thrust=0;	//use R/C throttle stick input /2 to control thrust (just for testing)
		WO_CTRL_Input.yaw=2047;
		return(0);
	}
}


