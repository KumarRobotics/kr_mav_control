/*

AscTec SDK 3.0

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

/**********************************************************
                  Header files
 **********************************************************/
#include "LPC214x.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "mymath.h"
#include "hardware.h"
#include "irq.h"
#include "i2c.h"
#include "i2c1.h"
#include "gpsmath.h"
#include "adc.h"
#include "uart.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "sdk.h"
#include "buzzer.h"
#include "ublox.h"
#include "pelican_ptu.h"
#include "declination.h"
#include "asctecCommIntfOnboard.h"
#include "lpc_aci_eeprom.h"

#ifdef MATLAB
#include "..\custom_mdl\onboard_matlab_ert_rtw\onboard_matlab.h"
#endif

/* *********************************************************
               Function declarations
  ********************************************************* */

void Initialize(void);
void feed(void);
void beeper(unsigned char);
void ACISDK(void);

/**********************************************************
                  Global Variables
 **********************************************************/
struct HL_STATUS HL_Status;
struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;

volatile unsigned int int_cnt=0, cnt=0, mainloop_cnt=0;
volatile unsigned char mainloop_trigger=0;
volatile unsigned int GPS_timeout=0;
volatile unsigned int trigger_cnt=0;
volatile char SYSTEM_initialized=0;

unsigned int uart_cnt;
unsigned char DataOutputsPerSecond=10;
unsigned char fireflyLedEnabled=0;
unsigned char PTU_cam_option_4_version=2;
unsigned short mainloop_overflows=0;

void timer0ISR(void) __irq
{
  T0IR = 0x01;      //Clear the timer 0 interrupt
  IENABLE;
  trigger_cnt++;
  if(trigger_cnt==ControllerCyclesPerSecond)
  {
  	trigger_cnt=0;
  	HL_Status.up_time++;
  	HL_Status.cpu_load=mainloop_cnt;

  	mainloop_cnt=0;
  }

  if(mainloop_trigger<10) mainloop_trigger++;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

/**********************************************************
                       MAIN
**********************************************************/
int	main (void) {

  static int vbat1; //battery_voltage (lowpass-filtered)
  unsigned int TimerT1, TimerT2;

  init();
  buzzer(OFF);
  LL_write_init();

  //initialize AscTec Firefly LED fin on I2C1 (not necessary on AscTec Hummingbird or Pelican)
  I2C1Init();
  I2C1_setRGBLed(255,0,0);

  ADC0triggerSampling(1<<VOLTAGE_1); //activate ADC sampling

  generateBuildInfo();

  HL_Status.up_time=0;

  LED(1,ON);

  ACISDK();	//AscTec Communication Interface: publish variables, set callbacks, etc.

  //update parameters stored by ACI:
  //...

  PTU_init();	//initialize camera PanTiltUnit
#ifdef MATLAB
  //ee_read((unsigned int*)&matlab_params); //read params from eeprom
  onboard_matlab_initialize(); //initialize matlab code
#endif

  while(1)
  {
      if(mainloop_trigger)
      {
      	TimerT1 =  T0TC;
     	if(GPS_timeout<ControllerCyclesPerSecond) GPS_timeout++;
	  	else if(GPS_timeout==ControllerCyclesPerSecond)
	  	{
  	 		GPS_timeout=ControllerCyclesPerSecond+1;
	  		GPS_Data.status=0;
	  		GPS_Data.numSV=0;
	  	}

        //battery monitoring
        ADC0getSamplingResults(0xFF,adcChannelValues);
        vbat1=(vbat1*14+(adcChannelValues[VOLTAGE_1]*9872/579))/15;	//voltage in mV

		HL_Status.battery_voltage_1=vbat1;
        mainloop_cnt++;
		if(!(mainloop_cnt%10)) buzzer_handler(HL_Status.battery_voltage_1);

	    if(mainloop_trigger) mainloop_trigger--;
        mainloop();
        // CPU Usage calculation
        TimerT2 = T0TC;
        if (mainloop_trigger)
        {
        	HL_Status.cpu_load = 1000;
        	mainloop_overflows++;
        }
        else if (TimerT2 < TimerT1)
        	HL_Status.cpu_load = (T0MR0 - TimerT1 + TimerT2)*1000/T0MR0; // load = "timer cycles" / "timer cycles per controller cycle" * 1000
        else
        	HL_Status.cpu_load = (TimerT2 - TimerT1)*1000/T0MR0; // load = "timer cycles" / "timer cycles per controller cycle" * 1000
      }

  }
  return 0;
}


void mainloop(void) //mainloop is triggered at 1 kHz
{
    static unsigned char led_cnt=0, led_state=1;
    static int Firefly_led_fin_cnt=0;
	unsigned char t;

	//blink red led if no GPS lock available
	led_cnt++;
	if((GPS_Data.status&0xFF)==0x03)
	{
		LED(0,OFF);
	}
	else
	{
	    if(led_cnt==150)
	    {
	      LED(0,ON);
	    }
	    else if(led_cnt==200)
	    {
	      led_cnt=0;
	      LED(0,OFF);
	    }
	}


	//after first lock, determine magnetic inclination and declination
	if (SYSTEM_initialized)
	{
		if ((!declinationAvailable) && (GPS_Data.horizontal_accuracy<10000) && ((GPS_Data.status&0x03)==0x03)) //make sure GPS lock is valid
		{
			int status;
			estimatedDeclination=getDeclination(GPS_Data.latitude,GPS_Data.longitude,GPS_Data.height/1000,2012,&status);
			if (estimatedDeclination<-32000) estimatedDeclination=-32000;
			if (estimatedDeclination>32000) estimatedDeclination=32000;
			declinationAvailable=1;
		}
	}

	//toggle green LED and update SDK input struct when GPS data packet is received
    if (gpsLEDTrigger)
    {
		if(led_state)
		{
			led_state=0;
			LED(1,OFF);
		}
		else
		{
			LED(1,ON);
			led_state=1;
		}

		RO_ALL_Data.GPS_height=GPS_Data.height;
		RO_ALL_Data.GPS_latitude=GPS_Data.latitude;
		RO_ALL_Data.GPS_longitude=GPS_Data.longitude;
		RO_ALL_Data.GPS_speed_x=GPS_Data.speed_x;
		RO_ALL_Data.GPS_speed_y=GPS_Data.speed_y;
		RO_ALL_Data.GPS_status=GPS_Data.status;
		RO_ALL_Data.GPS_sat_num=GPS_Data.numSV;
		RO_ALL_Data.GPS_week=GPS_Time.week;
		RO_ALL_Data.GPS_time_of_week=GPS_Time.time_of_week;
		RO_ALL_Data.GPS_heading=GPS_Data.heading;
		RO_ALL_Data.GPS_position_accuracy=GPS_Data.horizontal_accuracy;
		RO_ALL_Data.GPS_speed_accuracy=GPS_Data.speed_accuracy;
		RO_ALL_Data.GPS_height_accuracy=GPS_Data.vertical_accuracy;

		gpsLEDTrigger=0;
    }

	//re-trigger UART-transmission if it was paused by modem CTS pin
	if(trigger_transmission)
	{
		if(!(IOPIN0&(1<<CTS_RADIO)))
	  	{
	  		trigger_transmission=0;
		    if(ringbuffer(RBREAD, &t, 1))
		    {
		      transmission_running=1;
		      UARTWriteChar(t);
		    }
	  	}
	}

#ifdef MATLAB
	//re-trigger UART-transmission if it was paused by modem CTS pin
	if(trigger_transmission)
	{
		if(!(IOPIN0&(1<<CTS_RADIO)))
	  	{
	  		trigger_transmission=0;
		    if(UART_Matlab_fifo(RBREAD, &t, 1))
		    {
		      transmission_running=1;
		      UARTWriteChar(t);
		    }
	  	}
	}
#endif

	//send data packet as an example how to use HL_serial_0 (please refer to uart.c for details)
/*
    if(uart_cnt++==ControllerCyclesPerSecond/DataOutputsPerSecond)
    {
    	uart_cnt=0;
      	if((sizeof(RO_ALL_Data))<ringbuffer(RBFREE, 0, 0))
       	{
       		UART_SendPacket(&RO_ALL_Data, sizeof(RO_ALL_Data), PD_RO_ALL_DATA);
       	}
    }
*/
    //handle gps data reception
    uBloxReceiveEngine();

	//run SDK mainloop. Please put all your data handling / controller code in sdk.c
	SDK_mainloop();

    //write data to transmit buffer for immediate transfer to LL processor
    HL2LL_write_cycle();

    //control pan-tilt-unit ("cam option 4" @ AscTec Pelican and AscTec Firefly)
    PTU_update();

    //synchronize all variables, commands and parameters with ACI
    aciSyncVar();
    aciSyncCmd();
    aciSyncPar();

    //run ACI engine
    aciEngine();

    //send buildinfo
    if ((SYSTEM_initialized) && (!transmitBuildInfoTrigger))
		transmitBuildInfoTrigger=1;

    //Firefly LED
    if (SYSTEM_initialized&&fireflyLedEnabled)
    {
    	if(++Firefly_led_fin_cnt==10)
    	{
    		Firefly_led_fin_cnt=0;
    		fireFlyLedHandler();
    	}
    }

}


void ACISDK(void)
{
	aciInit(1000);
	lpc_aci_init();
#ifndef MATLAB
	aciSetStartTxCallback(UARTWriteChar);
	// Variables
	aciPublishVariable(&RO_ALL_Data.UAV_status, VARTYPE_INT16, 0x0001, "UAV_status", "UAV status information","See in wiki");
	aciPublishVariable(&RO_ALL_Data.flight_time, VARTYPE_INT16, 0x0002, "flight_time", "Total flight time","s");
	aciPublishVariable(&RO_ALL_Data.battery_voltage, VARTYPE_INT16, 0x0003, "battery_voltage", "Battery voltage","mV");
	aciPublishVariable(&RO_ALL_Data.HL_cpu_load, VARTYPE_INT16, 0x0004, "HL_cpu_load", "High-level CPU load","Hz");
	aciPublishVariable(&RO_ALL_Data.HL_up_time, VARTYPE_INT16, 0x0005, "HL_up_time", "AHigh-level up-time","ms");

	aciPublishVariable(&RO_ALL_Data.motor_rpm[0], VARTYPE_UINT8, 0x0100, "motor_rpm[0]", "Quadcopter: front, Hexcopter front-left", "RPM measurements (0..200)");
	aciPublishVariable(&RO_ALL_Data.motor_rpm[1], VARTYPE_UINT8, 0x0101, "motor_rpm[1]", "Quadcopter: rear, Hexcopter left", "RPM measurements (0..200)");
	aciPublishVariable(&RO_ALL_Data.motor_rpm[2], VARTYPE_UINT8, 0x0102, "motor_rpm[2]", "Quadcopter: left, Hexcopter rear-left", "RPM measurements (0..200)");
	aciPublishVariable(&RO_ALL_Data.motor_rpm[3], VARTYPE_UINT8, 0x0103, "motor_rpm[3]", "Quadcopter: right, Hexcopter rear-right", "RPM measurements (0..200)");
	aciPublishVariable(&RO_ALL_Data.motor_rpm[4], VARTYPE_UINT8, 0x0104, "motor_rpm[4]", "Quadcopter: N/A, Hexcopter right", "RPM measurements (0..200)");
	aciPublishVariable(&RO_ALL_Data.motor_rpm[5], VARTYPE_UINT8, 0x0105, "motor_rpm[5]", "Quadcopter: N/A, Hexcopter front-right", "RPM measurements (0..200)");

	aciPublishVariable(&RO_ALL_Data.GPS_latitude, VARTYPE_INT32, 0x0106, "GPS_latitude", "Latitude from the GPS sensor", "degrees * 10^7");
	aciPublishVariable(&RO_ALL_Data.GPS_longitude, VARTYPE_INT32, 0x0107, "GPS_longitude", "Longitude from the GPS sensor", "degrees * 10^7");
	aciPublishVariable(&RO_ALL_Data.GPS_height, VARTYPE_INT32, 0x0108, "GPS_height", "Height from the GPS sensor", "mm");
	aciPublishVariable(&RO_ALL_Data.GPS_speed_x, VARTYPE_INT32, 0x0109, "GPS_speed_x", "Speed in East/West from the GPS sensor", "mm/s");
	aciPublishVariable(&RO_ALL_Data.GPS_speed_y, VARTYPE_INT32, 0x010A, "GPS_speed_y", "Speed in North/South from the GPS sensor", "mm/s");
	aciPublishVariable(&RO_ALL_Data.GPS_heading, VARTYPE_INT32, 0x010B, "GPS_heading", "Heading from the Compass", "deg * 1000");
	aciPublishVariable(&RO_ALL_Data.GPS_position_accuracy, VARTYPE_UINT32, 0x010C, "GPS_position_accuracy", "GPS position accuracy estimate", "mm");
	aciPublishVariable(&RO_ALL_Data.GPS_height_accuracy, VARTYPE_UINT32, 0x010D, "GPS_height_accuracy", "GPS height accuracy estimate", "mm");
	aciPublishVariable(&RO_ALL_Data.GPS_speed_accuracy, VARTYPE_UINT32, 0x010E, "GPS_speed_accuracy", "GPS speed accuracy estimate", "mm/s");
	aciPublishVariable(&RO_ALL_Data.GPS_sat_num, VARTYPE_UINT32, 0x010F, "GPS_sat_num", "Number of satellites used in NAV solution", "count");
	aciPublishVariable(&RO_ALL_Data.GPS_status, VARTYPE_INT32, 0x0110, "GPS_status", "GPS status information", "see documentation");
	aciPublishVariable(&RO_ALL_Data.GPS_time_of_week, VARTYPE_UINT32, 0x0111, "GPS_time_of_week", "Time of the week (1 week = 604,800 s)", "ms");
	aciPublishVariable(&RO_ALL_Data.GPS_week, VARTYPE_UINT16, 0x0112, "GPS_week", "Week counter since 1980", "count");

	aciPublishVariable(&RO_ALL_Data.angvel_pitch, VARTYPE_INT32, 0x0200, "angvel_pitch", "Pitch angle velocity", "0.0154 degree/s, ""bias free");
	aciPublishVariable(&RO_ALL_Data.angvel_roll, VARTYPE_INT32, 0x0201, "angvel_roll", "Roll angle velocity", "0.0154 degree/s, bias free");
	aciPublishVariable(&RO_ALL_Data.angvel_yaw, VARTYPE_INT32, 0x0202, "angvel_yaw", "Yaw angle velocity", "0.0154 degree/s, bias free");

	aciPublishVariable(&RO_ALL_Data.acc_x, VARTYPE_INT16, 0x0203, "acc_x", "Acc-sensor output in x, body frame coordinate system","-10000..+10000 = -1g..+1g");
	aciPublishVariable(&RO_ALL_Data.acc_y, VARTYPE_INT16, 0x0204, "acc_y", "Acc-sensor output in y, body frame coordinate system","-10000..+10000 = -1g..+1g");
	aciPublishVariable(&RO_ALL_Data.acc_z, VARTYPE_INT16, 0x0205, "acc_z", "Acc-sensor output in z, body frame coordinate system","-10000..+10000 = -1g..+1g");

	aciPublishVariable(&RO_ALL_Data.Hx, VARTYPE_INT32, 0x0206, "Hx", "Magnetic field sensors output in x", "+-2500 =+- earth field strength");
	aciPublishVariable(&RO_ALL_Data.Hy, VARTYPE_INT32, 0x0207, "Hy", "Magnetic field sensors output in y", "+-2500 =+- earth field strength");
	aciPublishVariable(&RO_ALL_Data.Hz, VARTYPE_INT32, 0x0208, "Hz", "Magnetic field sensors output in z", "+-2500 =+- earth field strength");

	aciPublishVariable(&RO_ALL_Data.angle_pitch, VARTYPE_INT32, 0x0300, "angle_pitch", "Pitch angle derived by by data fusion", "degree*1000");
	aciPublishVariable(&RO_ALL_Data.angle_roll, VARTYPE_INT32, 0x0301, "angle_roll", "Roll angle derived by data fusion", "degree*1000");
	aciPublishVariable(&RO_ALL_Data.angle_yaw, VARTYPE_INT32, 0x0302, "angle_yaw", "Yaw angle derived by data fusion", "degree*1000");

	aciPublishVariable(&RO_ALL_Data.fusion_latitude, VARTYPE_INT32, 0x0303, "fusion_latitude", "Fused latitude with all other sensors (best estimations)", "degrees * 10^7");
	aciPublishVariable(&RO_ALL_Data.fusion_longitude, VARTYPE_INT32, 0x0304, "fusion_longitude", "Fused longitude with all other sensors (best estimations)", "degrees * 10^7");
	aciPublishVariable(&RO_ALL_Data.fusion_dheight, VARTYPE_INT32, 0x0305, "fusion_dheight", "Difference height after data fusion", "mm/s");
	aciPublishVariable(&RO_ALL_Data.fusion_height, VARTYPE_INT32, 0x0306, "fusion_height", "Height after data fusion", "mm");
	aciPublishVariable(&RO_ALL_Data.fusion_speed_x, VARTYPE_INT16, 0x0307, "fusion_speed_x", "Fused speed in East/West with all other sensors (best estimations)", "mm/s");
	aciPublishVariable(&RO_ALL_Data.fusion_speed_y, VARTYPE_INT16, 0x0308, "fusion_speed_y", "Fused speed in North/South with all other sensors (best estimations)", "mm/s");

	aciPublishVariable(&RO_ALL_Data.channel[0], VARTYPE_UINT16, 0x0600, "channel[0]", "Pitch command received from the remote control", "0..4095");
	aciPublishVariable(&RO_ALL_Data.channel[1], VARTYPE_UINT16, 0x0601, "channel[1]", "Roll command received from the remote control", "0..4095");
	aciPublishVariable(&RO_ALL_Data.channel[2], VARTYPE_UINT16, 0x0602, "channel[2]", "Thrust command received from the remote control", "0..4095");
	aciPublishVariable(&RO_ALL_Data.channel[3], VARTYPE_UINT16, 0x0603, "channel[3]", "Yaw command received from the remote control", "0..4095");
	aciPublishVariable(&RO_ALL_Data.channel[4], VARTYPE_UINT16, 0x0604, "channel[4]", "Serial interface enable/disable", ">2048 enabled, else disabled");
	aciPublishVariable(&RO_ALL_Data.channel[5], VARTYPE_UINT16, 0x0605, "channel[5]", "Manual / height control / GPS + height control", "see documentation");
	aciPublishVariable(&RO_ALL_Data.channel[6], VARTYPE_UINT16, 0x0606, "channel[6]", "Custom remote control data","n/a");
	aciPublishVariable(&RO_ALL_Data.channel[7], VARTYPE_UINT16, 0x0607, "channel[7]", "Custom remote control data","n/a");


	// Commands
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[0]), VARTYPE_UINT8, 0x0500, "DIMC motor[0]", "Direct motor control 1", "0..200 = 0..100 %");
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[1]), VARTYPE_UINT8, 0x0501, "DIMC motor[1]", "Direct motor control 2", "0..200 = 0..100 %");
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[2]), VARTYPE_UINT8, 0x0502, "DIMC motor[2]", "Direct motor control 3", "0..200 = 0..100 %");
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[3]), VARTYPE_UINT8, 0x0503, "DIMC motor[3]", "Direct motor control 4", "0..200 = 0..100 %");
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[4]), VARTYPE_UINT8, 0x0504, "DIMC motor[4]", "Direct motor control 5", "0..200 = 0..100 %");
	aciPublishCommand(&(WO_Direct_Individual_Motor_Control.motor[5]), VARTYPE_UINT8, 0x0505, "DIMC motor[5]", "Direct motor control 6", "0..200 = 0..100 %");

	aciPublishCommand(&WO_Direct_Motor_Control.pitch, VARTYPE_UINT8, 0x0506, "DMC pitch", "Pitch input (DMC)", "0..200 = - 100..+100%");
	aciPublishCommand(&WO_Direct_Motor_Control.roll, VARTYPE_UINT8, 0x0507, "DMC roll", "Roll input (DMC)", "0..200 = - 100..+100%");
	aciPublishCommand(&WO_Direct_Motor_Control.yaw, VARTYPE_UINT8, 0x0508, "DMC yaw", "Yaw input (DMC)", "0..200 = - 100..+100%");
	aciPublishCommand(&WO_Direct_Motor_Control.thrust, VARTYPE_UINT8, 0x0509, "DMC thrust", "Thrust input (DMC)", "0..200 = 0..100 %");

	aciPublishCommand(&WO_CTRL_Input.pitch, VARTYPE_INT16, 0x050A, "CRTL pitch", "Pitch input (CRTL)", "-2047..+2047 (0=neutral)");
	aciPublishCommand(&WO_CTRL_Input.roll, VARTYPE_INT16, 0x050B, "CTRL roll", "Roll input (CRTL)", "-2047..+2047 (0=neutral)");
	aciPublishCommand(&WO_CTRL_Input.yaw, VARTYPE_INT16, 0x050C, "CTRL yaw", "Yaw input (CRTL)", "-2047..+2047 (0=neutral)");
	aciPublishCommand(&WO_CTRL_Input.thrust, VARTYPE_INT16, 0x050D, "CTRL thrust", "Thrust input (CRTL)", "0..4095 = 0..100%");
	aciPublishCommand(&WO_CTRL_Input.ctrl, VARTYPE_INT16, 0x050E, "CTRL ctrl", "Control byte for enable different controls", "see documentation");

	aciPublishCommand(&WO_SDK.ctrl_mode,VARTYPE_UINT8,0x0600,"ctrl_mode","Control mode setting parameter","0:DIMC, 1: DMC, 2: CRTL, 3: GPS");
	aciPublishCommand(&WO_SDK.ctrl_enabled,VARTYPE_UINT8,0x0601,"ctrl_enabled","Control commands are accepted/ignored by LL processor", "0x00: ignored, 0x01: accepted");
	aciPublishCommand(&WO_SDK.disable_motor_onoff_by_stick,VARTYPE_UINT8,0x0602,"disable_motor_onoff_by_stick","Setting if motors can be turned on by using the stick input","0x00: disable, 0x01 enable");

	// Parameters
	aciPublishParameter(&ALARM_battery_warning_voltage_high,VARTYPE_UINT16,0x0001,"battery_warning_voltage_high","First battery warning level","mV");
	aciPublishParameter(&ALARM_battery_warning_voltage_low,VARTYPE_UINT16,0x0002,"battery_warning_voltage_low","Second battery warning level","mV");
	aciPublishParameter(&buzzer_warnings,VARTYPE_UINT8,0x0003,"buzzer_warnings","Enable/Disable acoustic warnings","");
	aciPublishParameter(&PTU_cam_option_4_version,VARTYPE_UINT8,0x0004,"PTU_cam_option_4_version","Version of Pelican/Firefly PanTilt camera mount option 4","1 or 2");
	aciPublishParameter(&PTU_cam_angle_roll_offset,VARTYPE_INT32,0x0400,"cam_angle_roll_offset","Camera roll angle offset","0.001deg");
	aciPublishParameter(&PTU_cam_angle_pitch_offset,VARTYPE_INT32,0x0401,"cam_angle_pitch_offset","Camera pitch angle offset","0.001deg");

	aciPublishParameter(&PTU_enable_plain_ch7_to_servo,VARTYPE_UINT8,0x0005,"PTU_enable_plain_ch7_to_servo","Channel7 mapped directly to servo out","1=enable 0=disable");


#else
	// Matlab parameters

	aciPublishParameter(&matlab_params.p01,VARTYPE_STRUCT_WITH_SIZE(60),0x0F00,"Matlab Parameter Set 1","Matlab paramters 1..15","");
	aciPublishParameter(&matlab_params.p16,VARTYPE_STRUCT_WITH_SIZE(60),0x0F01,"Matlab Parameter Set 2","Matlab paramters 15..30","");
	aciPublishParameter(&matlab_params.p30,VARTYPE_STRUCT_WITH_SIZE(48),0x0F02,"Matlab Parameter Set 3","Matlab paramters 30..40 and CRC","");
#endif

	//get initial values from flash for all parameters
	lpc_aci_ReadParafromFlash();

}


