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

#ifndef MAIN_H_
#define MAIN_H_

extern void mainloop(void);
extern void timer0ISR(void);


volatile unsigned int GPS_timeout;
volatile char SYSTEM_initialized;

#define ControllerCyclesPerSecond 	1000

#if (ControllerCyclesPerSecond%50)
#error "Use a multiple of 50 for ControllerCyclesPerSecond"
#endif
#define OFF 0
#define ON  1

#define NORMAL 0

//packet descriptors
#define PD_IMURAWDATA       0x01
#define PD_LLSTATUS        	0x02
#define PD_IMUCALCDATA      0x03
#define PD_HLSTATUS        	0x04

#define PD_CTRLOUT			0x11
#define PD_FLIGHTPARAMS     0x12
#define PD_CTRLCOMMANDS		0x13
#define PD_CTRLINTERNAL		0x14
#define PD_RCDATA       	0x15
#define PD_CTRLSTATUS		0x16

#define PD_WAYPOINT     	0x20
#define PD_CURRENTWAY   	0x21
#define PD_NMEADATA     	0x22
#define PD_GPSDATA			0x23

#define PD_CAMERACOMMANDS	0x30
#define PD_RO_ALL_DATA		0x90

#define PD_JETI_SETNAME		0xA0
#define PD_JETI_SETSENSOR   0xA1
#define PD_JETI_UPDATESDATA 0xA2
#define PD_JETI_SETALARM	0xA3
#define PD_JETI_SETTEXT		0xA4
#define PD_JETI_RCDATA		0xA5
#define PD_JETI_SETSENSOR2	0xA6
#define PD_JETI_SETTEXT2	0xA7
#define PD_JETI_UPDATESENSORBLOCK_1 0xA8
#define PD_JETI_UPDATESENSORBLOCK_2 0xA9


struct MATLAB_DEBUG
{
	short debug01;
	short debug02;
	short debug03;
	short debug04;
	short debug05;
	short debug06;
	short debug07;
	short debug08;
	short debug09;
	short debug10;
	short debug11;
    short debug12;
	short debug13;
	short debug14;
	short debug15;
	short debug16;
	short debug17;
	short debug18;
	short debug19;
	short debug20;

	short cpu_load;
	short battery_voltage;

};
extern struct MATLAB_DEBUG matlab_debug;

//
struct MATLAB_PARAMS
{
	 unsigned int p01;
	 unsigned int p02;
	 unsigned int p03;
	 unsigned int p04;
	 unsigned int p05;
	 unsigned int p06;
	 unsigned int p07;
	 unsigned int p08;
	 unsigned int p09;
	 unsigned int p10;
	 unsigned int p11;
	 unsigned int p12;
	 unsigned int p13;
	 unsigned int p14;
	 unsigned int p15;
	 unsigned int p16;
	 unsigned int p17;
	 unsigned int p18;
	 unsigned int p19;
	 unsigned int p20;
	 unsigned int p21;
	 unsigned int p22;
	 unsigned int p23;
	 unsigned int p24;
	 unsigned int p25;
	 unsigned int p26;
 	 unsigned int p27;
 	 unsigned int p28;
 	 unsigned int p29;
 	 unsigned int p30;
 	 unsigned int p31;
 	 unsigned int p32;
 	 unsigned int p33;
 	 unsigned int p34;
 	 unsigned int p35;
 	 unsigned int p36;
 	 unsigned int p37;
 	 unsigned int p38;
 	 unsigned int p39;
 	 unsigned int p40;
	 unsigned int crc;
};
extern struct MATLAB_PARAMS matlab_params, matlab_params_tmp;

struct  MATLAB_UART
{
	short ctrl01;
	short ctrl02;
	short ctrl03;
	short ctrl04;
	short ctrl05;
	short ctrl06;
	short ctrl07;
	short ctrl08;
	short ctrl09;
	short ctrl10;
	short ctrl11;
	short ctrl12;
	unsigned short crc;

};
extern struct MATLAB_UART matlab_uart, matlab_uart_tmp;


//system status defines for buzzer handling
#define FM_COMPASS_FAILURE			0x10
#define FM_CALIBRATION_ERROR		0x100
#define FM_CALIBRATION_ERROR_GYROS 	0x200
#define FM_CALIBRATION_ERROR_ACC   	0x400
#define FM_ADC_STARTUP_ERROR        0x800
#define FM_MAG_FIELD_STRENGTH_ERROR 0x4000
#define FM_MAG_INCLINATION_ERROR	0x8000



struct IMU_CALCDATA {
//angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
    int angle_nick;
    int angle_roll;
    int angle_yaw;

//angular velocities, 16 bit values, bias free, 1 LSB = 0.0154 °/s (=> 64.8 = 1 °/s)
    int angvel_nick;
    int angvel_roll;
    int angvel_yaw;

//acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g, body frame coordinate system
    short acc_x_calib;
    short acc_y_calib;
    short acc_z_calib;

//horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
    short acc_x;
    short acc_y;
    short acc_z;

//reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
    int acc_angle_nick;
    int acc_angle_roll;

//total acceleration measured (10000 = 1g)
    int acc_absolute_value;

//magnetic field sensors output, offset free and scaled; units not determined, as only the direction of the field vector is taken into account
    int Hx;
    int Hy;
    int Hz;

//compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
    int mag_heading;

//pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
    int speed_x;
    int speed_y;
    int speed_z;

//height in mm (after data fusion)
    int height;

//diff. height in mm/s (after data fusion)
    int dheight;

//diff. height measured by the pressure sensor [mm/s]
    int dheight_reference;

//height measured by the pressure sensor [mm]
    int height_reference;
};
extern struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;



#endif /*MAIN_H_*/

