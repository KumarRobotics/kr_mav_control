/*
 * buzzer.c
 *
 *  Created on: 14.09.2011
 *      Author: daniel
 */

#include "buzzer.h"
#include "main.h"
#include "hardware.h"
#include "system.h"
#include "LL_HL_comm.h"
#include "gpsmath.h"
#include "LPC214x.h"

//Buzzer active defines
#define BU_INIT				0x01
#define BU_ERROR_GYRO 		0x02
#define BU_ERROR_ACC  		0x04
#define BU_ERROR_ADC		0x08
#define BU_BATTERY			0x10
#define BU_COMPASS_FAILURE	0x20
#define BU_WARNING_MAG_FS	0x40
#define BU_WARNING_MAG_INC	0x80
#define BU_GPS_BEEP			0x100

#define BUZZ_LENGTH		5	//50 ms
#define BUZZ_PAUSE		5 //50 ms
#define BUZZ_INTERVAL	200 //2 s
#define BUZZ_MAG_WARNING_TIMEOUT 500 //mag warning for 5 seconds only
#define BUZZ_NR_OF_WARNINGS		9 //total number of different buzzer signals (see BU_ defines above)

void buzzer_handler(unsigned int vbat)	//needs to be triggered at 100 Hz
{
	unsigned int buz_active = 0;

	unsigned int buz_priority=0;
	static uint16_t buz_cnt=0;

	static int bat_cnt=0, bat_warning=0;
	static char bat_warning_enabled=0;

	if(++buz_cnt >= BUZZ_INTERVAL)
    buz_cnt=0;

	//battery warning
	if(++bat_cnt == 100)
    bat_cnt=0;

	if(vbat < BATTERY_MIN_VOLTAGE)
    vbat = BATTERY_MIN_VOLTAGE;

	if(vbat < BATTERY_WARNING_VOLTAGE)	//decide if it's really an empty battery
	{
		if(bat_warning < ControllerCyclesPerSecond/5)
      bat_warning++;
		else
      bat_warning_enabled=1;
	}
	else
	{
		if(bat_warning > 10)
      bat_warning -= 2;
		else
		{
			bat_warning_enabled = 0;
			buz_active &= ~BU_BATTERY;
		}
	}
	if(bat_warning_enabled)
	{
		if(bat_cnt > ((vbat - BATTERY_MIN_VOLTAGE)/BAT_DIV))
      buz_active |= BU_BATTERY; //Beeper on
		else
      buz_active &= ~BU_BATTERY; //Beeper off
		buz_priority |= BU_BATTERY;
	}
	else
	{
		buz_active &= ~BU_BATTERY;
		buz_priority &= ~BU_BATTERY;
	}

#ifdef GPS_BEEP
	if(((GPS_Data.status & 0xFF) != 3) && (LL_1khz_attitude_data.RC_data[5] > 200))	//no lock and in GPS mode
	{
		buz_priority |= BU_GPS_BEEP;
		if(buz_cnt < 5) buz_active |= BU_GPS_BEEP;
		else buz_active &= ~BU_GPS_BEEP;
	}
	else
	{
		buz_active &= ~BU_GPS_BEEP;
		buz_priority &= ~BU_GPS_BEEP;
	}
#endif

#ifdef ERROR_BEEP
	//gyro error
	if((LL_1khz_attitude_data.flightMode & FM_CALIBRATION_ERROR_GYROS) && (SYSTEM_initialized))
	{
		buz_priority |= BU_ERROR_GYRO;
		if(buz_cnt < 155) buz_active|=BU_ERROR_GYRO;
		else if(buz_cnt < 160) buz_active&=~BU_ERROR_GYRO;
		else if(buz_cnt < 165) buz_active|=BU_ERROR_GYRO;
		else buz_active &= ~BU_ERROR_GYRO;
	}
	else
	{
		buz_priority &= ~BU_ERROR_GYRO;
		buz_active &= ~BU_ERROR_GYRO;
	}

	//ACC error
	if((LL_1khz_attitude_data.flightMode & FM_CALIBRATION_ERROR_ACC) && (SYSTEM_initialized))
	{
		buz_priority |= BU_ERROR_ACC;
		if(buz_cnt < 145) buz_active |= BU_ERROR_ACC;
		else if(buz_cnt < 150) buz_active &= ~BU_ERROR_ACC;
		else if(buz_cnt < 155) buz_active |= BU_ERROR_ACC;
		else if(buz_cnt < 160) buz_active &= ~BU_ERROR_ACC;
		else if(buz_cnt < 165) buz_active |= BU_ERROR_ACC;
		else buz_active&=~BU_ERROR_ACC;
	}
	else
	{
		buz_priority &= ~BU_ERROR_ACC;
		buz_active &= ~BU_ERROR_ACC;
	}

	//ADC error
	if((LL_1khz_attitude_data.flightMode & FM_ADC_STARTUP_ERROR) && (SYSTEM_initialized))
	{
		buz_priority |= BU_ERROR_ADC;
		if(buz_cnt < 135) buz_active |= BU_ERROR_ADC;
		else if(buz_cnt < 140) buz_active &= ~BU_ERROR_ADC;
		else if(buz_cnt < 145) buz_active |= BU_ERROR_ADC;
		else if(buz_cnt < 150) buz_active &= ~BU_ERROR_ADC;
		else if(buz_cnt < 155) buz_active |= BU_ERROR_ADC;
		else if(buz_cnt < 160) buz_active &= ~BU_ERROR_ADC;
		else if(buz_cnt < 165) buz_active |= BU_ERROR_ADC;
		else buz_active &= ~BU_ERROR_ADC;
	}
	else
	{
		buz_priority &= ~BU_ERROR_ADC;
		buz_active &= ~BU_ERROR_ADC;
	}

#ifdef MAG_BEEP
	static uint16_t error_cnt_mag_fs = 0;
	static uint16_t error_cnt_mag_inc = 0;
	static uint16_t error_cnt_compass = 0;

	//compass failure: warn 3 seconds only
	if((LL_1khz_attitude_data.flightMode & FM_COMPASS_FAILURE) && (SYSTEM_initialized) && (error_cnt_compass < 400))
	{
		buz_priority |= BU_COMPASS_FAILURE;
		if(buz_cnt%100 < 5) buz_active |= BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 10) buz_active &= ~BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 15) buz_active |=BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 20) buz_active &= ~BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 25) buz_active |=BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 30) buz_active &= ~BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 35) buz_active |=BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 40) buz_active &= ~BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 45) buz_active |= BU_COMPASS_FAILURE;
		else if(buz_cnt%100 < 50) buz_active &= ~BU_COMPASS_FAILURE;
		else buz_active &= ~BU_COMPASS_FAILURE;
    error_cnt_compass++;
	}
	else
	{
		buz_priority &= ~BU_COMPASS_FAILURE;
		buz_active &= ~BU_COMPASS_FAILURE;
	}

	//mag fieldstrength warning: warn 3 times only
	if((LL_1khz_attitude_data.flightMode & FM_MAG_FIELD_STRENGTH_ERROR) && (SYSTEM_initialized) && (error_cnt_mag_fs < 400))
	{
		buz_priority |= BU_WARNING_MAG_FS;
		if(buz_cnt%100 < 5) buz_active|=BU_WARNING_MAG_FS;
		else if(buz_cnt%100 < 10) buz_active &= ~BU_WARNING_MAG_FS;
		else if(buz_cnt%100 < 15) buz_active |=BU_WARNING_MAG_FS;
		else if(buz_cnt%100 < 20) buz_active &= ~BU_WARNING_MAG_FS;
		else if(buz_cnt%100 < 25) buz_active |=BU_WARNING_MAG_FS;
		else if(buz_cnt%100 < 30) buz_active &= ~BU_WARNING_MAG_FS;
		else buz_active &= ~BU_WARNING_MAG_FS;
    error_cnt_mag_fs++;
	}
	else
	{
		buz_priority &= ~BU_WARNING_MAG_FS;
		buz_active &= ~BU_WARNING_MAG_FS;
	}

	//mag inclination warning: warn 3 times only
	if((LL_1khz_attitude_data.flightMode & FM_MAG_INCLINATION_ERROR) && (SYSTEM_initialized) && (error_cnt_mag_inc < 400))
	{
		buz_priority |=BU_WARNING_MAG_INC;
		if(buz_cnt%100 < 5) buz_active|=BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 10) buz_active &= ~BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 15) buz_active |=BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 20) buz_active &= ~BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 25) buz_active |=BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 30) buz_active &= ~BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 35) buz_active |=BU_WARNING_MAG_INC;
		else if(buz_cnt%100 < 40) buz_active &= ~BU_WARNING_MAG_INC;
		else buz_active &= ~BU_WARNING_MAG_INC;
    error_cnt_mag_inc++;
	}
	else
	{
		buz_priority &= ~BU_WARNING_MAG_INC;
		buz_active &= ~BU_WARNING_MAG_INC;
	}
#endif

#endif

#ifdef INIT_BEEP

		if(!SYSTEM_initialized)
		{
			buz_priority |= BU_INIT;
			if(buz_cnt%100 < 5) buz_active |= BU_INIT;
			else if(buz_cnt%100 < 10) buz_active &= ~BU_INIT;
			else if(buz_cnt%100 < 15) buz_active |= BU_INIT;
			else buz_active &= ~BU_INIT;
		}
		else
		{
			buz_active &= ~BU_INIT;
			buz_priority &= ~BU_INIT;
		}
#endif

	//buzzer control
	for(uint8_t i=0; i < BUZZ_NR_OF_WARNINGS; i++)
	{
		if(buz_priority & (1<<i))
		{
			buz_active &= (1<<i);
			i = BUZZ_NR_OF_WARNINGS;
		}
	}

	if(buz_active) buzzer(ON);
	else buzzer(OFF);
}


void buzzer(uint8_t offon)
{
	if(offon)	//beeper on
	{
		IOSET1 = (1<<17);
	}
	else
	{
		IOCLR1 = (1<<17);
	}
}
