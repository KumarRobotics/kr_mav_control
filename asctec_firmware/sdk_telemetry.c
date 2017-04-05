#include "main.h"
#include "sdk.h"
#include "LL_HL_comm.h"
#include "jetiTelemetry.h"
#include "sdk_telemetry.h"
#include "gpsmath.h"
#include "system.h"
#include <string.h>


void SDK_jetiAscTecExampleUpdateDisplay(unsigned char state) {
	char text[31];
	switch (state) {
	//				Line	   11111111111111112222222222222222
	case 0:
#ifndef MATLAB
		if (RO_ALL_Data.flying)
			jetiSetTextDisplay("<-Waypoint Test  EmergencyMode->");
		else
#endif
			jetiSetTextDisplay("AscTec JetiTest  EmergencyMode->");
		break;
	case 1:
		if (emergencyMode == EM_SAVE)
			jetiSetTextDisplay("Current EmMode<>Direct Landing");
		else if (emergencyMode == EM_SAVE_EXTENDED_WAITING_TIME)
			jetiSetTextDisplay("Current EmMode<>Wait&Land");
		else if (emergencyMode == EM_RETURN_AT_MISSION_SUMMIT)
			jetiSetTextDisplay("Current EmMode<>Come Home High");
		else
			jetiSetTextDisplay("Current EmMode<>Come Home ");
		break;
	case 2:
		jetiSetTextDisplay("EmMode v=Set  <>Direct Landing");
		break;
	case 3:
		jetiSetTextDisplay("EmMode v=Set  <>Wait&Land");
		break;
	case 4:
		jetiSetTextDisplay("EmMode v=Set  <>Come Home     ");
		break;
	case 5:
		jetiSetTextDisplay("EmMode v=Set  <>Come Home High");
		break;
	case 6:
		sprintf(&text,"WP Act. v=Stop <>WP# %2i Dist: %2im",wpExampleWpNr,wpCtrlDistToWp/10);
		jetiSetTextDisplay((unsigned char *)&text);
		break;
	}
}

void SDK_jetiAscTecExampleKeyChange(unsigned char key) {
	static unsigned char displayState = 0;

	switch (displayState) {
	case 0:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			break;
		case JETI_KEY_LEFT:
			if (RO_ALL_Data.flying)
			{
				wpExampleActive=1;
				displayState=6;
			}
			break;
		case JETI_KEY_RIGHT:
			displayState++;
			break;
		}
		break;

	case 1:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			break;
		case JETI_KEY_LEFT:
			displayState = 0;
			break;
		case JETI_KEY_RIGHT:
			displayState++;
			break;
		}
		break;

	case 2:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			SDK_SetEmergencyMode(EM_SAVE);
			displayState = 0;
			break;
		case JETI_KEY_LEFT:
			displayState = 5;
			break;
		case JETI_KEY_RIGHT:
			displayState++;
			break;
		}
		break;

	case 3:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			SDK_SetEmergencyMode(EM_SAVE_EXTENDED_WAITING_TIME);
			displayState = 0;
			break;
		case JETI_KEY_LEFT:
			displayState--;
			break;
		case JETI_KEY_RIGHT:
			displayState++;
			break;
		}
		break;
	case 4:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			SDK_SetEmergencyMode(EM_RETURN_AT_PREDEFINED_HEIGHT);
			displayState = 0;
			break;
		case JETI_KEY_LEFT:
			displayState--;
			break;
		case JETI_KEY_RIGHT:
			displayState++;
			break;
		}
		break;

	case 5:
		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			SDK_SetEmergencyMode(EM_RETURN_AT_MISSION_SUMMIT);
			displayState = 0;
			break;
		case JETI_KEY_LEFT:
			displayState--;
			break;
		case JETI_KEY_RIGHT:
			displayState = 2;
			break;
		}
		break;


	case 6:
		//switch back when waypoint example is finished
		if (wpExampleActive==0)
			displayState = 0;

		switch (key) {
		case JETI_KEY_UP:
			break;
		case JETI_KEY_DOWN:
			displayState = 0;
			wpExampleActive=0;
			break;
		case JETI_KEY_LEFT:
			break;
		case JETI_KEY_RIGHT:
			break;
		}
		break;
	}

	SDK_jetiAscTecExampleUpdateDisplay(displayState);
}

void SDK_jetiAscTecExampleInit(void) {
	jetiSetDeviceName("AscTec SDK");
	jetiSetTextDisplay("AscTec Example  ->more Infos");
	jetiInitValue(0, "Pitch", "°");
	jetiSetDecimalPoint(0, 2);
	jetiInitValue(1, "Roll", "°");
	jetiSetDecimalPoint(1, 2);
	jetiInitValue(2, "Yaw", "°");
	jetiSetDecimalPoint(2, 2);
	jetiInitValue(3, "Height", "m");
	jetiSetDecimalPoint(3, 2);
	jetiInitValue(4, "FlightMode", "");
	jetiInitValue(5, "Speed", "m/s");
	jetiSetDecimalPoint(5, 1);
	jetiInitValue(6, "GPS", "%");
	jetiInitValue(7, "BAT", "V");
	jetiSetDecimalPoint(7, 2);
	jetiInitValue(8, "CPU Time", "us"); //TODO Einheit checken!
	jetiInitValue(9, "Lat", "");
	jetiInitValue(10, "Lon", "");
	jetiInitValue(11, "LatRem", "");
	jetiInitValue(12, "LonRem", "");
	jetiSetValue14B(0, 0);
	jetiSetValue14B(1, 0);
	jetiSetValue22B(2, 0);
	jetiSetValue22B(3, 0);
	jetiSetValue6B(4, 0);
	jetiSetValue14B(5, 0);
	jetiSetValue14B(6, 0);
	jetiSetValue22B(7, 0);
	jetiSetValue14B(8, 0);
	jetiSetValue14B(9, 0);
	jetiSetValue14B(10, 0);
	jetiSetValue22B(11, 0);
	jetiSetValue22B(12, 0);

	SDK_jetiAscTecExampleUpdateDisplay(0);
}

void SDK_jetiAscTecExampleRun(void) {
	int speed;
	int gps_quality;
	unsigned char key;
	static unsigned char first = 0;
	//counter for updating the jeti display regularly
	static unsigned char jetiDisplayUpdateCnt=0;

	if (!first) {
		first = 1;
		SDK_jetiAscTecExampleInit();
	}

	jetiSetValue14B(0, LL_1khz_attitude_data.angle_pitch);
	jetiSetValue14B(1, LL_1khz_attitude_data.angle_roll);
	jetiSetValue22B(2, LL_1khz_attitude_data.angle_yaw);
	jetiSetValue22B(3, LL_1khz_attitude_data.height / 10);


	jetiSetValue14B(9, GPS_Data.latitude/10000000);
	jetiSetValue14B(10, GPS_Data.longitude/10000000);
	jetiSetValue22B(11, (GPS_Data.latitude/10)%1000000);
	jetiSetValue22B(12, (GPS_Data.longitude/10)%1000000);

	if ((LL_1khz_attitude_data.flightMode & FM_POS) && ((GPS_Data.status & 0xFF) == 3))
		jetiSetValue6B(4, 2);
	else if (LL_1khz_attitude_data.flightMode & FM_HEIGHT)
		jetiSetValue6B(4, 1);
	else
		jetiSetValue6B(4, 0);

	speed = fast_sqrt(GPS_Data.speed_x * GPS_Data.speed_x + GPS_Data.speed_y
			* GPS_Data.speed_y);
	speed /= 100;
	jetiSetValue14B(5, speed);

	if (((GPS_Data.status & 0xFF) != 3) || (GPS_Data.numSV < 3)
			|| (GPS_Data.horizontal_accuracy > 10000))
		gps_quality = 0;
	else
		gps_quality = 100 - ((GPS_Data.horizontal_accuracy - 1500) / 85);
	if (gps_quality > 100)
		gps_quality = 100;

	jetiSetValue14B(6, gps_quality);
	jetiSetValue22B(7, LL_1khz_attitude_data.battery_voltage1 / 10);
	if ((LL_1khz_attitude_data.battery_voltage1 < 10500)
			&& (LL_1khz_attitude_data.battery_voltage1 > 5000))
		jetiSetAlarm('U', 0);
	else
		jetiSetAlarm(0, 0);

	jetiSetValue14B(8, HL_Status.cpu_load);

	key=jetiCheckForKeyChange();
	jetiDisplayUpdateCnt++;
	if ((jetiDisplayUpdateCnt==20) || (key))
	{
		SDK_jetiAscTecExampleKeyChange(key);
		jetiDisplayUpdateCnt=0;
	}
}
