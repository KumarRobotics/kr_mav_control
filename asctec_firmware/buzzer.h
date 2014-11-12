/*
 * buzzer.h
 *
 *  Created on: 14.09.2011
 *      Author: daniel
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdint.h>

//#define GPS_BEEP	  //warning if GPS has no lock
#define ERROR_BEEP  //sensor calibration errors signaled by buzzer
//#define MAG_BEEP    //beep for magnetometer errors
#define INIT_BEEP 	//double beep during system initialization

void buzzer_handler(unsigned int);
void buzzer(uint8_t);

#endif /* BUZZER_H_ */
