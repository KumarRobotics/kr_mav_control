/*
 * buzzer.h
 *
 *  Created on: 14.09.2011
 *      Author: daniel
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#define BUZZER_WARNING_INIT_BEEP	0x01
#define BUZZER_WARNING_GPS_BEEP		0x02

void buzzer_handler(unsigned int);
void buzzer(unsigned char);

extern unsigned short ALARM_battery_warning_voltage_high;
extern unsigned short ALARM_battery_warning_voltage_low;

extern unsigned char buzzer_warnings;

#endif /* BUZZER_H_ */
