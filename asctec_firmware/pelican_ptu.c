/*
*	Camera Pan Tilt Unit (PTU) control for AscTec Pelican
*	to be included in HL SDK
*
*	To use this code, please do the following steps:
*   - copy "pelican_ptu.c" and "pelican_ptu.h" to HL SDK directory
*	- modify makefile: Look for the line "SRCARMINT += ssp.c" and add "SRCARMINT += pelican_ptu.c" directly underneath it
*	- add '#include"pelican_ptu.h"' under "Header files" in main.c
*   - add 'PTU_init();' under 'init();' in main.c
*	- add 'PTU_update();' under 'led_cnt++;' in the mainloop(void) function in main.c
*	- Build the project
*
*/

/*
* Modified by Justin Thomas to use the PWM signals directly for our own servos
*/

#include "main.h"
#include "system.h"
#include "pelican_ptu.h"
#include "system.h"
#include "LPC214x.h"

struct CAMERA_PTU CAMERA_ptu;
struct CAMERA_COMMANDS CAMERA_Commands;

void PTU_init(void)
{
#ifdef HUMMINGBIRD_ROLL_SERVO
#ifndef HUMMINGBIRD_ROLL_SERVO_ON_SSEL0
	PINSEL0&=~0x01;
	PINSEL0|=0x02;
#else
	PINSEL0|=0x8000;
	PINSEL0&=~0x4000;
#endif
#endif

#ifdef PELICAN_PTU
  CAMERA_ptu.servo_pitch_offset=60500;
	CAMERA_ptu.servo_pitch_scale=54853;
	CAMERA_ptu.servo_pitch_min=44000;
	CAMERA_ptu.servo_pitch_max=128000;

	CAMERA_ptu.servo_roll_offset=74000;
	CAMERA_ptu.servo_roll_scale=160000;
	CAMERA_ptu.servo_roll_min=53900;
	CAMERA_ptu.servo_roll_max=94500;
#endif
}

void PTU_update(int servo_pos)
{
	static int ptu_cnt=0;
	if(++ptu_cnt>9)	//generate 100Hz
	{
		ptu_cnt=0;
		SERVO_roll_move(servo_pos);
	}
}

void SERVO_roll_move (int angle)
{
  unsigned long value = 10 * angle; 

  if (value < 60000) value = 60000;
  if (value > 120000) value = 120000;

#ifdef HUMMINGBIRD_ROLL_SERVO_ON_SSEL0
    PWMMR2 = value;
#else
    PWMMR1 = value;
#endif
    PWMLER = LER5_EN|LER1_EN|LER2_EN;
}

void SERVO_pitch_move (int angle)
{
    unsigned int value = 10*angle;

    PWMMR5 = value;
    PWMLER = LER5_EN|LER1_EN|LER2_EN;
}
