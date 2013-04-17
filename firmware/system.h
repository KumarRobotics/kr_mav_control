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

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>

extern unsigned int processorClockFrequency(void);
extern unsigned int peripheralClockFrequency(void);
extern void delay(int);
extern void init(void);
extern void pll_init(void);
extern void pll_feed(void);
extern void init_timer0(void);
extern void init_interrupts(void);
extern void init_ports(void);
extern void init_spi(void);
extern void init_pwm(void);
extern void init_get_calibdata_from_flash(void);
extern void write_calibdata_to_flash(void);
extern void init_spi1(void);
extern void SPI1Send(char *, unsigned int, uint8_t);
extern void PWM_Init( void );
extern void SPI_get_data(unsigned int);

extern uint8_t CAM_Commands_received;

struct HL_STATUS {
      int16_t battery_voltage_1;
      int16_t battery_voltage_2;

      int16_t up_time;
      int16_t flight_time;

      int latitude;
      int longitude;

      int16_t status;
      int16_t cpu_load;
      int16_t yawenabled;
      int16_t chksum_error;
};

extern struct HL_STATUS HL_Status;


//PWM defines
#define PWM_CYCLE		1200
#define PWM_OFFSET		200

#define MR0_INT			1 << 0
#define MR1_INT			1 << 1
#define MR2_INT			1 << 2
#define MR3_INT			1 << 3
#define MR4_INT			1 << 8
#define MR5_INT			1 << 9
#define MR6_INT			1 << 10

#define TCR_CNT_EN		0x00000001
#define TCR_RESET		0x00000002
#define TCR_PWM_EN		0x00000008

#define PWMMR0I			1 << 0
#define PWMMR0R			1 << 1
#define PWMMR0S			1 << 2
#define PWMMR1I			1 << 3
#define PWMMR1R			1 << 4
#define PWMMR1S			1 << 5
#define PWMMR2I			1 << 6
#define PWMMR2R			1 << 7
#define PWMMR2S			1 << 8
#define PWMMR3I			1 << 9
#define PWMMR3R			1 << 10
#define PWMMR3S			1 << 11
#define PWMMR4I			1 << 12
#define PWMMR4R			1 << 13
#define PWMMR4S			1 << 14
#define PWMMR5I			1 << 15
#define PWMMR5R			1 << 16
#define PWMMR5S			1 << 17
#define PWMMR6I			1 << 18
#define PWMMR6R			1 << 19
#define PWMMR6S			1 << 20

#define PWMSEL2			1 << 2
#define PWMSEL3			1 << 3
#define PWMSEL4			1 << 4
#define PWMSEL5			1 << 5
#define PWMSEL6			1 << 6
#define PWMENA1			1 << 9
#define PWMENA2			1 << 10
#define PWMENA3			1 << 11
#define PWMENA4			1 << 12
#define PWMENA5			1 << 13
#define PWMENA6			1 << 14

#define LER0_EN			1 << 0
#define LER1_EN			1 << 1
#define LER2_EN			1 << 2
#define LER3_EN			1 << 3
#define LER4_EN			1 << 4
#define LER5_EN			1 << 5
#define LER6_EN			1 << 6

#endif /*SYSTEM_H_*/

