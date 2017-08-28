/*

   AscTec AutoPilot HL SDK v3.0

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

 ** Checked for Kumar Lab by Andrew Block 5/20/16 **

*/

/**********************************************************
  Header files
 **********************************************************/
#include "LPC214x.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "hardware.h"
#include "irq.h"
#include "i2c.h"
#include "gpsmath.h"
#include "adc.h"
#include "uart.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "sdk.h"
#include "buzzer.h"
#include "pelican_ptu.h"

// Only one of these should be enabled (if both defined, IMU takes priority)
#define OUTPUT_IMU
//#define OUTPUT_STATUS

#if defined(OUTPUT_STATUS) || defined(OUTPUT_IMU)
#define OUTPUT
static const int OUTPUT_RATE = 100;
#endif

/* *********************************************************
   Function declarations
 ********************************************************* */

void Initialize(void);
void feed(void);
void beeper(uint8_t);

/**********************************************************
  Global Variables
 **********************************************************/
struct HL_STATUS HL_Status;
struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;

volatile unsigned int int_cnt = 0, cnt = 0, mainloop_cnt = 0;
volatile uint8_t mainloop_trigger = 0;
volatile unsigned int GPS_timeout = 0;
volatile unsigned int trigger_cnt = 0;
volatile uint8_t SYSTEM_initialized = 0;

void timer0ISR(void) __irq
{
  T0IR = 0x01;      //Clear the timer 0 interrupt
  IENABLE;
  trigger_cnt++;
  if(trigger_cnt == ControllerCyclesPerSecond)
  {
    trigger_cnt=0;
    HL_Status.up_time++;
    HL_Status.cpu_load = mainloop_cnt;

    mainloop_cnt=0;
  }

  if(mainloop_trigger < 10)
    mainloop_trigger++;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

/**********************************************************
  MAIN
 **********************************************************/
int	main (void) {

  static int vbat1; //battery_voltage (lowpass-filtered)

  init();
  buzzer(OFF);
  PTU_init();
  LL_write_init();
  ADC0triggerSampling(1<<VOLTAGE_1); //activate ADC sampling

  HL_Status.up_time=0;

  LED(1,ON);

  while(1)
  {
    if(mainloop_trigger)
    {
#if 0
      if(GPS_timeout < ControllerCyclesPerSecond)
        GPS_timeout++;
      else if(GPS_timeout == ControllerCyclesPerSecond)
      {
        GPS_timeout=ControllerCyclesPerSecond+1;
        GPS_Data.status=0;
        GPS_Data.numSV=0;
      }
#endif
      //battery monitoring
      ADC0getSamplingResults(0xFF,adcChannelValues);
      vbat1=(vbat1*14+(adcChannelValues[VOLTAGE_1]*9872/579))/15;	//voltage in mV
      HL_Status.battery_voltage_1=vbat1;

      mainloop_cnt++;

      if(!(mainloop_cnt % 10))
        buzzer_handler(HL_Status.battery_voltage_1);

      if(mainloop_trigger)
        mainloop_trigger--;

      mainloop();
    }
  }
  return 0;
}


void mainloop(void) //mainloop is triggered at 1 kHz
{
// no gps or magnetometer attached so they do not need to be initialized
#if 0
  static uint8_t led_cnt=0, led_state=1;

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
#endif
  uint8_t t;
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

  //run SDK mainloop. Please put all your data handling / controller code in sdk.c
  SDK_mainloop();

  //write data to transmit buffer for immediate transfer to LL processor
  HL2LL_write_cycle();

#if defined(OUTPUT)
  static int uart_cnt = 0;
  static uint8_t seq = 0;
  if(++uart_cnt == ControllerCyclesPerSecond/OUTPUT_RATE)
  {
#if defined(OUTPUT_IMU)
    Output_Data.loop_rate = HL_Status.cpu_load;
    Output_Data.voltage = HL_Status.battery_voltage_1;

    // Asctec co-ordinate frame is X front, Y right, Z down
    // but we want X front, Y left, Z up
    Output_Data.roll = RO_ALL_Data.angle_roll/10;
    Output_Data.pitch = -RO_ALL_Data.angle_pitch/10;
    if(RO_ALL_Data.angle_yaw < 180000)
      Output_Data.yaw = -RO_ALL_Data.angle_yaw/10;
    else
      Output_Data.yaw = -(RO_ALL_Data.angle_yaw - 360000)/10;
    Output_Data.ang_vel[0] = RO_ALL_Data.angvel_roll;
    Output_Data.ang_vel[1] = -RO_ALL_Data.angvel_pitch;
    Output_Data.ang_vel[2] = -RO_ALL_Data.angvel_yaw;
    Output_Data.acc[0] = RO_ALL_Data.acc_x/10;
    Output_Data.acc[1] = -RO_ALL_Data.acc_y/10;
    Output_Data.acc[2] = -RO_ALL_Data.acc_z/10;

    Output_Data.height = RO_ALL_Data.fusion_height;
    Output_Data.dheight = RO_ALL_Data.fusion_dheight;

    Output_Data.mag[0] = RO_ALL_Data.Hx;
    Output_Data.mag[1] = -RO_ALL_Data.Hy;
    Output_Data.mag[2] = -RO_ALL_Data.Hz;

    unsigned int i;
    for(i = 0; i < 8; i++)
      Output_Data.radio[i] = RO_ALL_Data.channel[i]/16;
    //for(i = 0; i < 4; i++)
    //  Output_Data.rpm[i] = RO_ALL_Data.motor_rpm[i];

    Output_Data.seq = seq;

    if(ringbuffer(RBFREE, 0, 0) > (sizeof(Output_Data)))
    {
      UART_SendPacket(&Output_Data, sizeof(Output_Data), TYPE_OUTPUT_DATA);
    }
#elif defined(OUTPUT_STATUS)
    Status_Data.loop_rate = HL_Status.cpu_load;
    Status_Data.voltage = HL_Status.battery_voltage_1;
    Status_Data.seq = seq;

    if(ringbuffer(RBFREE, 0, 0) > (sizeof(Status_Data)))
    {
      UART_SendPacket(&Status_Data, sizeof(Status_Data), TYPE_STATUS_DATA);
    }
#endif
    uart_cnt = 0;
    seq++;
  }
#endif
}
