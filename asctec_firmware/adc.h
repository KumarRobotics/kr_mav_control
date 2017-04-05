/*****************************************************************************
 *   adc.h:  Header file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

#define ADC_INTERRUPT_FLAG	0	/* 1 is interrupt driven, 0 is polling */

#define ADC_OFFSET	0x10
#define ADC_INDEX	4

#define ADC_DONE	0x80000000
#define ADC_OVERRUN	0x40000000
#define ADC_ADINT	0x00010000

#define VOLTAGE_1	2
#define VOLTAGE_2  	4
#define CURRENT_1	3
#define CURRENT_2	1

#define ADC_NUM		8		/* for LPC2146/8 */
#define ADC_CLK		1000000		/* set to 1Mhz */

extern void ADC0Handler( void ) __irq;
extern void ADC1Handler( void ) __irq;
extern unsigned int ADCInit( unsigned int ADC_Clk );
extern unsigned int ADC0Read( uint8_t channelNum );
extern unsigned int ADC1Read( uint8_t channelNum );
extern void ADC0triggerSampling(uint8_t selectChannels); //activate continuous sampling on selected ADC channels
extern void ADC0getSamplingResults(uint8_t selectChannels, unsigned int * channelValues);


extern volatile unsigned int ADC0Value[ADC_NUM], ADC1Value[ADC_NUM];
extern volatile unsigned int ADC0IntDone, ADC1IntDone;
extern unsigned int adcChannelValues[8];

#endif /* end __ADC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
