/*****************************************************************************
 *   adc.c:  ADC module file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
#include "LPC214x.h"                        /* LPC21xx definitions */
#include "type.h"
#include "irq.h"
#include "target.h"
#include "adc.h"

volatile unsigned int ADC0Value[ADC_NUM], ADC1Value[ADC_NUM];
volatile unsigned int ADC0IntDone = 0, ADC1IntDone = 0;
unsigned int adcChannelValues[8];

#if ADC_INTERRUPT_FLAG
/******************************************************************************
 ** Function name:		ADC0Handler
 **
 ** Descriptions:		ADC0 interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 ******************************************************************************/
void ADC0Handler (void) __irq
{
  unsigned int regVal;

  IENABLE;			/* handles nested interrupt */

  regVal = AD0STAT;		/* Read ADC will clear the interrupt */
  if ( regVal & 0x0000FF00 )	/* check OVERRUN error first */
  {
    regVal = (regVal & 0x0000FF00) >> 0x08;
    /* if overrun, just read ADDR to clear */
    /* regVal variable has been reused. */
    switch ( regVal )
    {
      case 0x01:
        regVal = AD0DR0;
        break;
      case 0x02:
        regVal = AD0DR1;
        break;
      case 0x04:
        regVal = AD0DR2;
        break;
      case 0x08:
        regVal = AD0DR3;
        break;
      case 0x10:
        regVal = AD0DR4;
        break;
      case 0x20:
        regVal = AD0DR5;
        break;
      case 0x40:
        regVal = AD0DR6;
        break;
      case 0x80:
        regVal = AD0DR7;
        break;
      default:
        break;
    }
    AD0CR &= 0xF8FFFFFF;	/* stop ADC now */
    ADC0IntDone = 1;
    return;
  }

  if ( regVal & ADC_ADINT )
  {
    switch ( regVal & 0xFF )	/* check DONE bit */
    {
      case 0x01:
        ADC0Value[0] = ( AD0DR0 >> 6 ) & 0x3FF;
        break;
      case 0x02:
        ADC0Value[1] = ( AD0DR1 >> 6 ) & 0x3FF;
        break;
      case 0x04:
        ADC0Value[2] = ( AD0DR2 >> 6 ) & 0x3FF;
        break;
      case 0x08:
        ADC0Value[3] = ( AD0DR3 >> 6 ) & 0x3FF;
        break;
      case 0x10:
        ADC0Value[4] = ( AD0DR4 >> 6 ) & 0x3FF;
        break;
      case 0x20:
        ADC0Value[5] = ( AD0DR5 >> 6 ) & 0x3FF;
        break;
      case 0x40:
        ADC0Value[6] = ( AD0DR6 >> 6 ) & 0x3FF;
        break;
      case 0x80:
        ADC0Value[7] = ( AD0DR7 >> 6 ) & 0x3FF;
        break;
      default:
        break;
    }
    AD0CR &= 0xF8FFFFFF;	/* stop ADC now */
    ADC0IntDone = 1;
  }

  IDISABLE;
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/******************************************************************************
 ** Function name:		ADC1Handler
 **
 ** Descriptions:		ADC1 interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 ******************************************************************************/
void ADC1Handler (void) __irq
{
  unsigned int regVal;

  IENABLE;			/* handles nested interrupt */

  regVal = AD1STAT;		/* Read ADC will clear the interrupt */
  if ( regVal & 0x0000FF00 )	/* check OVERRUN error first */
  {
    regVal = (regVal & 0x0000FF00) >> 0x08;
    /* if overrun, just read AD1DRx to clear */
    /* regVal variable has been reused. */
    switch ( regVal )
    {
      case 0x01:
        regVal = AD1DR0;
        break;
      case 0x02:
        regVal = AD1DR1;
        break;
      case 0x04:
        regVal = AD1DR2;
        break;
      case 0x08:
        regVal = AD1DR3;
        break;
      case 0x10:
        regVal = AD1DR4;
        break;
      case 0x20:
        regVal = AD1DR5;
        break;
      case 0x40:
        regVal = AD1DR6;
        break;
      case 0x80:
        regVal = AD1DR7;
        break;
      default:
        break;
    }
    AD1CR &= 0xF8FFFFFF;	/* stop ADC now */
    ADC1IntDone = 1;
    return;
  }

  if ( regVal & ADC_ADINT )
  {
    switch ( regVal & 0xFF )	/* check DONE bit */
    {
      case 0x01:
        ADC1Value[0] = ( AD1DR0 >> 6 ) & 0x3FF;
        break;
      case 0x02:
        ADC1Value[1] = ( AD1DR1 >> 6 ) & 0x3FF;
        break;
      case 0x04:
        ADC1Value[2] = ( AD1DR2 >> 6 ) & 0x3FF;
        break;
      case 0x08:
        ADC1Value[3] = ( AD1DR3 >> 6 ) & 0x3FF;
        break;
      case 0x10:
        ADC1Value[4] = ( AD1DR4 >> 6 ) & 0x3FF;
        break;
      case 0x20:
        ADC1Value[5] = ( AD1DR5 >> 6 ) & 0x3FF;
        break;
      case 0x40:
        ADC1Value[6] = ( AD1DR6 >> 6 ) & 0x3FF;
        break;
      case 0x80:
        ADC1Value[7] = ( AD1DR7 >> 6 ) & 0x3FF;
        break;
      default:
        break;
    }
    AD1CR &= 0xF8FFFFFF;	/* stop ADC now */
    ADC1IntDone = 1;
  }

  IDISABLE;
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}
#endif

/*****************************************************************************
 ** Function name:		ADCInit
 **
 ** Descriptions:		initialize ADC channel
 **
 ** parameters:			ADC clock rate
 ** Returned value:		true or false
 **
 *****************************************************************************/
unsigned int ADCInit( unsigned int ADC_Clk )
{
  AD0CR = ( 0x01 ) | 	// SEL=1,select channel 0, 1 to 4 on ADC0
    ( ( Fpclk / ADC_Clk - 1 ) << 8 ) |  // CLKDIV = Fpclk / 1000000 - 1
    ( 1 << 16 ) | 		// BURST = 0, no BURST, software controlled
    ( 0 << 17 ) |  		// CLKS = 0, 11 clocks/10 bits
    ( 1 << 21 ) |  		// PDN = 1, normal operation
    ( 0 << 22 ) |  		// TEST1:0 = 00
    ( 0 << 24 ) |  		// START = 0 A/D conversion stops
    ( 0 << 27 );		/* EDGE = 0 (CAP/MAT singal falling,trigger A/D
                       conversion) */


  /*
     AD0CR = ( 0x01 << 0 ) | 	// SEL=1,select channel 0, 1 to 4 on ADC0
     ( ( Fpclk / ADC_Clk - 1 ) << 8 ) |  // CLKDIV = Fpclk / 1000000 - 1
     ( 0 << 16 ) | 		// BURST = 0, no BURST, software controlled
     ( 0 << 17 ) |  		// CLKS = 0, 11 clocks/10 bits
     ( 1 << 21 ) |  		// PDN = 1, normal operation
     ( 0 << 22 ) |  		// TEST1:0 = 00
     ( 0 << 24 ) |  		// START = 0 A/D conversion stops
     ( 0 << 27 );		// EDGE = 0 (CAP/MAT singal falling,trigger A/D
     conversion) */

  AD1CR = ( 0x01 << 0 ) | 	// SEL=1,select channel 0, 0 to 7 on ADC1
    ( ( Fpclk / ADC_Clk - 1 ) << 8 ) |  // CLKDIV = Fpclk / 1000000 - 1
    ( 0 << 16 ) | 		// BURST = 0, no BURST, software controlled
    ( 0 << 17 ) |  		// CLKS = 0, 11 clocks/10 bits
    ( 1 << 21 ) |  		// PDN = 1, normal operation
    ( 0 << 22 ) |  		// TEST1:0 = 00
    ( 0 << 24 ) |  		// START = 0 A/D conversion stops
    ( 0 << 27 );		/* EDGE = 0 (CAP/MAT singal falling,trigger A/D
                       conversion) */

  /* If POLLING, no need to do the following */
#if ADC_INTERRUPT_FLAG
  AD0INTEN = 0x11E;		// Enable all interrupts
  AD1INTEN = 0x1FF;

  if ( install_irq( ADC0_INT, (void *)ADC0Handler ) == FALSE )
  {
    return (FALSE);
  }
  if ( install_irq( ADC1_INT, (void *)ADC1Handler ) == FALSE )
  {
    return (FALSE);
  }
#endif

  return (TRUE);
}

/*****************************************************************************
 ** Function name:		ADC0Read
 **
 ** Descriptions:		Read ADC0 channel
 **
 ** parameters:			Channel number
 ** Returned value:		Value read, if interrupt driven, return channel #
 **
 *****************************************************************************/
unsigned int ADC0Read( uint8_t channelNum )
{
#if !ADC_INTERRUPT_FLAG
  unsigned int regVal, ADC_Data;
  volatile unsigned int timeout=0;
#endif

  /* channel number is 0 through 7 */
  if ( channelNum >= ADC_NUM )
  {
    channelNum = 0;		/* reset channel number to 0 */
  }
  AD0CR &= 0xFFFFFF00;
  AD0CR |= (1 << 24) | (1 << channelNum);
  /* switch channel,start A/D convert */
#if !ADC_INTERRUPT_FLAG
  while ( timeout++ < 5000 )			/* wait until end of A/D convert */
  {
    regVal = *(volatile unsigned long *)(AD0_BASE_ADDR
        + ADC_OFFSET + ADC_INDEX * channelNum);
    /* read result of A/D conversion */
    if ( regVal & ADC_DONE )
    {
      break;
    }
  }

  AD0CR &= 0xF8FFFFFF;	/* stop ADC now */
  if ( regVal & ADC_OVERRUN )	/* save data when it's not overrun
                                 otherwise, return zero */
  {
    return ( 0 );
  }
  ADC_Data = ( regVal >> 6 ) & 0x3FF;
  return ( ADC_Data );	/* return A/D conversion value */
#else
  return ( channelNum );	/* if it's interrupt driven, the
                             ADC reading is done inside the handler.
                             so, return channel number */
#endif
}

/*****************************************************************************
 ** Function name:		ADC1Read
 **
 ** Descriptions:		Read ADC1 channel
 **
 ** parameters:			Channel number
 ** Returned value:		Value read, if interrupt driven, return channel #
 **
 *****************************************************************************/
unsigned int ADC1Read( uint8_t channelNum )
{
#if !ADC_INTERRUPT_FLAG
  unsigned int regVal;
  unsigned int ADC_Data;
#endif

  /* channel number is 0 through 7 */
  if ( channelNum >= ADC_NUM )
  {
    channelNum = 0;		/* reset channel number to 0 */
  }
  AD1CR &= 0xFFFFFF00;
  AD1CR |= (1 << 24) | (1 << channelNum);
  /* switch channel,start A/D convert */
#if !ADC_INTERRUPT_FLAG
  while ( 1 )			/* wait until end of A/D convert */
  {
    regVal = *(volatile unsigned long *)(AD1_BASE_ADDR
        + ADC_OFFSET + ADC_INDEX * channelNum);
    /* read result of A/D conversion */
    if ( regVal & ADC_DONE )
    {
      break;
    }
  }

  AD1CR &= 0xF8FFFFFF;	/* stop ADC now */
  if ( regVal & ADC_OVERRUN )	/* save data when it's not overrun
                                 otherwise, return zero */
  {
    return ( 0 );
  }

  ADC_Data = ( regVal >> 6 ) & 0x3FF;
  return ( ADC_Data );	/* return A/D conversion value */
#else
  return ( channelNum );
#endif
}

void ADC0triggerSampling(uint8_t selectChannels)
{
  AD0CR |= (selectChannels);

}

void ADC0getSamplingResults(uint8_t selectChannels, unsigned int * channelValues)
{
  int i;
  //get last result from all selected channels
  for (i=0;i<8;i++)
    if (selectChannels & (1<<i))
    {
      unsigned int regVal;

      regVal=*(volatile unsigned long *)(AD0_BASE_ADDR
          + ADC_OFFSET + ADC_INDEX * i);

      if ((regVal&(ADC_OVERRUN|ADC_DONE))==0)
        channelValues[i]=0;
      else
        channelValues[i]=( regVal >> 6 ) & 0x3FF;
    }
}

/*********************************************************************************
 **                            End Of File
 *********************************************************************************/
