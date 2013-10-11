/*****************************************************************************
 *   target.c:  Target C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/

#include "LPC214x.h"
#include "type.h"
#include "irq.h"
#include "target.h"

/*****************************************************************************
** Function name:		IRQ_Exception
**
** Descriptions:		interrupt exceptional handler , change it as needed
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void IRQ_Exception(void) __irq;
void IRQ_Exception(void) __irq  
{
    while(1);                   /*  change it to your code */
}

/*****************************************************************************
** Function name:		FIQ_Exception
**
** Descriptions:		Fast interrupt exceptional handler , change it as needed
**
** parameters:			None
** Returned value:		None
**
******************************************************************************/
void FIQ_Exception(void) __fiq;
void FIQ_Exception(void) __fiq 
{
    while(1);                   /* change it to your code */
}

/******************************************************************************
** Function name:		TargetInit
**
** Descriptions:		Initialize the target board; it is called in a necessary 
**				place, change it as needed
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void TargetInit(void)
{
    /* Add your codes here */
}

/******************************************************************************
** Function name:		TargetResetInit
**
** Descriptions:		Initialize the target board before running the main() 
**				function; User may change it as needed, but may not 
**				deleted it.
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
/* mthomas: all reset-initialisation is done in startup.S */
void TargetResetInit(void)
{
#ifdef __DEBUG_RAM    
    MEMMAP = 0x2;                   /* set remap register */
#endif

#ifdef __DEBUG_FLASH    
    MEMMAP = 0x1;                   /* set remap register */
#endif

#ifdef __IN_CHIP    
    MEMMAP = 0x1;                   /* set remap register */
#endif

    /* Set system timers for each component */
    PLLCON = 1;
#if (Fpclk / (Fcclk / 4)) == 1
    VPBDIV = 0;
#endif
#if (Fpclk / (Fcclk / 4)) == 2
    VPBDIV = 2;
#endif
#if (Fpclk / (Fcclk / 4)) == 4
    VPBDIV = 1;
#endif

#if (Fcco / Fcclk) == 2
    PLLCFG = ((Fcclk / Fosc) - 1) | (0 << 5);
#endif
#if (Fcco / Fcclk) == 4
    PLLCFG = ((Fcclk / Fosc) - 1) | (1 << 5);
#endif
#if (Fcco / Fcclk) == 8
    PLLCFG = ((Fcclk / Fosc) - 1) | (2 << 5);
#endif
#if (Fcco / Fcclk) == 16
    PLLCFG = ((Fcclk / Fosc) - 1) | (3 << 5);
#endif
    PLLFEED = 0xaa;
    PLLFEED = 0x55;
    while((PLLSTAT & (1 << 10)) == 0);
    PLLCON = 3;
    PLLFEED = 0xaa;
    PLLFEED = 0x55;
    
    /* Set memory accelerater module*/
    MAMCR = 0;
#if Fcclk < 20000000
    MAMTIM = 1;
#else
#if Fcclk < 40000000
    MAMTIM = 2;
#else
    MAMTIM = 3;
#endif
#endif
    MAMCR = 2;
    
    /* Add your codes here */
    return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
