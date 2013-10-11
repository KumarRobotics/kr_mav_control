/*****************************************************************************
 *   irq.c: Interrupt handler C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "LPC214x.h"			/* LPC23XX Peripheral Registers	*/
#include "irq.h"

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif


/******************************************************************************
** Function name:		DefaultVICHandler
**
** Descriptions:		Default VIC interrupt handler.
**				This handler is set to deal with spurious 
**				interrupt.
**				If the IRQ service routine reads the VIC
**				address register, and no IRQ slot responses
**				as described above, this address is returned.
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
// mthomas: inserted static to avoid gcc-warning
static void DefaultVICHandler (void) __irq 
{
    /* if the IRQ is not installed into the VIC, and interrupt occurs, the
    default interrupt VIC address will be used. This could happen in a race 
    condition. For debugging, use this endless loop to trace back. */
    /* For more details, see Philips appnote AN10414 */
    VICVectAddr = 0;		/* Acknowledge Interrupt */ 
    while ( 1 );
}

/* Initialize the interrupt controller */
/******************************************************************************
** Function name:		init_VIC
**
** Descriptions:		Initialize VIC interrupt controller.
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void init_VIC(void) 
{
    unsigned long i = 0;
    unsigned long *vect_addr, *vect_cntl;
   	
    /* initialize VIC*/
    VICIntEnClr = 0xffffffff;
    VICVectAddr = 0;
    VICIntSelect = 0;

    /* set all the vector and vector control register to 0 */
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	vect_addr = (unsigned long *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (unsigned long *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	*vect_addr = 0;	
	*vect_cntl = 0;
    }

    /* Install the default VIC handler here */
    VICDefVectAddr = (unsigned long)DefaultVICHandler;   
    return;
}

/******************************************************************************
** Function name:		install_irq
**
** Descriptions:		Install interrupt handler
**				The max VIC size is 16, but, there are 32 interrupt
**				request inputs. Not all of them can be installed into
**				VIC table at the same time.
**				The order of the interrupt request installation is
**				first come first serve.
** parameters:			Interrupt number and interrupt handler address
** Returned value:		true or false, when the table is full, return false
** 
******************************************************************************/
unsigned long install_irq( unsigned long IntNumber, void *HandlerAddr )
{
    unsigned long i;
    unsigned long *vect_addr;
    unsigned long *vect_cntl;
      
    VICIntEnClr = 1 << IntNumber;	/* Disable Interrupt */
    
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	/* find first un-assigned VIC address for the handler */

	vect_addr = (unsigned long *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (unsigned long *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	if ( *vect_addr == (unsigned long)NULL )
	{
	    *vect_addr = (unsigned long)HandlerAddr;	/* set interrupt vector */
	    *vect_cntl = (unsigned long)(IRQ_SLOT_EN | IntNumber);
	    break;
	}
    }
    if ( i == VIC_SIZE )
    {
	return( FALSE );		/* fatal error, can't find empty vector slot */
    }
    VICIntEnable = 1 << IntNumber;	/* Enable Interrupt */
    return( TRUE );
}

/******************************************************************************
** Function name:		uninstall_irq
**
** Descriptions:		Uninstall interrupt handler
**				Find the interrupt handler installed in the VIC
**				based on the interrupt number, set the location
**				back to NULL to uninstall it.
** parameters:			Interrupt number
** Returned value:		true or false, when the interrupt number is not found, 
**				return false
** 
******************************************************************************/
unsigned long uninstall_irq( unsigned long IntNumber )
{
    unsigned long i;
    unsigned long *vect_addr;
    unsigned long *vect_cntl;
      
    VICIntEnClr = 1 << IntNumber;	/* Disable Interrupt */
    
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	/* find first un-assigned VIC address for the handler */
	vect_addr = (unsigned long *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (unsigned long *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	if ( (*vect_cntl & ~IRQ_SLOT_EN ) == IntNumber )
	{
	    *vect_addr = (unsigned long)NULL;	/* clear the VIC entry in the VIC table */
	    *vect_cntl &= ~IRQ_SLOT_EN;	/* disable SLOT_EN bit */	
	    break;
	}
    }
    if ( i == VIC_SIZE )
    {
	return( FALSE );		/* fatal error, can't find interrupt number 
					in vector slot */
    }
    VICIntEnable = 1 << IntNumber;	/* Enable Interrupt */
    return( TRUE );
}

/******************************************************************************
**                            End Of File
******************************************************************************/
