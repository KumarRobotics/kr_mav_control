/******************************************************************************
 *   irq.h:  Interrupt related Header file for Philips LPC214x Family 
 *   Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
 *   21. Apr. 2006 - modified for gnu/gcc by Martin Thomas
 ******************************************************************************/
#ifndef __IRQ_H 
#define __IRQ_H

// #define __irq __attribute__ ((interrupt("IRQ")))
// #define __fiq __attribute__ ((interrupt("FIQ")))
#define __irq
#define __fiq


/* if nested interrupt is used, NEST_INTERRUPT needs to be set to 1, otherwise 0 */
// mthomas : macros disabled for now 
// nesting is the default in the current arm-elf-gcc/WinARM-port
// see Startup.S function IRQ_Wrapper
//#define NESTED_INTERRUPT	1
#define NESTED_INTERRUPT	0

#define I_Bit			0x80
#define F_Bit			0x40

#define SYS32Mode		0x1F
#define IRQ32Mode		0x12
#define FIQ32Mode		0x11

#define	WDT_INT			0
#define SWI_INT			1
#define ARM_CORE0_INT		2
#define	ARM_CORE1_INT		3
#define	TIMER0_INT		4
#define TIMER1_INT		5
#define UART0_INT		6
#define	UART1_INT		7
#define	PWM0_INT		8
#define I2C0_INT		9
#define SPI0_INT		10
#define	SPI1_INT		11
#define	PLL_INT			12
#define RTC_INT			13
#define EINT0_INT		14
#define EINT1_INT		15
#define EINT2_INT		16
#define EINT3_INT		17
#define	ADC0_INT		18
#define I2C1_INT		19
#define BOD_INT			20
#define ADC1_INT		21
#define USB_INT			22

#define IRQ_SLOT_EN		0x20	/* bit 5 in Vector control register */
#define VIC_SIZE		16

#define VECT_ADDR_INDEX	0x100
#define VECT_CNTL_INDEX 0x200

/* Be aware that, from compiler to compiler, nested interrupt will have to
be handled differently. More details can be found in Philips LPC2000
family app-note AN10381 */
 
/* unlike Keil Compiler, don't save and restore registers into the stack
in RVD as the compiler does that for you. See RVD ARM compiler Inline and
embedded assemblers, "Rules for using __asm and asm keywords. */
// static DWORD sysreg;		/* used as LR register */

#if 0
#if NESTED_INTERRUPT 
#define IENABLE __asm { MRS LR, SPSR } \
				__asm { STMFD SP!, {LR} } \
				__asm { MSR CPSR_c, #SYS32Mode } \
				__asm { STMFD SP!, {LR} }
#else
#define IENABLE ;	/* do nothing */
#endif

#if NESTED_INTERRUPT
#define IDISABLE	__asm { LDMFD SP!, {LR} } \
					__asm { MSR CPSR_c, #(IRQ32Mode|I_Bit) } \
					__asm { LDMFD SP!, {LR} } \
					__asm { MSR SPSR_cxsf, LR }
#else
#define IDISABLE ;	/* do nothing */
#endif
#endif

#if NESTED_INTERRUPT 
#warning "Macros not tested successfully with arm-elf-gcc
#define IENABLE asm volatile( \
 "MRS     LR, SPSR \n"  \
 "STMFD   SP!, {LR} \n" \
 "MSR     CPSR_c, #0x1F \n" \
 "STMFD   SP!, {LR} " )
#else
#define IENABLE ;	/* do nothing */
#endif

#if NESTED_INTERRUPT
#define IDISABLE asm volatile(  \
 "LDMFD   SP!, {LR} \n" \
 "MSR     CPSR_c, #0x92 \n" \
 "LDMFD   SP!, {LR} \n" \
 "MSR     SPSR_cxsf, LR \n" )
#else
#define IDISABLE ;	/* do nothing */
#endif


void init_VIC( void );
unsigned long install_irq( unsigned long IntNumber, void *HandlerAddr );
unsigned long uninstall_irq( unsigned long IntNumber );



#endif /* end __IRQ_H */

/******************************************************************************
**                            End Of File
******************************************************************************/
