/*****************************************************************************
*   swi_handler.s: SWI handler file ARM7TDMI-(S) (not just LPC2000)
*
*   by Martin Thomas 
*   http://www.siwawi.arubi.uni-kl.de/avr_projects
*   based on information from ARM documents
*
*   (I think there is nothing left from the Philips example-code beside
*   of the filename and some comments.)
*
*****************************************************************************/

.set SWI_IRQ_DIS,	0
.set SWI_IRQ_EN,	1
.set SWI_FIQ_DIS,	2
.set SWI_FIQ_EN,	3
.set SWI_GET_CPSR,	4
.set SWI_IRQ_REST,	5
.set SWI_FIQ_REST,	6

.set I_Bit, 0x80
.set F_Bit, 0x40
.set T_Bit, 0x20


#ifdef ROM_RUN
#ifdef VECTORS_IN_RAM
.set VECTREMAPPED, 1
#else
.set VECTREMAPPED, 0
#endif
#endif

#ifdef RAM_RUN
.set VECTREMAPPED, 0
#endif

  
/*********************************************************************
*  SWI interrupt handler                                             
*  Function :  SoftwareInterrupt(SWI_Number)                         
*              See below "SwiFunction" table                         
*  Parameters:    None                                               
*  input  :       SWI_Number (extracted from SWI command automaticly)
*  output :       states for some SWIs - see below
**********************************************************************/
.text
.arm

.if (VECTREMAPPED)
.print "SWI-Handler in section .vectmapped -> .data"
.section .vectmapped, "ax"
.else
.print "SWI-Handler in section .vectorg -> .code/.text"
.section .vectorg, "ax"
.endif

.global SoftwareInterrupt
.func   SoftwareInterrupt
SoftwareInterrupt:
SWI_HandlerMT:
	STMFD   sp!, {r4, lr}      /* store regs. */
	MRS     r4, spsr
	TST     r4, #T_Bit             /* test for thumb */
	LDRNEH  r4, [lr, #-2]          /* NE->thumb - get swi instruction code */
	BICNE   r4, r4, #0xff00        /* NE->thumb - clear top 8 bits leaving swi "comment field"=number */
	LDREQ   r4, [lr, #-4]          /* EQ->arm - get swi instruction code */
	BICEQ   r4, r4, #0xff000000    /* EQ->arm - clear top 8 bits leaving swi "comment field"=number */
	CMP     r4, #MAX_SWI           /* range-check */
	LDRLS   pc, [pc, r4, LSL #2]   /* jump to routine if <= MAX (LS) */
SWIOutOfRange:
	B       SWIOutOfRange

/* Jump-Table */
SwiTableStart:
	.word IRQDisable	// 0
	.word IRQEnable		// 1
	.word FIQDisable	// 2
	.word FIQEnable		// 3
	.word CPSRget		// 4
	.word IRQRestore	// 5
	.word FIQRestore	// 6
SwiTableEnd:
.set MAX_SWI, ((SwiTableEnd-SwiTableStart)/4)-1

IRQDisable:
	MRS     r0, SPSR                        /* Get SPSR = return value */
	ORR     r4, r0, #I_Bit                  /* I_Bit set */
	MSR     SPSR_c, r4                      /* Set SPSR */
	B       EndofSWI

IRQEnable:
	MRS     r0, SPSR                        /* Get SPSR = return value */
	BIC     r4, r0, #I_Bit                  /* I_Bit clear */
	MSR     SPSR_c, r4                      /* Set SPSR */
	B       EndofSWI                       

FIQDisable:
	MRS     r0, SPSR
	ORR     r4, r0, #F_Bit
	AND     r0, r0, #F_Bit
	MSR     SPSR_c, r4
	B       EndofSWI

FIQEnable:
	MRS     r0, SPSR
	BIC     r4, r0, #F_Bit
	AND     r0, r0, #F_Bit
	MSR     SPSR_c, r4
	B       EndofSWI

CPSRget:
	// LDR r0, =0xdeadbeef
	MRS     r0, SPSR                        /* Get SPSR */
	B       EndofSWI                       

IRQRestore:
	MRS     r4, SPSR                        /* Get SPSR */
	AND     r0, r0, #I_Bit
	TST     r0, #I_Bit             /* Test input for I_Bit */
	BICEQ   r4, r4, #I_Bit
	ORRNE   r4, r4, #I_Bit
	MSR     SPSR_c, r4
	B       EndofSWI

FIQRestore:
	MRS     r4, SPSR                        /* Get SPSR */
	AND     r0, r0, #F_Bit
	TST     r0, #F_Bit             /* Test input for F_Bit */
	BICEQ   r4, r4, #F_Bit
	ORRNE   r4, r4, #F_Bit
	MSR     SPSR_c, r4
	B       EndofSWI

EndofSWI:
	LDMFD   sp!, {r4,pc}^
.endfunc


/**********************************************************************
 *  Call SWI to restore IRQ
 *  Function : void IntEnable(uint32_t)
 *  Parameters:      None
 *  input  :         newstate
 *                   if I_bit in newstate cleared->IRQ on  -> clear I_BIT
 *                   if I_bit in newstate set    ->IRQ off -> set I_Bit
 *  output :         None
 **********************************************************************/
.arm
.text
.section .text, "ax"
.global IntRestore
.func   IntRestore
IntRestore:
		SWI     SWI_IRQ_REST
		BX      lr
.endfunc

/**********************************************************************
 *  Call SWI to restore FIQ
 *  Function : void IntEnable(uint32_t)
 *  Parameters:      None
 *  input  :         newstate
 *                   if F_bit in newstate cleared->FIQ on  -> clear F_BIT
 *                   if F_bit in newstate set    ->FIQ off -> set F_Bit
 *  output :         None
 **********************************************************************/
.arm
.text
.section .text, "ax"
.global FiqRestore
.func   FiqRestore
FiqRestore:
		SWI     SWI_FIQ_REST
		BX      lr
.endfunc

/**********************************************************************
 *  Call SWI to read IRQ/FIQ-status
 *  Function : uint32_t IntEnable(void)
 *  Parameters:      None
 *  input  :         None
 *  output :         CPSR (SPSR_SVC)
 **********************************************************************/
.arm
.text
.section .text, "ax"
.global IntGetCPSR
.func   IntGetCPSR
IntGetCPSR:
		SWI     SWI_GET_CPSR
		BX      lr
.endfunc

/**********************************************************************
 *  Call SWI to enable IRQ
 *  Function : uint32_t IntEnable(void)
 *  Parameters:      None
 *  input  :         None
 *  output :         previous status
 *                   I_Bit clear if IRQs were enabled
 *                   I_Bit set   if IRQs were disabled
 **********************************************************************/
.arm
.text
.section .text, "ax"
.global IntEnable
.func   IntEnable
IntEnable:
        SWI     SWI_IRQ_EN
        BX      lr
.endfunc /* end of IntEnable */

/**********************************************************************
 *  Call SWI to disable IRQ
 *  Function : uint32_t IntDisable(void)
 *  Parameters     : None
 *  input          : None
 *  output :         previous status
 *                   I_Bit clear if IRQs were enabled
 *                   I_Bit set   if IRQs were disabled
 **********************************************************************/
.arm
.global IntDisable
.section .text, "ax"
.func   IntDisable
IntDisable:
        SWI     SWI_IRQ_DIS
        BX      lr
.endfunc /* end of IntDisable */

/**********************************************************************
 *  Call SWI to enable FIQ
 *  Function : uint32_t FiqEnable(void)
 *  Parameters:      None
 *  input  :         None
 *  output :         previous status
 *                   F_Bit clear if FIQs were enabled
 *                   F_Bit set   if FIQs were disabled
 **********************************************************************/
.arm
.text
.section .text, "ax"
.global FiqEnable
.func   FiqEnable
FiqEnable:
        SWI     SWI_FIQ_EN
        BX      lr
.endfunc

/**********************************************************************
 *  Call SWI to disable FIQ
 *  Function : uint32_t FiqDisable(void)
 *  Parameters     : None
 *  input          : None
 *  output :         previous status
 *                   F_Bit clear if FIQs were enabled
 *                   F_Bit set   if FIQs were disabled
 **********************************************************************/
.arm
.global FiqDisable
.section .text, "ax"
.func   FiqDisable
FiqDisable:
        SWI     SWI_FIQ_DIS
        BX      lr
.endfunc


.end

/*************************************************************************
**                            End Of File
**************************************************************************/

