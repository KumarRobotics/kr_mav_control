/***********************************************************************/
/*  This file is part of the uVision/ARM development tools             */
/*  Copyright KEIL ELEKTRONIK GmbH 2002-2005                           */
/***********************************************************************/
/*                                                                     */
/*  STARTUP.S:  Startup file for Philips LPC2000 device series         */
/*                                                                     */
/***********************************************************************/

/* 
   This file has been heavily modified for the GNU-Toolchain by:
   Martin Thomas, Kaiserslautern, Germany
   <mthomas@rhrk.uni-kl.de>
   http://www.siwawi.arubi.uni-kl.de/avr_projects
   
   If it does not work for you: don't blame Keil or Philips. 
*/

/* 
//*** <<< Use Configuration Wizard in Context Menu >>> *** 
*/


/*
 *  The STARTUP.S code is executed after CPU Reset. This file may be 
 *  translated with the following SET symbols. In uVision these SET 
 *  symbols are entered under Options - ASM - Set.
 *
 *  REMAP: when set the startup code initializes the register MEMMAP 
 *  which overwrites the settings of the CPU configuration pins. The 
 *  startup and interrupt vectors are remapped from:
 *     0x00000000  default setting (not remapped)
 *     0x80000000  when EXTMEM_MODE is used
 *     0x40000000  when RAM_MODE is used
 *
 *  EXTMEM_MODE: when set the device is configured for code execution
 *  from external memory starting at address 0x80000000. The startup
 *  vectors are located to 0x80000000.
 *
 *  RAM_MODE: when set the device is configured for code execution
 *  from on-chip RAM starting at address 0x40000000. The startup
 *  vectors are located to 0x40000000.
 */

/* Map Preprocessor definitions to assembler definitions/symbols */

.set EXTMEM_MODE, 0

#ifdef ROM_RUN
.set RAM_MODE, 0
#ifdef VECTORS_IN_RAM
.set REMAP, 1
.set VECTREMAPPED, 1
#else
.set REMAP, 0
.set VECTREMAPPED, 0
#endif
#endif

#ifdef RAM_RUN
.set RAM_MODE, 1
.set REMAP, 1
.set VECTREMAPPED, 0
#endif



.if (RAM_MODE)
.print "RAM_MODE enabled"
.else
.print "ROM_MODE enabled"
.endif

.if (REMAP)
.print "remapping enabled"
.endif

.if (VECTREMAPPED)
.print "Vectors at start of RAM"
.else
.print "Vectors at start of Code"
.endif


// Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs

        .set Mode_USR, 0x10
        .set Mode_FIQ, 0x11
        .set Mode_IRQ, 0x12
        .set Mode_SVC, 0x13
        .set Mode_ABT, 0x17
        .set Mode_UND, 0x1B
        .set Mode_SYS, 0x1F

        .set I_Bit, 0x80    /* when I bit is set, IRQ is disabled */
        .set F_Bit, 0x40    /* when F bit is set, FIQ is disabled */


/*
// <h> Stack Configuration (Stack Sizes in Bytes)
//   <o0> Undefined Mode      <0x0-0xFFFFFFFF:4>
//   <o1> Supervisor Mode     <0x0-0xFFFFFFFF:4>
//   <o2> Abort Mode          <0x0-0xFFFFFFFF:4>
//   <o3> Fast Interrupt Mode <0x0-0xFFFFFFFF:4>
//   <o4> Interrupt Mode      <0x0-0xFFFFFFFF:4>
//   <o5> User/System Mode    <0x0-0xFFFFFFFF:4>
// </h>
*/
        .set UND_Stack_Size, 0x00000080
        .set SVC_Stack_Size, 0x00000800
        .set ABT_Stack_Size, 0x00000080
        .set FIQ_Stack_Size, 0x00000080
        .set IRQ_Stack_Size, 0x00000800
        .set USR_Stack_Size, 0x00000800

#if 0
AREA   STACK, DATA, READWRITE, ALIGN=2 
        DS   (USR_Stack_Size+3)&~3  ; Stack for User/System Mode 
        DS   (SVC_Stack_Size+3)&~3  ; Stack for Supervisor Mode
        DS   (IRQ_Stack_Size+3)&~3  ; Stack for Interrupt Mode
        DS   (FIQ_Stack_Size+3)&~3  ; Stack for Fast Interrupt Mode 
        DS   (ABT_Stack_Size+3)&~3  ; Stack for Abort Mode
        DS   (UND_Stack_Size+3)&~3  ; Stack for Undefined Mode
#endif

.arm
.section .stack, "w"
.align 4
        .space (USR_Stack_Size+3)&~3  // Stack for User/System Mode 
        .space (SVC_Stack_Size+3)&~3  // Stack for Supervisor Mode
        .space (IRQ_Stack_Size+3)&~3  // Stack for Interrupt Mode
        .space (FIQ_Stack_Size+3)&~3  // Stack for Fast Interrupt Mode 
        .space (ABT_Stack_Size+3)&~3  // Stack for Abort Mode
        .space (UND_Stack_Size+3)&~3  // Stack for Undefined Mode
Top_Stack:


// VPBDIV definitions
        .set VPBDIV, 0xE01FC100  /* VPBDIV Address */

/*
// <e> VPBDIV Setup
// <i> Peripheral Bus Clock Rate
//   <o1.0..1>   VPBDIV: VPB Clock
//               <0=> VPB Clock = CPU Clock / 4
//               <1=> VPB Clock = CPU Clock
//               <2=> VPB Clock = CPU Clock / 2
//   <o1.4..5>   XCLKDIV: XCLK Pin
//               <0=> XCLK Pin = CPU Clock / 4
//               <1=> XCLK Pin = CPU Clock
//               <2=> XCLK Pin = CPU Clock / 2
// </e>
*/
        .set VPBDIV_SETUP, 1
        .set VPBDIV_Val, 0x00000000


// Phase Locked Loop (PLL) definitions
        .set PLL_BASE,      0xE01FC080  /* PLL Base Address */
        .set PLLCON_OFS,    0x00        /* PLL Control Offset*/
        .set PLLCFG_OFS,    0x04        /* PLL Configuration Offset */
        .set PLLSTAT_OFS,   0x08        /* PLL Status Offset */
        .set PLLFEED_OFS,   0x0C        /* PLL Feed Offset */
        .set PLLCON_PLLE,   (1<<0)      /* PLL Enable */
        .set PLLCON_PLLC,   (1<<1)      /* PLL Connect */
        .set PLLCFG_MSEL,   (0x1F<<0)   /* PLL Multiplier */
        .set PLLCFG_PSEL,   (0x03<<5)   /* PLL Divider */
        .set PLLSTAT_PLOCK, (1<<10)     /* PLL Lock Status */

/*
// <e> PLL Setup
// <i> Phase Locked Loop
// <i> CCLK - Processor Clock
// <i> Fcco - PLL Oscillator
//   <o1.0..4>   MSEL: PLL Multiplier Selection
//               <1-32><#-1>
//               <i> PLL Multiplier "M" Value
//               <i> CCLK = M * Fosc
//   <o1.5..6>   PSEL: PLL Divider Selection
//               <0=> 1   <1=> 2   <2=> 4   <3=> 8
//               <i> PLL Divider "P" Value
//               <i> Fcco = CCLK * 2 * P
//               <i> 156MHz <= Fcco <= 320MHz
// </e>
*/
        .set PLL_SETUP,  1
        .set PLLCFG_Val, 0x00000024


// Memory Accelerator Module (MAM) definitions
        .set MAM_BASE,   0xE01FC000  /* MAM Base Address */
        .set MAMCR_OFS,  0x00        /* MAM Control Offset*/
        .set MAMTIM_OFS, 0x04        /* MAM Timing Offset */

/*
// <e> MAM Setup
// <i> Memory Accelerator Module
//   <o1.0..1>   MAM Control
//               <0=> Disabled
//               <1=> Partially Enabled
//               <2=> Fully Enabled
//               <i> Mode
//   <o2.0..2>   MAM Timing
//               <0=> Reserved  <1=> 1   <2=> 2   <3=> 3
//               <4=> 4         <5=> 5   <6=> 6   <7=> 7
//               <i> Fetch Cycles
// </e>
*/
        .set MAM_SETUP,    1
        .set MAMCR_Val,    0x00000002
        .set MAMTIM_Val,   0x00000004


// Starupt Code must be linked first at Address at which it expects to run.

.if     (EXTMEM_MODE)
        .set CODE_BASE,  0x80000000
.elseif (RAM_MODE)
        .set CODE_BASE,  0x40000000
.else
        .set CODE_BASE,  0x00000000
.endif

#if 0
AREA   STARTUPCODE, CODE, AT CODE_BASE   // READONLY, ALIGN=4
       PUBLIC  __startup

       EXTERN  CODE32 (?C?INIT)

__startup       PROC    CODE32

// Pre-defined interrupt handlers that may be directly 
// overwritten by C interrupt functions
EXTERN CODE32 (Undef_Handler?A)
EXTERN CODE32 (SWI_Handler?A)
EXTERN CODE32 (PAbt_Handler?A)
EXTERN CODE32 (DAbt_Handler?A)
EXTERN CODE32 (IRQ_Handler?A)
EXTERN CODE32 (FIQ_Handler?A)
#endif

.text
.arm

.if (VECTREMAPPED)
.print "Vectors in section .vectmapped -> .data"
.section .vectmapped, "ax"
.else
.print "Vectors in section .vectorg -> .text"
.section .vectorg, "ax"
.endif

// Pre-defined interrupt handlers that may be directly 
// overwritten by C interrupt functions
.extern Undef_Handler
.extern SWI_Handler
.extern PAbt_Handler
.extern DAbt_Handler
.extern IRQ_Handler
.extern FIQ_Handler


// Exception Vectors
// Mapped to Address 0.
// Absolute addressing mode must be used.

__Vectors:        LDR     PC,Reset_Addr         
                LDR     PC,Undef_Addr
                LDR     PC,SWI_Addr
                LDR     PC,PAbt_Addr
                LDR     PC,DAbt_Addr
                NOP                            /* Reserved Vector */
//                LDR     PC,IRQ_Addr
//                LDR     PC,[PC, #-0x0FF0]      /* Vector from VicVectAddr */
                LDR     PC,IRQ_Wrapper_Addr
                LDR     PC,FIQ_Addr

Reset_Addr:       .word     Reset_Handler
Undef_Addr:       .word     Undef_Handler
// SWI_Addr:         .word     SWI_Handler
// SWI_Wrapper_Addr: .word     SWI_Wrapper
SWI_Addr:         .word     SoftwareInterrupt      /* in swi_handler.S */
PAbt_Addr:        .word     PAbt_Handler
DAbt_Addr:        .word     DAbt_Handler
                  .word     0                      /* Reserved Address */
// IRQ_Addr:         .word     __IRQ_Handler
IRQ_Wrapper_Addr: .word    __IRQ_Wrapper
FIQ_Addr:         .word     FIQ_Handler

Undef_Handler:  B       Undef_Handler
/* SWI_Handler:    B       SWI_Handler */
PAbt_Handler:   B       PAbt_Handler
DAbt_Handler:   B       DAbt_Handler
/* IRQ_Handler:    B       IRQ_Handler */
FIQ_Handler:    B       FIQ_Handler

.size   __Vectors, . - __Vectors



.arm
.section .init, "ax"

.if (VECTREMAPPED)
/* mthomas: Dummy used during startup - mind the nops since the 
   flash-utility will overwrite the "reserved vector"-address
   with the checksum */
				B Reset_Handler
				NOP
				NOP
				NOP
				NOP
				NOP  /* Reserved Address */
				NOP
				NOP
.endif

.arm
.section .init, "ax"
.global __startup
.func __startup
__startup:

Reset_Handler:  

.if (VPBDIV_SETUP)
                LDR     R0, =VPBDIV
                LDR     R1, =VPBDIV_Val
                STR     R1, [R0]
.endif


.if (PLL_SETUP)
                LDR     R0, =PLL_BASE
                MOV     R1, #0xAA
                MOV     R2, #0x55

// Configure and Enable PLL
                MOV     R3, #PLLCFG_Val
                STR     R3, [R0, #PLLCFG_OFS] 
                MOV     R3, #PLLCON_PLLE
                STR     R3, [R0, #PLLCON_OFS]
                STR     R1, [R0, #PLLFEED_OFS]
                STR     R2, [R0, #PLLFEED_OFS]

// Wait until PLL Locked
PLL_Loop:       LDR     R3, [R0, #PLLSTAT_OFS]
                ANDS    R3, R3, #PLLSTAT_PLOCK
                BEQ     PLL_Loop

// Switch to PLL Clock
                MOV     R3, #(PLLCON_PLLE | PLLCON_PLLC)
                STR     R3, [R0, #PLLCON_OFS]
                STR     R1, [R0, #PLLFEED_OFS]
                STR     R2, [R0, #PLLFEED_OFS]
.endif


.if (MAM_SETUP)
                LDR     R0, =MAM_BASE
                MOV     R1, #MAMTIM_Val
                STR     R1, [R0, #MAMTIM_OFS] 
                MOV     R1, #MAMCR_Val
                STR     R1, [R0, #MAMCR_OFS] 
.endif


// Memory Mapping
                .set MEMMAP, 0xE01FC040  /* Memory Mapping Control */

.if (REMAP)
                LDR     R0, =MEMMAP
.if     (EXTMEM_MODE)                
                MOV     R1, #3
.elseif (RAM_MODE) || (VECTREMAPPED)
.print "MEMMAP to 2 on init"
                MOV     R1, #2
.else
                MOV     R1, #1
.endif
                STR     R1, [R0]
.endif

// Setup Stack for each mode
                LDR     R0, =Top_Stack

// Enter Undefined Instruction Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_UND|I_Bit|F_Bit
                MOV     SP, R0
                SUB     R0, R0, #UND_Stack_Size

// Enter Abort Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_ABT|I_Bit|F_Bit
                MOV     SP, R0
                SUB     R0, R0, #ABT_Stack_Size

// Enter FIQ Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_FIQ|I_Bit|F_Bit
                MOV     SP, R0
                SUB     R0, R0, #FIQ_Stack_Size

// Enter IRQ Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_IRQ|I_Bit|F_Bit
                MOV     SP, R0
                SUB     R0, R0, #IRQ_Stack_Size

// Enter Supervisor Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_SVC|I_Bit|F_Bit
                MOV     SP, R0
                SUB     R0, R0, #SVC_Stack_Size

// Enter User Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_USR /* Interrupts enabled */
//				MSR     CPSR_c, #Mode_USR|I_Bit|F_Bit /* Interrupts disabled */
                MOV     SP, R0


.if (RAM_MODE==0)
/* Relocate .data section (Copy from ROM to RAM) */
                LDR     R1, =_etext 
                LDR     R2, =_data 
                LDR     R3, =_edata 
                CMP     R2, R3
                BEQ     DataIsEmpty
LoopRel:        CMP     R2, R3 
                LDRLO   R0, [R1], #4 
                STRLO   R0, [R2], #4 
                BLO     LoopRel 
DataIsEmpty:
.endif
 
/* Clear .bss section (Zero init) */
                MOV     R0, #0 
                LDR     R1, =__bss_start__ 
                LDR     R2, =__bss_end__ 
                CMP     R1,R2
                BEQ     BSSIsEmpty
LoopZI:         CMP     R1, R2 
                STRLO   R0, [R1], #4 
                BLO     LoopZI 
BSSIsEmpty:


// call C++ constructors of global objects
		LDR 	r0, =__ctors_start__
		LDR 	r1, =__ctors_end__
ctor_loop:
		CMP 	r0, r1
		BEQ 	ctor_end
		LDR 	r2, [r0], #4
		STMFD 	sp!, {r0-r1}
		MOV 	lr, pc
		MOV 	pc, r2
		LDMFD 	sp!, {r0-r1}
		B 		ctor_loop
ctor_end:

// Enter the C code
                //LDR     R0,=INIT
                LDR     R0,=main
                TST     R0,#1             // Bit-0 set: main is Thumb
                LDREQ   LR,=__exit_ARM    // ARM Mode
                LDRNE   LR,=__exit_THUMB  // Thumb Mode
                BX      R0

.size   __startup, . - __startup
.endfunc

.arm
.global __exit_ARM
.func __exit_ARM
__exit_ARM:
                B       __exit_ARM
.size   __exit_ARM, . - __exit_ARM
.endfunc

.thumb
.global __exit_THUMB
.func __exit_THUMB
__exit_THUMB:
                B       __exit_THUMB
.size   __exit_THUMB, . - __exit_THUMB
.endfunc


/* mthomas: the following code is inspired by various examples and
   documents from ARM, Atmel, Anglia Designs and others */


.text
.arm

.if (VECTREMAPPED)
.print "Handlers in section .vectmapped -> .data"
.section .vectmapped, "ax"
.else
.print "Handlers in section .vectorg -> .code/.text"
.section .vectorg, "ax"
.endif

.set VIC_base_addr, 0xFFFFF000
.set VIC_vect_offs, 0x30

        .arm
        .global __IRQ_Wrapper
        .func   __IRQ_Wrapper
__IRQ_Wrapper:
/*- Manage Exception Entry  */
/*- Adjust and save LR_irq in IRQ stack  */
            sub         lr, lr, #4
            stmfd       sp!, {lr}

/*- Save SPSR need to be saved for nested interrupt */
            mrs         r14, SPSR
            stmfd       sp!, {r14}

/*- Save and r0 in IRQ stack  */
            stmfd       sp!, {r0}

/*- Write in the IVR to support Protect Mode  */
/*- No effect in Normal Mode  */
/*- De-assert the NIRQ and clear the source in Protect Mode */
/* R14 = LR */
            ldr         r14, =VIC_base_addr
            ldr         r0 , [r14, #VIC_vect_offs]
            /*str         r14, [r14, #VIC_vect_offs]*/

/*- Enable Interrupt and Switch in Supervisor Mode */
            msr         CPSR_c, #I_Bit | Mode_SVC

/*- Save scratch/used registers and LR in User Stack */
            /*stmfd       sp!, { r1-r3, r12, r14}*/
            stmfd       sp!, { r1-r12, r14 }

/*- Branch to the routine pointed by the VIC-Vector-Address  */
            mov         r14, pc
            bx          r0
/*- Restore scratch/used registers and LR from User Stack*/
            /* ldmia       sp!, { r1-r3, r12, r14} */
            ldmia       sp!, { r1-r12, r14 }

/*- Disable Interrupt and switch back in IRQ mode */
            msr         CPSR_c, #I_Bit | Mode_IRQ

#if 0
/* VICVectAddr=0 is already done in the ISRs of the Philips-Examples 
   so commented out here */
/*- Mark the End of Interrupt on the VIC */
            ldr         r14, =VIC_base_addr
            str         r14, [r14, #VIC_vect_offs]
#endif

/*- Restore SPSR_irq and r0 from IRQ stack */
            ldmia       sp!, {r0}

/*- Restore SPSR_irq and r0 from IRQ stack */
            ldmia       sp!, {r14}
            msr         SPSR_cxsf, r14

/*- Restore adjusted  LR_irq from IRQ stack directly in the PC */
            ldmia       sp!, {pc}^

.size   __IRQ_Wrapper, . - __IRQ_Wrapper
.endfunc


#if 0
/* mthomas:
   Wrapper to call a C swi-Function declared with 
   void SWI_Handler(int swi_num, int *regs)
   Inspired by Anglia Designs example 
   -- not used here - see swi_handler.S
*/
        .arm
        .global __SWI_Wrapper
        .func   __SWI_Wrapper
__SWI_Wrapper:                       /* r0 holds swi number */
        STMFD   sp!,{r0-r12,lr}    /* Save The workspace plus the current return */
                                   /* address lr_ mode into the stack */
        MRS     r1, spsr           /* Save the spsr_mode into r1 */
        STMFD   sp!, {r1}          /* Save spsr */
        MOV     r1, sp             /* load regs */
        LDR     r0,=SWI_Handler    
        MOV     lr, pc
        BX      r0                 /* call the C-funcktion */
        LDMFD   sp!, {r1}          /* Restore the saved spsr_mode into r1 */
        MSR     spsr_cxsf, r1      /* Restore spsr_mode */
        LDMFD   sp!, {r0-r12,pc}   /* Return to the instruction following */
                                   /* the exception interrupt */
        .size   __SWI_Wrapper, . - __SWI_Wrapper
        .endfunc
#endif

#if 0
/* mthomas: not used here - reminder for future tests */
		.arm
		.global __IRQ_Wrapper
		.func __IRQ_Wrapper
__IRQ_Wrapper:
		SUB		lr, lr, #4				/* Update the link register */
		STMFD	sp!,{r0-r12,lr}	        /* Save The workspace plus the current return */
										/* address lr_ mode into the stack */
		MRS		r1, spsr				/* Save the spsr_mode into r1 */
		STMFD	sp!, {r1}				/* Save spsr */
		LDR		lr, =ReturnAddress		/* Read the return address. */
		LDR		r0, =VIC_base_addr      /* Load VIC Base-Address */
		LDR		r1, [r0, #VIC_vect_offs] /* Load ISR-Address from VICVectAddr */
		bx      r1                      /* Branch to the IRQ handler. */
ReturnAddress:
		LDR     r2, =VIC_base_addr      /* clear Interrupt */
		MOV     r3, #0
		STR     R3, [R2, #VIC_vect_offs] /* by writing to VICVectAddr */
		LDMFD	sp!, {r1}				/* Restore the saved spsr_mode into r1 */
		MSR		spsr_cxsf, r1			/* Restore spsr_mode */
		LDMFD	sp!, {r0-r12,pc}^	    /* Return to the instruction following */
										/* the exception interrupt */
.size   __IRQ_Wrapper, . - __IRQ_Wrapper
.endfunc
#endif

.end

