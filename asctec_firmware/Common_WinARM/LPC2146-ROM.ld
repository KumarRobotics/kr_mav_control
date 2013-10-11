/***********************************************************************/
/*  ROM.ld:  Linker Script File                                        */
/***********************************************************************/

/* 
   Based on scripts found in the LPC yahoo-group and the Keil examples 
   modified by Martin Thomas, Kaiserslautern, Germany
   http://www.siwawi.arubi.uni-kl.de/avr_projects
*/

/* 
ENTRY(_boot)
STACK_SIZE = 0x800; -> see startup-file and .stack-section 
*/

/* Memory Definitions */
MEMORY
{
  ROM (rx) : ORIGIN = 0x00000000, LENGTH = 512k
  RAM (rw) : ORIGIN = 0x40000000, LENGTH = 32k
}

/* Section Definitions */
SECTIONS
{
  /* first section is .text which is used for code */
   .text :
  {
	/* *crt0.o (.text) */           /* Startup code */
	KEEP(*(.vectorg))
	. = ALIGN(4);
	KEEP(*(.init))             /* Startup code from .init-section */
	*(.text .text.*)           /* remaining code */
	*(.gnu.linkonce.t.*)
	*(.glue_7)
	*(.glue_7t)
	*(.gcc_except_table)
	*(.rodata)                 /* read-only data (constants) */
	*(.rodata*)
	*(.gnu.linkonce.r.*)
	. = ALIGN(4);
  } > ROM
  
  /***** old: 
  .text :
  {
    *crt0.o (.text)            
    *(.text)                   
    *(.rodata)                 
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
  } > ROM 
  *****/

  . = ALIGN(4);
  
	/* .ctors .dtors are used for c++ constructors/destructors */
	/* added by Martin Thomas 4/2005 based on Anglia Design example */
	.ctors :
	{
		PROVIDE(__ctors_start__ = .);
		KEEP(*(SORT(.ctors.*)))
		KEEP(*(.ctors))
		. = ALIGN(4);
		PROVIDE(__ctors_end__ = .);
		. = ALIGN(4);
	} >ROM
	
	.dtors :
	{
		PROVIDE(__dtors_start__ = .); 
		KEEP(*(SORT(.dtors.*)))
		KEEP(*(.dtors))
		. = ALIGN(4);
		PROVIDE(__dtors_end__ = .);
		. = ALIGN(4);
	} >ROM
  
    . = ALIGN(4);
	/* mthomas - end */
   
  _etext = . ;
  PROVIDE (etext = .);

  /* .data section which is used for initialized data */
  .data : AT (_etext)
  {
	_data = .;
	KEEP(*(.vectmapped)) /* mthomas: when code in ROM and vectors remapped */
	. = ALIGN(4);
	*(.fastrun) /* mthomas: for "RAMFUNC" */
	. = ALIGN(4);
	SORT(CONSTRUCTORS) /* mt 4/2005 */
	. = ALIGN(4);
	*(.data)
	*(.data.*)
	*(.gnu.linkonce.d*)
	. = ALIGN(4);
  } > RAM
 
  . = ALIGN(4);
  _edata = . ;
  PROVIDE (edata = .);

  /* .bss section which is used for uninitialized data */
  .bss (NOLOAD) :
  {
    __bss_start = . ;
    __bss_start__ = . ;
    *(.bss)
    *(.gnu.linkonce.b*)
    *(COMMON)
    . = ALIGN(4);
  } > RAM

  . = ALIGN(4);
  __bss_end__ = . ;
  PROVIDE (__bss_end = .);

  .stack ALIGN(256) :
  {
    *(.stack)
    PROVIDE (_stack = .);
    . = ALIGN(4);
  } > RAM

  _end = . ;
  PROVIDE (end = .);

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
