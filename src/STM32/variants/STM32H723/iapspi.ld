/*
* RRF IAP Memory configuration
* The IAP runs from RAM memory (so it can update the flash). We place it
* in the upper part of RAM above statically allocated items used by the
* firmware. This may overwrite dynamically allocated items but by the point
* we load the IAP this memory should no longer be in use.
*/

/* Entry Point */
ENTRY(Reset_Handler)

_Min_Heap_Size = 0x200;	/* required amount of heap  */
_Min_Stack_Size = 0x400;	/* required amount of stack */

/* Memories definition */
MEMORY
{
    DTCMRAM	(xrw)	: ORIGIN = 0x20000000,	LENGTH = 128K
    ITCMRAM	(xrw)	: ORIGIN = 0x00000000,	LENGTH = 64K
    /* We place the IAP the last 64K of "normal" RAM, note that we use the final 32K of RAM 
       as nocached file I/O buffers */
    RAM_D1	(xrw)	: ORIGIN = 0x24000000,	LENGTH = 320K - 32K - 64K
    CODE_RAM (xr): ORIGIN = 0x24038000,	LENGTH = 64K
    RAM_D2	(xrw)	: ORIGIN = 0x30000000,	LENGTH = 32K
    RAM_D3	(xrw)	: ORIGIN = 0x38000000,	LENGTH = 16K
}

/* Sections */
SECTIONS
{
  /* The startup code into RAM */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >CODE_RAM

  /* The program code and other data into RAM */
  .text :
  {
    __etext_start = .;
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >CODE_RAM

  /* Constant data into RAM */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    . = ALIGN(4);
  } >CODE_RAM

  .ARM.extab   : {
  	. = ALIGN(4);
  	*(.ARM.extab* .gnu.linkonce.armextab.*)
  	. = ALIGN(4);
  } >CODE_RAM

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >CODE_RAM

  .preinit_array     :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >CODE_RAM

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >CODE_RAM

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >CODE_RAM

  /* used by the startup to initialize data */
  _sidata = .;
  _sdata = .;        /* create a global symbol at data start */
  _edata = .;        /* define a global symbol at data end */
  _firmware_end = _sidata + (_edata - _sdata);

    /* Uninitialized data section into "RAM_D1" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* User_heap_stack section, used to check that there is enough "RAM_D1" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  .ram_nocache (NOLOAD) :
  {
    . = ALIGN(4);
    _szero_nocache = .;
    *(.CanMessage .CanMessage.*)
    *(.ram_nocache .ram_nocache.*)
    . = ALIGN(4);
    _ezero_nocache = .;
  } >RAM_D2

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
  PROVIDE(_nocache_ram_start = ORIGIN(RAM_D2));
  PROVIDE(_nocache_ram_end = ORIGIN(RAM_D2) + LENGTH(RAM_D2));
  /* Provide system stack location */
  __StackTop = ORIGIN(RAM_D1) + LENGTH(RAM_D1) - 64;	/* the 64 is to make room for the firmware filename */
  PROVIDE(_estack = __StackTop);
  PROVIDE(_firmware_crc = _firmware_end);
}
