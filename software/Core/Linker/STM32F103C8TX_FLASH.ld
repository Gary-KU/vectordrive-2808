/* Reserve two pages for motor calibration and two for CAN node ID storage. */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 60K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 18K
  STACK (rw)  : ORIGIN = 0x20004800, LENGTH = 2K
}

_estack = ORIGIN(STACK) + LENGTH(STACK);
_Min_Stack_Size = 0x800;
_Min_Heap_Size = 0;
_stack_limit = ORIGIN(STACK);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text .text.*)
    *(.rodata .rodata.*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  .noinit (NOLOAD) :
  {
    . = ALIGN(8);
    *(.noinit .noinit.*)
    . = ALIGN(8);
    _end = .;
    PROVIDE(end = .);
  } > RAM

  .stack _stack_limit (NOLOAD) :
  {
    __StackLimit = .;
    . = . + _Min_Stack_Size;
    __StackTop = .;
  } > STACK

  /DISCARD/ : { *(.note.GNU-stack) *(.comment) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}

ASSERT(_end <= _stack_limit, "SRAM overflow: data/bss overlaps reserved 2 KiB stack")
ASSERT(__StackTop == _estack, "Stack placement mismatch")
