ENTRY(Reset_Handler)

/* STM32F042K6
 * Flash: 32 KB  (0x0800_0000 .. 0x0800_7FFF)
 * SRAM : 6 KB   (0x2000_0000 .. 0x2000_17FF)
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
}

/* initial stack pointer = top of SRAM */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
    KEEP(*(.init))
    KEEP(*(.fini))
  } > FLASH

  /* .data lives in RAM but is loaded from FLASH */
  _sidata = LOADADDR(.data);
  .data :
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM AT>FLASH

  .bss :
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM

  /* Optional: helpful end-of-RAM symbol */
  _end = .;
}
