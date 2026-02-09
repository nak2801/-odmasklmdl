/*******************************************************************************
 * Linker Script for FU1 Bootrom
 * 
 * Memory Layout:
 *   ROM    : 0x00000000 - 0x00001FFF (8KB Mask ROM)
 *   SRAM   : 0x20000000 - 0x2000FFFF (64KB)
 *   
 * Bootrom resides in ROM, application loaded to SRAM
 ******************************************************************************/

MEMORY
{
    ROM  (rx)  : ORIGIN = 0x00000000, LENGTH = 8K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Entry point */
ENTRY(Reset_Handler)

SECTIONS
{
    /* Vector table at start of ROM */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > ROM

    /* Code section */
    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
    } > ROM

    /* ARM exception handling */
    .ARM.extab :
    {
        *(.ARM.extab*)
    } > ROM

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > ROM

    /* Literal pool (constants) */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);
    } > ROM

    /* End of ROM */
    _etext = .;

    /* Initialized data section (if any) - copied from ROM to SRAM */
    .data : AT(_etext)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM

    /* Uninitialized data section */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    /* Stack at top of SRAM */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + 0x400;  /* 1KB stack */
        . = ALIGN(8);
        _estack = .;
    } > SRAM

    /* Heap after BSS */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = .;
        . = ORIGIN(SRAM) + LENGTH(SRAM) - SIZEOF(.stack);
        _eheap = .;
    } > SRAM

    /* Discard debug sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
    }
}

/* Provide stack top for vector table */
_stack_top = ORIGIN(SRAM) + LENGTH(SRAM);
PROVIDE(__stack = _stack_top);
