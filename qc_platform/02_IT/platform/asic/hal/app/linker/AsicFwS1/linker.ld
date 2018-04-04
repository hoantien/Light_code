/*

 ATCM Map for Atlantis firmware

 +------------+ 0330 FFFFH
 |            |
 |            | DATA Region for S1 firmware
 |    24K     |
 |            |
 |            |
 +------------+ 0330 A000H
 |    ...     |
 |            |
 |            | CODE Region to let S1 firmware
 |    40K     |
 |            |
 |    ...     |
 +------------+ 0330 0000H

*/

ENTRY(start)

/* Base Address Memory Table 64KByte on ATCM */
MEMORY
{
	CODE_REGION (rx)    : ORIGIN = 0x03300154, LENGTH = 51K
	DATA_REGION (rwx)   : ORIGIN = 0x03400000, LENGTH = 24K
}

SYS_STACK_SIZE        = 0x1500;      /* Application stack size            */
SVC_STACK_SIZE        = 0x200;      /* SVC mode stack                    */
IRQ_STACK_SIZE        = 0x200;      /* IRQ mode stack                    */
FIQ_STACK_SIZE        = 0x200;      /* FRQ mode stack                    */
UND_STACK_SIZE        = 0x200;      /* SVC mode stack                    */
ABT_STACK_SIZE        = 0x200;      /* ABT mode stack                    */
HEAP_STACK_SIZE       = 0x1000;     /* Heap stack size                   */

USER_EXEC_BASE        = 0x03300154;  /* S1 Firmware loaded and runs from here */

SECTIONS
{
	.reset USER_EXEC_BASE :
	{
		reset_start = .;
		execute = .;
		_intvec_start = .;
		*start.o (.text);
		.   = ALIGN(0x4);
		_intvec_end = .;
		end_reset = .;
	} > CODE_REGION

	.text :
	{
		KEEP(*(.isr_vector))
		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
		*crtbegin.o(.dtors)
		*crtbegin?.o(.dtors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
		*(SORT(.dtors.*))
		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
    	__etext = .;
	} > CODE_REGION

	.rodata :
	{
		_rodata_start = .;
		*(.rodata)
		*(.rodata.*)
		.   = ALIGN(0x8);
		_start_data_rom = .;
		*(.data)
		*(.data.*)
		_end_data_rom = .;
		*(.got.plt)
		*(.got)
		.   = ALIGN(0x8);
		_rodata_end = .;
		PROVIDE(end = .);
	} > CODE_REGION

	_sram_data_size = (_end_data_rom - _start_data_rom);

	.data : AT (ADDR(.text) + SIZEOF(.text))
	{
		__data_start__ = .;
		_start_data_sram = .;
		. += _sram_data_size;

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

		__data_end__ = .;

	} > DATA_REGION

	.bss __data_end__ :
	{
        __bss_start__ = .;
        *(.bss)
        *(.bss.**)
	 	*(COMMON)
        __bss_end__ = .;
        end = .;
    }

	.heap :
	{
		heap_start = .;
		.   = ALIGN(0x8);
		*(.heap_stack)
		. += HEAP_STACK_SIZE;
		heap_end = .;
	} > DATA_REGION

	.sys_stack __bss_end__ :
	{
		sys_stack_start = .;
		.   = ALIGN(0x8);
		*(.sys_stack)
		. += SYS_STACK_SIZE;
		sys_stack_end = .;
	} > DATA_REGION

	.svc_stack sys_stack_end :
	{
		svc_stack_start = .;
		.   = ALIGN(0x8);
		*(.svc_stack)
		. += SVC_STACK_SIZE;
		svc_stack_end = .;
	} > DATA_REGION

	.irq_stack svc_stack_end :
	{
		irq_stack_start = .;
		.   = ALIGN(0x8);
		*(.irq_stack)
		. += IRQ_STACK_SIZE;
		irq_stack_end = .;
	} > DATA_REGION

	.fiq_stack irq_stack_end :
	{
		fiq_stack_start = .;
		.   = ALIGN(0x8);
		*(.fiq_stack)
		. += FIQ_STACK_SIZE;
		fiq_stack_end = .;
	} > DATA_REGION

	.und_stack fiq_stack_end :
	{
		und_stack_start = .;
		.   = ALIGN(0x8);
		*(.und_stack)
		. += UND_STACK_SIZE;
		und_stack_end = .;
	} > DATA_REGION

	.abt_stack und_stack_end :
	{
		abt_stack_start = .;
		.   = ALIGN(0x8);
		*(.abt_stack)
		. += ABT_STACK_SIZE;
		abt_stack_end = .;
	} > DATA_REGION

	/* Remove information from the standard libraries */
	/DISCARD/ :
	{
		libgcc.a
		libg.a
		libc.a
		libm.a
		libnosys.a
	}
}
