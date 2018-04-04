/*

 SRAM Map for Atlantis firmware

 +------------+ 0x022A 0000H REMAP TO 0x0002 0000H
 |            |
 |            |
 |    40K     | Fixed for S1 Firmware and will become DATA_REGION for S2 after S2 is loaded by S1
 |            |
 |            |
 +------------+ 0x0229 6000H REMAP TO 0x0001 A000H
 |    ...     |
 |            |
 |            |
 |            |
 |            | CODE Region for S2 firmware
 |    120K    |
 |            |
 |            |
 |            |
 |    ...     |
 |------------| 0x0228 0000H REMAP TO 0x0000 0000H


 DDR Map for Atlantis firmware

 +------------+
 |            |
 |            |
 |            |
 |            |
 |            |
 |            |
 |            |
 |            |
 |     1M     | DATA REGION FOR S2 FIRMWARE
 |            |
 |            |
 |            |
 |            |
 |            |
 |            |
 |            |
 +------------+ 4000 0000H

*/

ENTRY(start)

/* Memory Region configuration */
MEMORY
{
	/* All parameters support to development phase */
	CODE_REGION  (rx)    : ORIGIN = 0x00000000      , LENGTH = 256K  /* SRAM 0x02280000 */
	DATA_REGION  (rwx)   : ORIGIN = 0x03400000      , LENGTH = 128K  /* SRAM 0x022A8000 */
	BTCM_REGION  (rwx)	 : ORIGIN = 0x03400000      , LENGTH = 128K  /* BTCM */
	ATCM_REGION  (rwx)	 : ORIGIN = 0x03300000      , LENGTH = 64K   /* ATCM */
}

SYS_STACK_SIZE        = 0x400;      /* Application stack size            */
SVC_STACK_SIZE        = 0x400;      /* SVC mode stack                    */
IRQ_STACK_SIZE        = 0x400;      /* IRQ mode stack                    */
FIQ_STACK_SIZE        = 0x400;      /* FRQ mode stack                    */
UND_STACK_SIZE        = 0x400;      /* SVC mode stack                    */
ABT_STACK_SIZE        = 0x400;      /* ABT mode stack                    */

USER_EXEC_BASE        = 0x00000000;  /* S1 Firmware loaded and runs from here */

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

	.atcm :
	{
		*(.atcm.text)
		*(.atcm.data)
		*(.atcm.rodata)
		*(.atcm.bss)
	} > ATCM_REGION

	.btcm :
	{
		*(.btcm.text)
		*(.btcm.data)
		*(.btcm.rodata)
		*(.btcm.bss)
	} > BTCM_REGION

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

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > DATA_REGION

	/*
	Set stack top to end of RAM, and stack limit move down by
	size of stack_dummy section
	*/
	__StackTop = ORIGIN(DATA_REGION) + LENGTH(DATA_REGION);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + bss + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __bss_end__, "exceed RAM size")
}
