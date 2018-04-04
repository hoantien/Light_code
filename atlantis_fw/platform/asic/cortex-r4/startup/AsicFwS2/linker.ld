/*

 SRAM Map for Atlantis firmware

 +----------------+ DATA Region for S2 firmware
 |   STACK        |
 |----------------|
 |   HEAP         |
 |   (GROWS UP)   |
 |----------------|
 |   BSS          |
 |   SECTION      |
 |----------------|
 |   DATA         |
 |   SECTION      |
 +----------------+
 |   RODATA       |
 |   SECTION      |
 |----------------| CODE Region for S2 firmware
 |    TEXT        |
 |   SECTION      |
 |----------------|
 |      ISR       |
 | VECTOR SECTION |
 |----------------| 0x0228 0000H REMAP TO 0x0000 0000H


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
 +------------+ 0x0400 0000H (DDR start address, max size is 1GB)

*/

ENTRY(start)
GROUP(libgcc.a libg.a libc.a libm.a libstdc++.a libsupc++.a)
/* Memory Region configuration */
MEMORY
{
	/* All parameters support to development phase */
	CODE_REGION      (rx)    : ORIGIN = 0x00000000  , LENGTH = 205K /* SRAM 0x02280000 */
	DATA_REGION      (rwx)   : ORIGIN = 0x00033400  , LENGTH = 18K  /* SRAM 0x022A8000 */
	HEAP_REGION      (rwx)   : ORIGIN = 0x00037C00  , LENGTH = 33K  /* HEAP FREERTOS*/
	BTCM_REGION      (rwx)	 : ORIGIN = 0x03400000  , LENGTH = 128K /* BTCM */
	ATCM_REGION      (rw)	 : ORIGIN = 0x03300000  , LENGTH = 64K  /* ATCM */
    DDR_CODE_REGION  (rwx)   : ORIGIN = 0x3C000000  , LENGTH = 4M   /* DDR code - (@ 0x04000000 + 896MB, size 4MB) */
    DDR_HEAP_REGION  (rw)    : ORIGIN = 0x3C400000  , LENGTH = 124M /* DDR heap - (@ 0x04000000 + 900MB, size 124MB) */
}

/* _log2_ddr_code_region_size_ = log2(LENGTH(DDR_CODE_REGION) + LENGTH(DDR_HEAP_REGION)); */
_log2_ddr_code_region_size_ = 27;
_ddr_code_region_ = ORIGIN(DDR_CODE_REGION);

SYS_STACK_SIZE        = 0x200;      /* Application stack size            */
SVC_STACK_SIZE        = 0x200;      /* SVC mode stack                    */
IRQ_STACK_SIZE        = 0x200;      /* IRQ mode stack                    */
FIQ_STACK_SIZE        = 0x000;      /* FRQ mode stack                    */
UND_STACK_SIZE        = 0x000;      /* SVC mode stack                    */
ABT_STACK_SIZE        = 0x000;      /* ABT mode stack                    */
HEAP_STACK_SIZE       = 0x400;      /* HEAP SIZE*/
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

	.dram :
	{
		*(.dram.text)
		*(.dram.data)
		*(.dram.rodata)
		*(.dram.bss)

		*pb-c.o(.text* .data* .rodata* .bss*)
		*protobuf-c.o(.text* .rodata* .text* .bss*)
		*gyro.o(.text* .data* .rodata* .bss*)
		*tof.o(.text* .data* .rodata* .bss*)
		*light_header.o(.text* .data* .rodata* .bss*)
    } > DDR_CODE_REGION

	.text :
	{
		KEEP(*(.isr_vector))
		*(.text .text.* .gnu.linkonce.t.*)
		*(.glue_7t) *(.glue_7)
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		. = ALIGN(4);
		KEEP(*(.init))
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
		. = ALIGN(0x4);
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))
		. = ALIGN(4);
		KEEP(*(.fini))
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;

		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))
		. = ALIGN(4);
		*(.rodata*)
		KEEP(*(.eh_frame*))
	__etext = .;
	} > CODE_REGION
	/* .ARM.exidx is sorted, so has to go in its own output section.  */
	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx :
	{
	  *(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > CODE_REGION
	PROVIDE_HIDDEN (__exidx_end = .);
	. = ALIGN(4);
	__exidx_end = .;
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

	.data : AT (__exidx_end)
	{
		__data_start__ = .;
		_start_data_sram = .;
		. += _sram_data_size;
		PROVIDE(__dso_handle = .);
		__data_end__ = .;
	} > DATA_REGION
	. = ALIGN(4);
	.bss :
	{
		__bss_start__ = .;
		*(.bss)
		*(.bss.**)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
		end = .;
	} > DATA_REGION
	. = ALIGN(4);
	.heap (NOLOAD):
	{
		heap_start = .;
		.	= ALIGN(0x8);
		*(.heap_stack)
		. += HEAP_STACK_SIZE;
		heap_end = .;
	}> DATA_REGION

	.sys_stack (COPY):
	{
		sys_stack_start = .;
		.   = ALIGN(0x8);
		*(.sys_stack)
		. += SYS_STACK_SIZE;
		sys_stack_end = .;
	} > DATA_REGION

	.svc_stack sys_stack_end (COPY):
	{
		svc_stack_start = .;
		.   = ALIGN(0x8);
		*(.svc_stack)
		. += SVC_STACK_SIZE;
		svc_stack_end = .;
	} > DATA_REGION

	.irq_stack svc_stack_end (COPY):
	{
		irq_stack_start = .;
		.   = ALIGN(0x8);
		*(.irq_stack)
		. += IRQ_STACK_SIZE;
		irq_stack_end = .;
	} > DATA_REGION

	.fiq_stack irq_stack_end (COPY):
	{
		fiq_stack_start = .;
		.   = ALIGN(0x8);
		*(.fiq_stack)
		. += FIQ_STACK_SIZE;
		fiq_stack_end = .;
	} > DATA_REGION

	.und_stack fiq_stack_end (COPY):
	{
		und_stack_start = .;
		.   = ALIGN(0x8);
		*(.und_stack)
		. += UND_STACK_SIZE;
		und_stack_end = .;
	} > DATA_REGION

	.abt_stack und_stack_end (COPY):
	{
		abt_stack_start = .;
		.   = ALIGN(0x8);
		*(.abt_stack)
		. += ABT_STACK_SIZE;
		abt_stack_end = .;
	} > DATA_REGION

	. = ORIGIN(BTCM_REGION);
	.btcm :
	{
		*(.btcm.text)
		*(.btcm.data)
		*(.btcm.rodata)
		*(.btcm.bss)
	} > BTCM_REGION

	. = ORIGIN(HEAP_REGION);
	.os_heap_1 :
	{
		os_heap_start1 = ABSOLUTE(.);
		*(.os_heap_1)
		. += LENGTH(HEAP_REGION);
		os_heap_end1 = ABSOLUTE(.);
	} > HEAP_REGION

	. = ORIGIN(ATCM_REGION);
	.os_heap2 :
	{
		os_heap_start2 = ABSOLUTE(.);
		*(.os_heap_2)
		. += LENGTH(ATCM_REGION);
		os_heap_end2 = ABSOLUTE(.);
	} > ATCM_REGION

    . = ORIGIN(DDR_HEAP_REGION);
    .ddr_heap :
    {
        ddr_heap_start = ABSOLUTE(.);
        *(.ddr_heap)
        . += LENGTH(DDR_HEAP_REGION);
        ddr_heap_end = ABSOLUTE(.);
    } > DDR_HEAP_REGION
}