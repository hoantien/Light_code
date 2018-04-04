/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    reset_handler.s
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-15-2016
 * @brief
 *
 ******************************************************************************/
	.text
	.code 32

	.extern startup_main
	.global reset_handler
	.globl enable_irq
	.globl enable_fiq
	.globl disable_irq
	.globl disable_fiq
	.globl write_64bit_mem
	.globl read_64bit_mem
	.globl enable_TCM_RMW
	.globl disable_TCM_RMW

/****************************** reset_handler *************************************/
reset_handler:

	mrs r0, cpsr                /* Disalbe FIQ and IRQ */
	orr r0, r0, #0x000000C0
	msr cpsr, r0
	isb

	eor r0, r0, r0
	mov r1, r0
	mov r2, r0
	mov r3, r0
	mov r4, r0
	mov r5, r0
	mov r6, r0
	mov r7, r0
	mov r8, r0
	mov r9, r0
	mov r10, r0
	mov r11, r0
	mov r12, r0
	mov r13, r0
	mov r14, r0

stack_init:
	/* Stack setting */
	cps  #17  /* FIQ mode */
	ldr  sp, =fiq_stack_end
	cps  #18  /* IRQ mode */
	ldr  sp, =irq_stack_end
	cps  #23  /* Abort mode */
	ldr  sp, =abt_stack_end
	cps  #27  /* Undef mode */
	ldr  sp, =und_stack_end
	cps  #31  /* System mode */
	ldr  sp, =sys_stack_end
	cps  #19  /* SVC mode */
	ldr  sp, =svc_stack_end

	bl   cache_init
	bl   enable_irq
	b    startup_main

/***********************************************************************************************************************
* Function Name : cache_init
* Description   : Initialize I1, D1 cache and MPU settings
* Arguments     : none
* Return Value  : none
***********************************************************************************************************************/

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/

.equ	SCTLR_BR,        0x00020000
.equ	SCTLR_M,         0x00000001
.equ	SCTLR_I_C,       0x00001004

.equ	DRBAR_REGION_0,  0x00000000  			/* Base address = 0000_0000h */
.equ	DRACR_REGION_0,  0x00000000  			/* No Access */
.equ	DRSR_REGION_0,   0x0000003F  			/* Size 4G, MPU enable */

.equ	DRBAR_REGION_1,  0x04000000  			/* Base address = 0400_0000h */
.equ	DRACR_REGION_1,  0x00000308  			/* R/W(full), Normal, non-cacheable, non-shareable */
.equ	DRSR_REGION_1,   0x0000003D  			/* Size 2G, MPU enable */

.equ	DRBAR_REGION_2,  0x02000000  			/* Base address = 0200_0000h */
.equ	DRACR_REGION_2,  0x00001300  			/* R/W(full), XN, Device, non-share */
.equ	DRSR_REGION_2,   0x00000031  			/* Size 32MB, MPU enable */

.equ	DRBAR_REGION_3,  0x00000000  			/* Base address = 0000_0000h */
.equ	DRACR_REGION_3,  0x00000303  			/* R/W(full), Normal, write-back no allocate, non-shareable */
.equ	DRSR_REGION_3,   0x00000031  			/* Size 32MB, MPU enable */

.equ    DRBAR_REGION_4,  0x02280000  			/* Base address = 0228_0000h */
.equ    DRACR_REGION_4,  0x00000303  			/* R/W(full), Normal, write-back no allocate, non-shareable */
.equ    DRSR_REGION_4,   0x00000023  			/* Size 256KB, MPU enable */

/* BTCM */
.equ    DRBAR_REGION_5,  0x03400000  			/* Base address = 0340_0000h */
.equ    DRACR_REGION_5,  0x00000308  			/* R/W(full), Normal, non-cacheable, non-shareable */
.equ    DRSR_REGION_5,   0x00000021  			/* Size 128KB, MPU enable */

/* ATCM */
.equ    DRBAR_REGION_6,  0x03300000  			/* Base address = 0330_0000h */
.equ    DRACR_REGION_6,  0x00000308  			/* R/W(full), Normal, non-cacheable, non-shareable */
.equ    DRSR_REGION_6,   0x0000001F  			/* Size 64KB, MPU enable */

.equ	DRBAR_REGION_7,  0x02200000  			/* Base address = 0220_0000h */
.equ	DRACR_REGION_7,  0x00000603  			/* R only, Normal, write-back no allocate, non-shareable */
.equ	DRSR_REGION_7,   0x0000001D  			/* Size 32KB, MPU enable */

/* DDR */
.equ	DRBAR_REGION_8,  _ddr_code_region_  	/* Base address from the linker */
.equ	DRACR_REGION_8,  0x00000303  			/* R/W(full), Normal, write-back no allocate, cacheable, non-shareable */

cache_init:
	push  {lr}

cache_invalidate:
	/*Invalidate the I1, D1 cache */
	mov  r0, #0
	mcr  p15, #0, r0, c7, c5, #0   /*Invalidate all Instruction Caches (Write-value is Ignored) */
	isb                            /*Ensuring Context-changing */
	mcr  p15, #0, r0, c15, c5, #0  /*Invalidate all Data Caches (Write-value is Ignored) */
	isb                            /*Ensuring Context-changing */

	/*Adopt default memory map as background map. */
	ldr  r0, =SCTLR_BR           /*Set SCTLR.BR bit to 1 */
	mrc  p15, 0, r1, c1, c0, 0
	orr  r1, r1, r0
	dsb
	mcr  p15, 0, r1, c1, c0, 0
	isb                         /*Ensuring Context-changing */

	/*Initialize MPU settings (region 0 to 11) */
	/*Define region 0 */
	mov  r0,  #0
	ldr  r1, =DRBAR_REGION_0
	ldr  r2, =DRACR_REGION_0
	ldr  r3, =DRSR_REGION_0
	bl  mpu_init

	/*Define region 1 */
	mov  r0,  #1
	ldr  r1, =DRBAR_REGION_1
	ldr  r2, =DRACR_REGION_1
	ldr  r3, =DRSR_REGION_1
	bl  mpu_init

	/*Define region 2 */
	mov  r0,  #2
	ldr  r1, =DRBAR_REGION_2
	ldr  r2, =DRACR_REGION_2
	ldr  r3, =DRSR_REGION_2
	bl  mpu_init

	/*Define region 3 */
	mov  r0,  #3
	ldr  r1, =DRBAR_REGION_3
	ldr  r2, =DRACR_REGION_3
	ldr  r3, =DRSR_REGION_3
	bl  mpu_init

	/*Define region 4 */
	mov  r0,  #4
	ldr  r1, =DRBAR_REGION_4
	ldr  r2, =DRACR_REGION_4
	ldr  r3, =DRSR_REGION_4
	bl  mpu_init

	/*Define region 5 */
	mov  r0,  #5
	ldr  r1, =DRBAR_REGION_5
	ldr  r2, =DRACR_REGION_5
	ldr  r3, =DRSR_REGION_5
	bl  mpu_init

	/*Define region 6 */
	mov  r0,  #6
	ldr  r1, =DRBAR_REGION_6
	ldr  r2, =DRACR_REGION_6
	ldr  r3, =DRSR_REGION_6
	bl  mpu_init

	/*Define region 7 */
	mov  r0,  #7
	ldr  r1, =DRBAR_REGION_7
	ldr  r2, =DRACR_REGION_7
	ldr  r3, =DRSR_REGION_7
	bl  mpu_init

	/*Define region 8 */
	mov  r0,  #8
	ldr  r1, =DRBAR_REGION_8
	ldr  r2, =DRACR_REGION_8
	b    cal_ddr_code_region_size_
cal_ddr_code_region_size_ret:    
	bl  mpu_init

	/*Enables MPU operation */
	ldr  r0, =SCTLR_M            /*Set SCTLR.M bit to 1 */
	mrc  p15, 0, r1, c1, c0, 0
	orr  r1, r1, r0
	dsb
	mcr  p15, 0, r1, c1, c0, 0
	isb                         /*Ensuring Context-changing */

	/*Enables I1,D1 cache operation */
	ldr  r0, =SCTLR_I_C          /*Set SCTLR.I and C bit to 1 */
	mrc  p15, 0, r1, c1, c0, 0
	orr  r1, r1, r0
	dsb
	mcr  p15, 0, r1, c1, c0, 0
	isb                         /*Ensuring Context-changing */

	pop  {pc}
	bx  lr


cal_ddr_code_region_size_:
	ldr  r5, =_log2_ddr_code_region_size_
	sub  r5,#0x01
	lsl  r5,#0x01
	orr  r3,r5,#0x01
	b    cal_ddr_code_region_size_ret

/***********************************************************************************************************************
* Function Name : mpu_init
* Description   : Initialize MPU settings
* Arguments     : none
* Return Value  : none
***********************************************************************************************************************/
mpu_init:
	/*RGNR(MPU Memory Region Number Register) */
	mcr p15, #0, r0, c6, c2, #0
	isb                             /*Ensuring Context-changing */

	/*DRBAR(Data Region Base Address Register) */
	mcr  p15, #0, r1, c6, c1, #0
	isb                             /*Ensuring Context-changing */

	/*DRACR(Data Region Access Control Register) */
	mcr p15, #0, r2, c6, c1, #4
	isb                             /*Ensuring Context-changing */

	/*DRSR(Data Region Size and Enable Register) */
	mcr p15, #0, r3, c6, c1, #2
	isb                             /*Ensuring Context-changing */

	bx      lr


/***********************************************************************************************************************
* Function Name : set_low_vec
* Description   : Initialize sysytem by loader program
* Arguments     : none
* Return Value  : none
***********************************************************************************************************************/
set_low_vec:
	mrc  p15, 0, r0, c1, c0, 0  /*Set SCTLR.V bit to 1 (low-vector)*/
	and  r0, r0, #0xFFFFDFFF
	mcr  p15, 0, r0, c1, c0, 0
	isb                         /*Ensuring Context-changing*/
	bx  lr


enable_irq:
	mrs r0, cpsr
	and r0, r0, #0xFFFFFF7F
	msr cpsr, r0
	bx  lr

disable_irq:
	mrs r0, cpsr
	orr r0, r0, #0x00000080
	msr cpsr, r0
	bx  lr

enable_fiq:
	mrs r0, cpsr
	and r0, r0, #0xFFFFFFBF
	msr cpsr, r0
	bx  lr

disable_fiq:
	mrs r0, cpsr
	orr r0, r0, #0x00000040
	msr cpsr, r0
	bx  lr

enable_TCM_RMW:
	mrc  p15, 0, r0, c15, c0, 0
	orr  r0, r0, #3
	mcr  p15, 0, r0, c15, c0, 0
	isb
	bx  lr

disable_TCM_RMW:
	mrc  p15, 0, r0, c15, c0, 0
	and  r0, r0, #0xFFFFFFFC
	mcr  p15, 0, r0, c15, c0, 0
	bx  lr

write_64bit_mem:
	strd r0, [r2]
	dsb
	bx  lr

read_64bit_mem:
	ldrd r0, [r0]
	dsb
	bx  lr

	.end
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
