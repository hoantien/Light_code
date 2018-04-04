/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    jump.s
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-15-2016
 * @brief
 *
 ******************************************************************************/

#define	REMAP_REG_ADDR		0x02000028
#define	REMAP_SRAM		    0x00000002

	.syntax unified
	.arch   armv7-r

	.section .text

	.globl jump_to_SRAM
	.extern flush_dcache_all

jump_to_SRAM:

	/* disable FIQ and IRQ */
	mrs r0, cpsr
	orr r0, r0, #0x000000C0
	msr cpsr, r0
	isb

	/* clean and invalidate d-cache */
	bl	flush_dcache_all

	/* disable I/D cache */
	ldr  r0, =0xFFFFEFFB          /*Set SCTLR.I and C bit to 0 */
	mrc  p15, 0, r1, c1, c0, 0
	and  r1, r1, r0
	dsb
	mcr  p15, 0, r1, c1, c0, 0
	isb

	/* invalidate I/D cache */
	mov  r0, #0
	mcr  p15, #0, r0, c7, c5, #0   /* Invalidate all Instruction Caches (Write-value is Ignored) */
	isb
	mcr  p15, #0, r0, c15, c5, #0  /*Invalidate all Data Caches (Write-value is Ignored) */
	isb

	/* disable MPU */
	ldr  r0, =0xFFFFFFFE            /* Set SCTLR.M bit to 0 */
	mrc  p15, 0, r1, c1, c0, 0
	and  r1, r1, r0
	dsb
	mcr  p15, 0, r1, c1, c0, 0
	isb

	/* prepare registers */
	ldr  r4, =REMAP_REG_ADDR
	ldr  r5, =REMAP_SRAM
	ldr  r6, =0x00000000

	/* jump to SRAM to do the real switch and jump */
jump_code:
	/* r4: remap register address */
	/* r5: remap register value */
	/* r6: target address */
	str  r5, [r4]
	dsb
	isb
	mov  pc, r6

	.end

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
