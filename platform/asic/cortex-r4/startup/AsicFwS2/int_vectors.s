/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    int_vectors.s
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-09-2016
 * @brief
 *
 ******************************************************************************/
/* This program is allocated to section "intvec" */

	.text
	.code 32

	.global undefined_handler
	.global svc_handler
	.global prefetch_handler
	.global abort_handler
	.global reserved_handler
	.global irq_handler
	.global fiq_handler
	.global swi_handler

.macro save_user_regs
	sub	sp, sp, #72
	stmia sp, {r0 - r12}
	add	r8, sp, #60
	stmdb r8, {sp, lr}^
	str	lr, [r8, #0]
	mrs	r6, spsr
	str	r6, [r8, #4]
	str	r0, [r8, #8]
	mov	r0, sp
.endm

.macro restore_user_regs
	ldmia sp, {r0 - lr}^
	mov	r0, r0
	ldr	lr, [sp, #60]
	add	sp, sp, #72
	subs pc, lr, #4
.endm

undefined_handler:
	b  undefined_handler

svc_handler:
	b  svc_handler

prefetch_handler:
	b  prefetch_handler

#ifndef OS
swi_handler:
	b .
#endif

abort_handler:
	b  abort_handler

reserved_handler:
	b  reserved_handler

#ifndef OS
	.extern int_default_handler
irq_handler:
	save_user_regs
	bl  int_default_handler
	restore_user_regs
#endif

fiq_handler:
	b  fiq_handler

	.end

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
