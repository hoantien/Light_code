/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    armv7_start.s
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-15-2016
 * @brief
 *
 ******************************************************************************/
 /* This program is allocated to section "initvector" */


	.text
	.code 32

	.extern reset_handler
	.extern undefined_handler
	.extern svc_handler
	.extern prefetch_handler
	.extern abort_handler
	.extern reserved_handler
	.extern irq_handler
	.extern fiq_handler
	.extern swi_handler

	.global start
	.func   start

start:
	ldr pc, =reset_handler                  /* Reset Vector */
	ldr pc, =undefined_handler
#ifdef OS
	ldr pc, =FreeRTOS_SVC_Handler
#else
	ldr pc, =swi_handler
#endif
	ldr pc, =prefetch_handler
	ldr pc, =abort_handler
	ldr pc, =reserved_handler
#ifdef OS
	ldr pc, =FreeRTOS_IRQ_Handler
#else
	ldr pc, =irq_handler
#endif
	ldr pc, =fiq_handler

code_start:
	.word    start                          /* pointer to the user application start address */
code_end:
	.word    end
code_execute:
	.word    execute                          /* execute address of first instruction */
	.string ".TheLightCo_CortexR4_ASICFwS2."  /* firmware validation signature */
	.align 4
	.end

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
