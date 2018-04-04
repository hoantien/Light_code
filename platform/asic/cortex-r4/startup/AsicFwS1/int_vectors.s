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

undefined_handler:
	b  undefined_handler

svc_handler:
	b  svc_handler

prefetch_handler:
	b  prefetch_handler

abort_handler:
	b  abort_handler

reserved_handler:
	b  reserved_handler

irq_handler:
	b  irq_handler

fiq_handler:
	b  fiq_handler

	.end

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
