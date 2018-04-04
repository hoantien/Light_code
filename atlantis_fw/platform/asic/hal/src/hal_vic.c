/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_vic.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-17-2016
 * @brief   This file contains all the functions prototype of the PrimeCell
 * Vectored Interrupt Controller (VIC).
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_vic.h"
#include "cortex_r4.h"
#include "assert.h"

#ifdef OS
#include "portmacro.h"
#endif

/* Private define ------------------------------------------------------------*/
#define VIC_WORD_SIZE	32

#define VIC_IRQ_SR		0x000 /* IRQ status register. */
#define VIC_FIQ_SR		0x004 /* FIQ status register. */
#define VIC_RAW_INT_SR	0x008 /* Raw interrupt status register */
#define VIC_INT_SR		0x00c /* Interrupt select register. */
#define VIC_INT_ER		0x010 /* Interrupt enable register. */
#define VIC_INT_ECR		0x014 /* Interrupt enable clear register. */
#define VIC_SW_INTR		0x018 /* Software interrupt register. */
#define VIC_SW_INT_CR	0x01c /* Software interrupt clear register. */
#define VIC_PER			0x020 /* Protection enable register. */
#define VIC_SW_PMR		0x024 /* Software priority mask register. */
#define VIC_PDR			0x028 /* Priority daisy-chain register. */
#define VIC_VA0R		0x100 /* Vector address 0 register. */
#define VIC_PA0R		0x200 /* Vector priority address 0 register. */
#define VIC_VAR			0xF00 /* Vector Address register. */

/* Macro use in assert param */
#define IS_VIC_IRQ_TYPE(TYPE)	((TYPE >= SPI_IRQn) && (TYPE < IRQn_MAX_IDX))
#define IS_VIC_IRQ_PRIO(PRIO)	(PRIO < 16)

/* Private typedef -----------------------------------------------------------*/
/* Private function_ ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static irq_handler vic_isr[IRQn_MAX_IDX];
static volatile uint8_t vic_init_flag;

/* Exported functions --------------------------------------------------------*/
void vic_init(void)
{
	unsigned int vic_base = VIC0_BASE;

	for(int i = 0; i < IRQn_MAX_IDX; i++)
	{
		if(i >= VIC_WORD_SIZE)
			vic_base = VIC1_BASE;
		writel(vic_base + VIC_VA0R + ((i%VIC_WORD_SIZE) << 2), i);
	}
}

void vic_enable_irq(irq_n_type_t n)
{
	unsigned int vic_base = VIC0_BASE;
	assert_param(IS_VIC_IRQ_TYPE(n));

	if(n >= VIC_WORD_SIZE)
	{
		vic_base = VIC1_BASE;
		n -= VIC_WORD_SIZE;
	}
	writel(vic_base + VIC_INT_ER, 0x1 << n);
}

void vic_disable_irq(irq_n_type_t n)
{
	unsigned int vic_base = VIC0_BASE;
	assert_param(IS_VIC_IRQ_TYPE(n));

	if(n >= VIC_WORD_SIZE)
	{
		vic_base = VIC1_BASE;
		n -= VIC_WORD_SIZE;
	}
	writel(vic_base + VIC_INT_ECR, 0x1 << n);
}

void vic_register_irq(irq_n_type_t n, irq_handler handler)
{
	assert_param(IS_VIC_IRQ_TYPE(n));
	assert_param(handler != NULL);

	if(vic_init_flag == 0)
	{
		vic_init();
		vic_init_flag = 1;
	}
	vic_isr[(uint32_t)n] = handler;
	vic_enable_irq(n);
}

void vic_unregister_irq(irq_n_type_t n)
{
	assert_param(IS_VIC_IRQ_TYPE(n));

	vic_disable_irq(n);
	vic_isr[(uint32_t)n] = NULL;
}

#ifdef OS
void vApplicationIRQHandler(void)
{
	unsigned int int_n;
	int_n = readl(VIC0_BASE + VIC_VAR);

	if(vic_isr[int_n] != 0)
	{
		portENABLE_INTERRUPTS();
		vic_isr[int_n]();
	}
	writel(VIC1_BASE + VIC_VAR, 0);
}
#else
void int_default_handler(void)
{
	unsigned int int_n;
	int_n = readl(VIC0_BASE + VIC_VAR);

	if(vic_isr[int_n] != 0)
	{
		vic_isr[int_n]();
	}
	writel(VIC1_BASE + VIC_VAR, 0);
	writel(VIC0_BASE + VIC_VAR, 0);
}
#endif

void vic_set_priority_irq(irq_n_type_t n, uint8_t prio)
{
	unsigned int vic_base = VIC0_BASE;
	assert_param(IS_VIC_IRQ_TYPE(n));
	assert_param(IS_VIC_IRQ_PRIO(prio));

	if(n >= VIC_WORD_SIZE)
	{
		vic_base = VIC1_BASE;
		n -= VIC_WORD_SIZE;
	}
	writel(vic_base + VIC_PA0R + (n << 2), (unsigned int)prio);
}

uint8_t vic_get_priority_irq(irq_n_type_t n)
{
	assert_param(IS_VIC_IRQ_TYPE(n));

	unsigned int vic_base = VIC0_BASE;
	if(n >= VIC_WORD_SIZE)
	{
		vic_base = VIC1_BASE;
		n -= VIC_WORD_SIZE;
	}
	return (uint8_t)readl(vic_base + VIC_PA0R + (n << 2));
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
