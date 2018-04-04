/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    hal_wdt.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-18-2016
 * @brief   This file contains expand of hal_wdt
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "board_config.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "hal_wdt.h"

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief wdt_reg_t
 *
 * Watchdog register
 */
typedef struct wdt_reg
{
	__IO uint32_t WDT_CR;		/* Offset 0x00 Control register */
	__IO uint32_t WDT_TORR;		/* Offset 0x04 Timeout range register */
	__IO uint32_t WDT_CCVR;		/* Offset 0x08 Current counter value register */
	__IO uint32_t WDT_CRR;		/* Offset 0x0c Counter restart register */
	__IO uint32_t WDT_STAT;		/* Offset 0x10 Interrupt status register */
	__IO uint32_t WDT_EOI;		/* Offset 0x14 Interrupt clear register */
} wdt_reg_t;

/* Private definition --------------------------------------------------------*/
#define WDT_MAX_TOP				15

/* Registers */
#define WDT_EN			(uint32_t)(1<<0)		/* Enable WDT */
#define WDT_RPL			(uint32_t)(0x00000000)	/* 2 pclk cycles */
#define WDT_EN_IRQ		(uint32_t)(1<<1)		/* Enable IRQ WDT */
#define WDT_ALWAYS_EN	(uint32_t)(0x00000001)	/* Always enable WDT */
#define WDT_TORR_WIDTH	(uint32_t)(0x0000000f)	/* Timeout width */
#define WDT_RESTART		(uint32_t)(0x76)

/* Private function prototypes -----------------------------------------------*/
static void(*wdt_clb)(void);
static uint32_t wdt_top_convert(uint8_t top)
{
	uint32_t sec = 0;
	sec = ((1 << (16 + top)) - 1) / BOARD_PCLOCK;
	return sec;
}

/* Exported functions---------------------------------------------------------*/
wdt_status_t wdt_set_timeout(uint32_t period)
{
	uint8_t top_value = 0;
	uint8_t ret;
	wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	for(int i = 0; i <= WDT_MAX_TOP; i++)
	{
		if(wdt_top_convert(i) >= period)
		{
			top_value = i;
			break;
		}
	}
	if(!top_value)
	{
		ret = HAL_INVALID_PERIOD;
	}
	else
	{
		ret = HAL_WDT_OK;
	}
	/* Update timeout period. */
	wdtx->WDT_TORR = top_value & 0xF;
	return ret;
}

static void wdt_handler(void)
{
	register wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	/* Call a call back function to upper layer. */
	if(wdt_clb)
	{
		wdt_clb();
	}
	/* Clear the end of wdt interrupt. */
	wdtx->WDT_EOI;
}

void hal_wdt_init(uint32_t timeout, void(*clb_func)(void), uint8_t priority)
{
	wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	/* Enable APB clock for WDT via 0x34 bit [25:25] */
	*(volatile uint32_t*)(SCU_BASE + 0x34) &= 0xFDFFFFFF;
	__asm("isb");
	/* Reset WDT as nornal via 0x40 bit [25:25]*/
	*(volatile uint32_t*)(SCU_BASE + 0x40) &= 0xFDFFFFFF;
	__asm("isb");
	/* Allow watchdog timer for all IPs */
	writel(SCU_BASE + 0x48, 0xFFFFFFFF);
	writel(SCU_BASE + 0x4C, 0xFFFFFFFF);
	writel(SCU_BASE + 0x50, 0xFFFFFFFF);
	__asm("isb");

	/* Set Watchdog time out. */
	wdt_set_timeout(timeout);
	wdtx->WDT_CR	|=	WDT_RPL;
	wdtx->WDT_CR	|=	WDT_EN_IRQ;
	/* Initialize the interrupt line for WDT. */
	vic_register_irq(WDT_IRQn, wdt_handler);
	vic_set_priority_irq(WDT_IRQn, priority);
	/* Initialize a call back function. */
	if(clb_func)
	{
		wdt_clb = clb_func;
	}
}

void hal_wdt_start(void)
{
	wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	wdtx->WDT_CR |= WDT_EN ;	/* Enable Watchdog */
}

void hal_wdt_stop(void)
{
	wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	wdtx->WDT_CR &= ~WDT_EN;	/* Disable Watchdog */
}

void hal_wdt_kick_dog(void)
{
	wdt_reg_t *wdtx = (wdt_reg_t *)WDT_BASE;
	/* Restart the counter Watchdog. */
	wdtx->WDT_CRR = 	WDT_RESTART;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
