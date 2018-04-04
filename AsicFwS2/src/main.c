/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    main.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-28-2016
 * @brief   This file contains expand of the main application
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "timer.h"
#include "rtc.h"
#include "light_system.h"
#include "gen_heap.h"
#include "ddr_heap.h"
#include "hal_cache.h"


extern unsigned long os_heap_start1;
extern unsigned long os_heap_end1;
extern unsigned long os_heap_start2;
extern unsigned long os_heap_end2;
extern unsigned long ddr_heap_start;
extern unsigned long ddr_heap_end;

unsigned int total_os_heap_size;

static void  prv_initialise_heap(void);
extern void lpddr_test(void);
/* Private variables----------------------------------------------------------*/
static const char __LIGHT_LCC_TEXT[] = "\tLight Control Board\r\n";
#ifdef P2
static void config_cam_clk_24mhz(void)
{
	/*TODO: HAL SCU should config the 192MHz clock first */
	/* Enable cam clock */
	uint32_t clken = readl(SCU_BASE + 0x3C);
	/* Bit [21-16] = 0 to enable cam clock */
	clken &= ~(uint32_t)(BIT16 | BIT17 | BIT18 | BIT19 | BIT20 | BIT21);
	writel(SCU_BASE + 0x3C, clken);

	/* Select FLL output */
	/*writel(SCU_BASE + 0x24, 0x07); */
	/* Select cam clock */
	writel(SCU_BASE + 0x98, 0x3F);
	/* Set the divider clk = src_clk / (clk_div + 1)*/
	/* src_clk = 192 Mhz */
	/* clk = 24 Mhz */
	/* Divider for cam 0, 1, 2, 3 */
#ifndef PLL_25MHZ
	writel(SCU_BASE + 0x90, 0x07070707);
	/* Divider for cam 4, 5 */
	writel(SCU_BASE + 0x94, 0x00000707);
#endif
	/* Enable cam divider */
	writel(SCU_BASE + 0x74, 0x1);
	writel(SCU_BASE + 0x78, 0x1);
	writel(SCU_BASE + 0x7C, 0x1);
	writel(SCU_BASE + 0x80, 0x1);
	writel(SCU_BASE + 0x84, 0x1);
	writel(SCU_BASE + 0x88, 0x1);
}

static void wait_pll_lock(unsigned int pll_num)
{
	unsigned int get_value;
	unsigned int pll_lock;
	/*Wait PLL lock, and switch clock*/
	pll_lock = 0;
	do
	{
		get_value = readl(SCU_BASE + 0x228);
		pll_lock = get_value & (0x1 << pll_num);
	} while(pll_lock != (0x1 << pll_num));
}

void wait_cycles (unsigned int x)
{
	volatile int y;
	y = x;

	while (y)
	{
		y--;
	}
}
static void pll1_192m_configuration(void)
{
	/*
	M = 63
	N = 0
	P = 8
	PLL Fout = 27 * (M+1) / (N+1)/ (P+1) (FSE = 1)
	PLL Fout = 27 * (63+1) / (0+1) / (8+1) = 192 MHz. (FSE = 1)
	*/
#ifdef PLL_25MHZ
	writel(SCU_BASE+0xC, 0x12186301);
	writel(SCU_BASE+0x10, 0x00100035);
	wait_cycles(5);
	writel(SCU_BASE+0x10, 0x01101035);
#else
	writel(SCU_BASE+0xC, 0x12183F00);
	writel(SCU_BASE+0x10, 0x00100008);
	wait_cycles(5);
	writel(SCU_BASE + 0x10, 0x01101008);
	//
#endif
	wait_pll_lock(1);
	writel(SCU_BASE+0x24,0x7); // Select PLL FOUT or 24MHz
	uint32_t ip_debug_sel = readl(SCU_BASE + 0xA8);
	ip_debug_sel &= ~(0x00fe0000);
	writel(SCU_BASE + 0xA8, ip_debug_sel);
	wait_cycles(1000);
}
#endif


/* Main function--------------------------------------------------------------*/
int main(void)
{
	prv_initialise_heap();
#ifdef P2
	/* configure sensor clock */
	pll1_192m_configuration();
	config_cam_clk_24mhz();
#endif
	/* debuging lib init */
	log_init();
	/* init timer */
	timer_init();
	/* init rtc */
	rtc_init();

	/* Wellcom to LightLCC */
	printf(__LIGHT_LCC_TEXT);
	printf("ASIC%01d Firmware Ver %02d.%02d - Build id: %04d\r\n\r\n",
	        ASIC_NUM, ASIC_FW_VERSION_MAJOR, ASIC_FW_VERSION_MINOR, BUILD_ID);

	/* lpddr_test cannot be called as we are using DDR for code and data */
	//lpddr_test();

	/* Light.co system management */
	light_main();

	while (1);

	/* The program never reach here :D */
	return 0;
}


void task_testing(void *param)
{
	task_handle_t *hdl = (task_handle_t *)(param);
	uint8_t EXIT = 0;
	uint8_t i = 0;

	while(!EXIT)
	{
		if(TASK_READY == task_handler[task_query_tid("console")].state)
		{
			/* break out while loop */
			EXIT = 1;
		}
		vTaskDelay(1);
	}

	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	hdl->state = TASK_READY;

	for (;;)
	{
		vTaskDelay(5000);
		if(!i)
		{
			lpddr_test();
			i = 1;
		}
		vTaskDelay(1);
	}
}


static void  prv_initialise_heap(void)
{
	unsigned int addr_region_1 = (unsigned int)&os_heap_start1;
    unsigned int size_region_1 = (unsigned int)&os_heap_end1 - addr_region_1;
	unsigned int addr_region_2 = (unsigned int)&os_heap_start2;
    unsigned int size_region_2 = (unsigned int)&os_heap_end2 - addr_region_2;
	total_os_heap_size = size_region_1 + size_region_2;
	const HeapRegion_t heap_regions[] =
	{
		{(uint8_t *)addr_region_1,	size_region_1},
		{(uint8_t *)addr_region_2,	size_region_2},
		{NULL, 0 }
	};
	/* Fill zero two regions of os*/
	memset((uint8_t *)addr_region_1, 0x00, size_region_1);
	memset((uint8_t *)addr_region_2, 0x00, size_region_2);

    unsigned int addr_ddr_heap = (unsigned int)&ddr_heap_start;
    unsigned int size_ddr_heap = (unsigned int)&ddr_heap_end - addr_ddr_heap;

	const HeapRegion_t ddr_heap_regions[] =
	{
		/* Start address with dummy offsets		Size */
		{(uint8_t *)addr_ddr_heap,	size_ddr_heap},
		{ NULL, 0 }
	};

	/* Initialized the heap regions */
	gen_vPortDefineHeapRegions(&default_heap_attribute, heap_regions, portBYTE_ALIGNMENT);

	size_t min_ddr_heap_alignment = 16; /* 16 byte alignment needed for images */
	size_t cache_line_len = get_cache_line_len();
	/* Ensure alignment is a multiple of cache line length to simplify cache management for captured images */
    if (min_ddr_heap_alignment < cache_line_len)
        min_ddr_heap_alignment = cache_line_len;
	gen_vPortDefineHeapRegions(&ddr_heap_attribute, ddr_heap_regions, cache_line_len);
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
