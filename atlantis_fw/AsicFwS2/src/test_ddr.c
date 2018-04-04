/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	test_lpddr.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	May-12-2016
 * @brief
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "os.h"
#include "board_config.h"
#include "log.h"
#include "test_ddr.h"

/* Privated define------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_LPDDR3

/* Privated functions prototypes ---------------------------------------------*/
/* Privated variables --------------------------------------------------------*/

static unsigned int compare_32(unsigned int addr, unsigned int pat)
{
	unsigned int ret = 0;
	unsigned int val = readl(addr);
	if (val != pat) {
		ret = 1;
	}
	return ret;
}

static unsigned int lpddr_test_address(void)
{
	static unsigned int addr = 0;
	unsigned int i, ret = 0;

//	if (addr == 0)
//		SLOGF(SLOG_INFO, "LPDDR TEST ADDRESS\r\n");
	while(addr <= 0x40000000)
	{
		for (i = 0 ; i < 0x100 ; i += 0x4) {
			writel(DDR_BASE + addr + i, addr + i);
		}

		for (i = 0 ; i < 0x100 ; i += 0x4) {
			ret |= compare_32(DDR_BASE + addr + i, addr + i);
			if(ret)
			{
//				SLOGF(SLOG_ERROR, "Address test failed at add 0x%08X", addr);
				printf("LPDDR address test failed at [0x%08x]\r\n", addr);
				return TEST_FAIL;
			}
		}
		addr += 0x1000;
	}
	if(!ret)
	{
//		SLOGF(SLOG_INFO, "Address test PASS");
		printf("LPDDR address test pass \r\n");
	}
	addr = 0;
	return TEST_PASS;
}

static unsigned int lpddr_test_pattern(void)
{
	static unsigned int addr = 0;
	unsigned int ret = 0;

//	if (addr == 0)
//		SLOGF(SLOG_INFO, "LPDDR TEST PATTERN\r\n");
	while(addr <= 0x40000000)
	{
		for (unsigned int i = 0 ; i < 0x100 ; i += 4) {
			writel(DDR_BASE + addr + i, (0xFEDCBA00 + i));}

		for (unsigned int i = 0 ; i < 0x100 ; i += 4) {
			ret |= compare_32(DDR_BASE + addr + i, (0xFEDCBA00 + i));
			if(ret)
			{
//				SLOGF(SLOG_ERROR, "Pattern test failed at add 0x%08X", addr);
				printf("LPDDR Pattern test failed at [0x%08x]\r\n", addr);
				return TEST_FAIL;
			}
		}
		addr += 0x1000;
	}
	if(!ret)
	{
//		SLOGF(SLOG_INFO, "Pattern test PASS");
		printf("LPDDR Pattern test pass \r\n");
	}
	addr = 0;
	return TEST_PASS;
}

/* Exported functions implementation -----------------------------------------*/
unsigned int lpddr_test(void)
{
	//static uint8_t tc = 0;
	unsigned int ret = TEST_PASS;
	ret = lpddr_test_address();
	if(TEST_FAIL == ret)
		return ret;
	ret = lpddr_test_pattern();
	return ret;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
