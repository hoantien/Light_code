/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_ddr.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    2-Aug-2016
 * @brief   Test application for HAL DDR module
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	2-Aug-2016	Initial revision:
 * * 1.0.1 10-Aug-2016  Implement test cases LPDDR3
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_hal_ddr.h"
#include "hal_gpio.h"
#include "hal_ddr.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported global variables -------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/
IMPORT void DDR_MCTL_init (void);
IMPORT void DDRPHY_init0 (void);
IMPORT void DDRPHY_init1 (void);
IMPORT void DDR_MCTL_init_2 (void);
IMPORT void DDR_MCTL_init_3 (void);


/**
 * @brief To Set data to register
 * @details To Set data to register
 * @param[in] 	:-Register address
 * 				 -Register value
 * @param[out]		: N/A
 */
static void SETREG32(unsigned int IP_Addr, unsigned int IP_Val)
{
	volatile unsigned int *p = (unsigned int *)IP_Addr;
	*p = IP_Val;
  __asm__("dsb");
}

/**
 * @brief To Read data to register
 * @details To Read data to register
 * @param[in] 	:Register address
 * @param[out]	:Register value
 */
int GETREG32(unsigned int IP_Addr)
{
	volatile unsigned int *p = (unsigned int *)IP_Addr;
        return *p;
}

///**
// * @brief To Initialize MCTL LPDDR3 module
// * @details To Initialize LPDDR3 module
// * @param[in] argv	: N/A
// * @param[in] argc	: N/A
// * @param[out]		: N/A
// */
//void DDR_MCTL_init (void)
//{
//	 SETREG32(base_DDRCs+0x0304, 0x00000001);
//	 SETREG32(base_DDRCs+0x0030, 0x00000001);
//	 GETREG32(base_DDRCs+0x0004);
//	 SETREG32(base_DDRCs+0x0000, 0x03040000);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0000, 0x03040008);
//	 SETREG32(base_DDRCs+0x0010, 0x00000030);
//	 SETREG32(base_DDRCs+0x0010, 0x00000030);
//	 SETREG32(base_DDRCs+0x0010, 0x00003030);
//	 SETREG32(base_DDRCs+0x0010, 0x00003030);
//	 SETREG32(base_DDRCs+0x0014, 0x0000a630);
//	 SETREG32(base_DDRCs+0x0020, 0x00000001);
//	 SETREG32(base_DDRCs+0x0020, 0x00000001);
//	 SETREG32(base_DDRCs+0x0020, 0x00000041);
//	 SETREG32(base_DDRCs+0x0024, 0x49af1c53);
//	 SETREG32(base_DDRCs+0x0030, 0x00000000);
//	 SETREG32(base_DDRCs+0x0030, 0x00000000);
//	 SETREG32(base_DDRCs+0x0030, 0x00000000);
//	 SETREG32(base_DDRCs+0x0030, 0x00000008);
//	 SETREG32(base_DDRCs+0x0030, 0x00000008);
//	 SETREG32(base_DDRCs+0x0034, 0x00402000);
//	 SETREG32(base_DDRCs+0x0034, 0x00404900);
//	 SETREG32(base_DDRCs+0x0034, 0x000d4900);
//	 SETREG32(base_DDRCs+0x0038, 0x00000002);
//	 SETREG32(base_DDRCs+0x0038, 0x00000000);
//	 SETREG32(base_DDRCs+0x0038, 0x00a70000);
//	 SETREG32(base_DDRCs+0x0050, 0x00210000);
//	 SETREG32(base_DDRCs+0x0050, 0x00210070);
//	 SETREG32(base_DDRCs+0x0050, 0x0021f070);
//	 SETREG32(base_DDRCs+0x0050, 0x0071f070);
//	 SETREG32(base_DDRCs+0x0060, 0x00000000);
//	 SETREG32(base_DDRCs+0x0060, 0x00000000);
//	 SETREG32(base_DDRCs+0x0064, 0x00620038);
//	 SETREG32(base_DDRCs+0x0064, 0x00620038);
//	 SETREG32(base_DDRCs+0x0064, 0x00200038);
//	 SETREG32(base_DDRCs+0x00c0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00c0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00c0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00d0, 0x00020003);
//	 SETREG32(base_DDRCs+0x00d0, 0x00030003);
//	 SETREG32(base_DDRCs+0x00d0, 0x00030003);
//	 SETREG32(base_DDRCs+0x00d4, 0x0000000f);
//	 SETREG32(base_DDRCs+0x00d8, 0x00000d06);
//	 SETREG32(base_DDRCs+0x00d8, 0x00000a06);
//	 SETREG32(base_DDRCs+0x00dc, 0x00000046);
//	 SETREG32(base_DDRCs+0x00dc, 0x00c30046);
//	 SETREG32(base_DDRCs+0x00e0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00e0, 0x00020000);
//	 SETREG32(base_DDRCs+0x00e4, 0x00100004);
//	 SETREG32(base_DDRCs+0x00e4, 0x000a0004);
//	 SETREG32(base_DDRCs+0x00f0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00f0, 0x00000000);
//	 SETREG32(base_DDRCs+0x00f4, 0x0000066e);
//	 SETREG32(base_DDRCs+0x00f4, 0x0000068e);
//	 SETREG32(base_DDRCs+0x00f4, 0x0000098e);
//	 SETREG32(base_DDRCs+0x0100, 0x0f101b0b);
//	 SETREG32(base_DDRCs+0x0100, 0x0f10110b);
//	 SETREG32(base_DDRCs+0x0100, 0x0f0e110b);
//	 SETREG32(base_DDRCs+0x0100, 0x080e110b);
//	 SETREG32(base_DDRCs+0x0104, 0x00080413);
//	 SETREG32(base_DDRCs+0x0104, 0x00080213);
//	 SETREG32(base_DDRCs+0x0104, 0x00020213);
//	 SETREG32(base_DDRCs+0x0108, 0x03050607);
//	 SETREG32(base_DDRCs+0x0108, 0x03050607);
//	 SETREG32(base_DDRCs+0x0108, 0x03040607);
//	 SETREG32(base_DDRCs+0x0108, 0x02040607);
//	 SETREG32(base_DDRCs+0x010c, 0x00505000);
//	 SETREG32(base_DDRCs+0x010c, 0x00505000);
//	 SETREG32(base_DDRCs+0x0110, 0x05040407);
//	 SETREG32(base_DDRCs+0x0110, 0x05040307);
//	 SETREG32(base_DDRCs+0x0110, 0x05020307);
//	 SETREG32(base_DDRCs+0x0110, 0x07020307);
//	 SETREG32(base_DDRCs+0x0114, 0x05050404);
//	 SETREG32(base_DDRCs+0x0114, 0x05050404);
//	 SETREG32(base_DDRCs+0x0114, 0x05070404);
//	 SETREG32(base_DDRCs+0x0114, 0x08070404);
//	 SETREG32(base_DDRCs+0x0118, 0x02020006);
//	 SETREG32(base_DDRCs+0x0118, 0x020a0006);
//	 SETREG32(base_DDRCs+0x0118, 0x0a0a0006);
//	 SETREG32(base_DDRCs+0x011c, 0x0000020e);
//	 SETREG32(base_DDRCs+0x011c, 0x0000070e);
//	 SETREG32(base_DDRCs+0x0120, 0x00004401);
//	 SETREG32(base_DDRCs+0x0120, 0x00002901);
//	 SETREG32(base_DDRCs+0x0138, 0x0000003b);
//	 SETREG32(base_DDRCs+0x0180, 0x02000018);
//	 SETREG32(base_DDRCs+0x0180, 0x00600018);
//	 SETREG32(base_DDRCs+0x0180, 0x00600018);
//	 SETREG32(base_DDRCs+0x0180, 0x40600018);
//	 SETREG32(base_DDRCs+0x0180, 0x40600018);
//	 SETREG32(base_DDRCs+0x0184, 0x02000070);
//	 SETREG32(base_DDRCs+0x0184, 0x00e00070);
//	 SETREG32(base_DDRCs+0x0188, 0x00000000);
//	 SETREG32(base_DDRCs+0x0190, 0x07020002);
//	 SETREG32(base_DDRCs+0x0190, 0x07020102);
//	 SETREG32(base_DDRCs+0x0190, 0x07020102);
//	 SETREG32(base_DDRCs+0x0190, 0x07040102);
//	 SETREG32(base_DDRCs+0x0190, 0x07040102);
//	 SETREG32(base_DDRCs+0x0190, 0x02030002);
//	 SETREG32(base_DDRCs+0x0194, 0x00000402);
//	 SETREG32(base_DDRCs+0x0194, 0x00000202);
//	 SETREG32(base_DDRCs+0x0194, 0x00030202);
//	 SETREG32(base_DDRCs+0x0198, 0x07000000);
//	 SETREG32(base_DDRCs+0x0198, 0x07000001);
//	 SETREG32(base_DDRCs+0x0198, 0x07000001);
//	 SETREG32(base_DDRCs+0x0198, 0x07000001);
//	 SETREG32(base_DDRCs+0x0198, 0x07009001);
//	 SETREG32(base_DDRCs+0x0198, 0x07719001);
//	 SETREG32(base_DDRCs+0x0198, 0x07719001);
//	 SETREG32(base_DDRCs+0x01a0, 0x00400005);
//	 SETREG32(base_DDRCs+0x01a0, 0x00400005);
//	 SETREG32(base_DDRCs+0x01a0, 0x20400005);
//	 SETREG32(base_DDRCs+0x01a0, 0x60400005);
//	 SETREG32(base_DDRCs+0x01a0, 0x60400005);
//	 SETREG32(base_DDRCs+0x01a4, 0x000100d8);
//	 SETREG32(base_DDRCs+0x01a4, 0x003000d8);
//	 SETREG32(base_DDRCs+0x01a8, 0x00000000);
//	 SETREG32(base_DDRCs+0x01b0, 0x00000001);
//	 SETREG32(base_DDRCs+0x01b0, 0x00000001);
//	 SETREG32(base_DDRCs+0x01b0, 0x00000001);
//	 SETREG32(base_DDRCs+0x01b0, 0x00000001);
//	 SETREG32(base_DDRCs+0x0200, 0x00000000);
//	 SETREG32(base_DDRCs+0x0204, 0x00000003);
//	 SETREG32(base_DDRCs+0x0204, 0x00000b03);
//	 SETREG32(base_DDRCs+0x0204, 0x00070b14);
//	 SETREG32(base_DDRCs+0x0208, 0x00000000);
//	 SETREG32(base_DDRCs+0x0208, 0x00000600);
//	 SETREG32(base_DDRCs+0x0208, 0x00040600);
//	 SETREG32(base_DDRCs+0x0208, 0x07040600);
//	 SETREG32(base_DDRCs+0x020c, 0x00000001);
//	 SETREG32(base_DDRCs+0x020c, 0x00000701);
//	 SETREG32(base_DDRCs+0x020c, 0x00050701);
//	 SETREG32(base_DDRCs+0x020c, 0x02050701);
//	 SETREG32(base_DDRCs+0x0210, 0x0000000f);
//	 SETREG32(base_DDRCs+0x0210, 0x00000f0f);
//	 SETREG32(base_DDRCs+0x0214, 0x00000009);
//	 SETREG32(base_DDRCs+0x0214, 0x00000309);
//	 SETREG32(base_DDRCs+0x0214, 0x00080309);
//	 SETREG32(base_DDRCs+0x0214, 0x0b080309);
//	 SETREG32(base_DDRCs+0x0218, 0x00000009);
//	 SETREG32(base_DDRCs+0x0218, 0x00000709);
//	 SETREG32(base_DDRCs+0x0218, 0x00050709);
//	 SETREG32(base_DDRCs+0x0218, 0x0f050709);
//	 SETREG32(base_DDRCs+0x0218, 0x0f050709);
//	 SETREG32(base_DDRCs+0x0224, 0x00000007);
//	 SETREG32(base_DDRCs+0x0224, 0x00000a07);
//	 SETREG32(base_DDRCs+0x0224, 0x00080a07);
//	 SETREG32(base_DDRCs+0x0224, 0x00080a07);
//	 SETREG32(base_DDRCs+0x0228, 0x00000004);
//	 SETREG32(base_DDRCs+0x0228, 0x00000104);
//	 SETREG32(base_DDRCs+0x0228, 0x00080104);
//	 SETREG32(base_DDRCs+0x0228, 0x05080104);
//	 SETREG32(base_DDRCs+0x022c, 0x00000000);
//
//	 SETREG32(base_DDRCs+0x0200, 0x00001515);
//	 SETREG32(base_DDRCs+0x0204, 0x00131313);
//	 SETREG32(base_DDRCs+0x0208, 0x00000000);
//	 SETREG32(base_DDRCs+0x020c, 0x00000000);
//	 SETREG32(base_DDRCs+0x0210, 0x00000f0f);
//	 SETREG32(base_DDRCs+0x0214, 0x07040404);
//
//	 SETREG32(base_DDRCs+0x0218, 0x0f0f0707);
//	 SETREG32(base_DDRCs+0x0224, 0x04040404);
//	 SETREG32(base_DDRCs+0x0228, 0x04040404);
//
//	 SETREG32(base_DDRCs+0x0240, 0x04000440);
//	 SETREG32(base_DDRCs+0x0240, 0x04000c40);
//	 SETREG32(base_DDRCs+0x0240, 0x04120c40);
//	 SETREG32(base_DDRCs+0x0240, 0x0c120c40);
//	 SETREG32(base_DDRCs+0x0244, 0x00002212);
//	 SETREG32(base_DDRCs+0x0244, 0x00002232);
//	 SETREG32(base_DDRCs+0x0244, 0x00002132);
//	 SETREG32(base_DDRCs+0x0244, 0x00000132);
//	 SETREG32(base_DDRCs+0x0250, 0x00000905);
//	 SETREG32(base_DDRCs+0x0250, 0x00000905);
//	 SETREG32(base_DDRCs+0x0250, 0x00000905);
//	 SETREG32(base_DDRCs+0x0250, 0x00000905);
//	 SETREG32(base_DDRCs+0x0250, 0x00f20905);
//	 SETREG32(base_DDRCs+0x0250, 0x07f20905);
//	 SETREG32(base_DDRCs+0x0254, 0x0000001e);
//	 SETREG32(base_DDRCs+0x025c, 0x0f00d6a1);
//	 SETREG32(base_DDRCs+0x025c, 0x1300d6a1);
//	 SETREG32(base_DDRCs+0x0264, 0x0f00cfbe);
//	 SETREG32(base_DDRCs+0x0264, 0xd000cfbe);
//	 SETREG32(base_DDRCs+0x026c, 0x0f00e287);
//	 SETREG32(base_DDRCs+0x026c, 0x1600e287);
//	 SETREG32(base_DDRCs+0x0300, 0x00000001);
//	 SETREG32(base_DDRCs+0x0300, 0x00000011);
//	 SETREG32(base_DDRCs+0x0304, 0x00000000);
//	 SETREG32(base_DDRCs+0x0304, 0x00000000);
//	 SETREG32(base_DDRCs+0x030c, 0x00000000);
//	 SETREG32(base_DDRCs+0x030c, 0x00000000);
//	 SETREG32(base_DDRCs+0x030c, 0x00000000);
//	 SETREG32(base_DDRCs+0x0320, 0x00000001);
//	 SETREG32(base_DDRCs+0x036c, 0x00110010);
//	 SETREG32(base_DDRCs+0x036c, 0x00110000);
//	 SETREG32(base_DDRCs+0x036c, 0x00110000);
//	 SETREG32(base_DDRCs+0x036c, 0x00100000);
//	 SETREG32(base_DDRCs+0x036c, 0x00000000);
//	 SETREG32(base_DDRCs+0x036c, 0x00000000);
//
//	 SETREG32(base_DDRCs+0x2020, 0x00000001);
//	 SETREG32(base_DDRCs+0x2020, 0x00000001);
//	 SETREG32(base_DDRCs+0x2020, 0x00000041);
//	 SETREG32(base_DDRCs+0x2024, 0x49af1c53);
//	 SETREG32(base_DDRCs+0x2050, 0x00210070);
//	 SETREG32(base_DDRCs+0x2050, 0x00210070);
//	 SETREG32(base_DDRCs+0x2050, 0x0021f070);
//	 SETREG32(base_DDRCs+0x2050, 0x0071f070);
//	 SETREG32(base_DDRCs+0x2064, 0x00620038);
//	 SETREG32(base_DDRCs+0x2064, 0x00628038);
//	 SETREG32(base_DDRCs+0x2064, 0x00208038);
//	 SETREG32(base_DDRCs+0x20dc, 0x00000046);
//	 SETREG32(base_DDRCs+0x20dc, 0x00c30046);
//	 SETREG32(base_DDRCs+0x20e0, 0x00000000);
//	 SETREG32(base_DDRCs+0x20e0, 0x00020000);
//	 SETREG32(base_DDRCs+0x2100, 0x0f101b0b);
//	 SETREG32(base_DDRCs+0x2100, 0x0f10110b);
//	 SETREG32(base_DDRCs+0x2100, 0x0f0e110b);
//	 SETREG32(base_DDRCs+0x2100, 0x080e110b);
//	 SETREG32(base_DDRCs+0x2104, 0x00080413);
//	 SETREG32(base_DDRCs+0x2104, 0x00080213);
//	 SETREG32(base_DDRCs+0x2104, 0x00020213);
//	 SETREG32(base_DDRCs+0x2108, 0x03050607);
//	 SETREG32(base_DDRCs+0x2108, 0x03050607);
//	 SETREG32(base_DDRCs+0x2108, 0x03040607);
//	 SETREG32(base_DDRCs+0x2108, 0x02040607);
//	 SETREG32(base_DDRCs+0x210c, 0x00505000);
//	 SETREG32(base_DDRCs+0x210c, 0x00505000);
//	 SETREG32(base_DDRCs+0x2110, 0x05040407);
//	 SETREG32(base_DDRCs+0x2110, 0x05040307);
//	 SETREG32(base_DDRCs+0x2110, 0x05020307);
//	 SETREG32(base_DDRCs+0x2110, 0x07020307);
//	 SETREG32(base_DDRCs+0x2114, 0x05050404);
//	 SETREG32(base_DDRCs+0x2114, 0x05050404);
//	 SETREG32(base_DDRCs+0x2114, 0x05070404);
//	 SETREG32(base_DDRCs+0x2114, 0x08070404);
//	 SETREG32(base_DDRCs+0x2118, 0x02020006);
//	 SETREG32(base_DDRCs+0x2118, 0x020a0006);
//	 SETREG32(base_DDRCs+0x2118, 0x0a0a0006);
//	 SETREG32(base_DDRCs+0x211c, 0x0000020e);
//	 SETREG32(base_DDRCs+0x211c, 0x0000070e);
//	 SETREG32(base_DDRCs+0x2120, 0x00004401);
//	 SETREG32(base_DDRCs+0x2120, 0x00002901);
//	 SETREG32(base_DDRCs+0x2138, 0x0000003b);
//	 SETREG32(base_DDRCs+0x2180, 0x02000018);
//	 SETREG32(base_DDRCs+0x2180, 0x00600018);
//	 SETREG32(base_DDRCs+0x2180, 0x00600018);
//	 SETREG32(base_DDRCs+0x2180, 0x40600018);
//	 SETREG32(base_DDRCs+0x2180, 0x40600018);
//	 SETREG32(base_DDRCs+0x2190, 0x07020002);
//	 SETREG32(base_DDRCs+0x2190, 0x07020102);
//	 SETREG32(base_DDRCs+0x2190, 0x07020102);
//	 SETREG32(base_DDRCs+0x2190, 0x07040102);
//	 SETREG32(base_DDRCs+0x2190, 0x07040102);
//	 SETREG32(base_DDRCs+0x2190, 0x02040102);
//	 SETREG32(base_DDRCs+0x2194, 0x00000402);
//	 SETREG32(base_DDRCs+0x2194, 0x00000202);
//	 SETREG32(base_DDRCs+0x2194, 0x00030202);
//	 SETREG32(base_DDRCs+0x2240, 0x04000440);
//	 SETREG32(base_DDRCs+0x2240, 0x04000c40);
//	 SETREG32(base_DDRCs+0x2240, 0x04120c40);
//	 SETREG32(base_DDRCs+0x2240, 0x0c120c40);
//
//	 SETREG32(base_DDRCs+0x0400, 0x00000010);
//
//	 SETREG32(base_DDRCs+0x0404, 0x000000da);
//	 SETREG32(base_DDRCs+0x0408, 0x00006201);
//	 SETREG32(base_DDRCs+0x0490, 0x00000001);
//	 SETREG32(base_DDRCs+0x0494, 0x00200001);
//
//	 SETREG32(base_DDRCs+0x04b4, 0x0000119b);
//	 SETREG32(base_DDRCs+0x04b8, 0x00001000);
//	 SETREG32(base_DDRCs+0x0540, 0x00000001);
//	 SETREG32(base_DDRCs+0x0544, 0x0011000C);
//
//	 SETREG32(base_DDRCs+0x0564, 0x00000156);
//	 SETREG32(base_DDRCs+0x0568, 0x00005000);
//	 SETREG32(base_DDRCs+0x05f0, 0x00000001);
//	 SETREG32(base_DDRCs+0x05f4, 0x0011000C);
//
//	 SETREG32(base_DDRCs+0x0614, 0x00004223);
//	 SETREG32(base_DDRCs+0x0618, 0x00003000);
//	 SETREG32(base_DDRCs+0x06a0, 0x00000001);
//	 SETREG32(base_DDRCs+0x06a4, 0x0011000C);
//
//	 SETREG32(base_DDRCs+0x06c4, 0x00005288);
//	 SETREG32(base_DDRCs+0x06c8, 0x0000410f);
//	 SETREG32(base_DDRCs+0x0750, 0x00000001);
//	 SETREG32(base_DDRCs+0x0754, 0x00000001);
//
//	 SETREG32(base_DDRCs+0x0490, 0x00000001);
//	 SETREG32(base_DDRCs+0x0540, 0x00000001);
//	 SETREG32(base_DDRCs+0x05f0, 0x00000001);
//	 SETREG32(base_DDRCs+0x06a0, 0x00000001);
//	 SETREG32(base_DDRCs+0x0750, 0x00000001);
//
//	 GETREG32(base_DDRCs+0x0060);
//}

///**
// * @brief To Initialize MTCL SEGMENT_2 LPDDR3 module
// * @details To Initialize LPDDR3 module
// * @param[in] argv	: N/A
// * @param[in] argc	: N/A
// * @param[out]		: N/A
// */
//LOCAL void DDR_MCTL_init_2 (void)
//{
//	SETREG32(base_DDRCs+0x0304, 0x00000000);
//	GETREG32(base_DDRCs+0x0030);
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	GETREG32(base_DDRCs+0x0030);
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	SETREG32(base_DDRCs+0x0320, 0x00000000);
//	SETREG32(base_DDRCs+0x01b0, 0x00000000);
//}
//
///**
// * @brief To Initialize PHY SEGMENT_0 LPDDR3 module
// * @details To Initialize LPDDR3 module
// * @param[in] argv	: N/A
// * @param[in] argc	: N/A
// * @param[out]		: N/A
// */
//LOCAL void DDRPHY_init0 (void)
//{
//	unsigned int get_value,pgsr_ok;
//
//	SETREG32(base_ddrp0+DCR,0x0000000d);
//	SETREG32(base_ddrp0+MR0,0x00000c52);
//	SETREG32(base_ddrp0+MR1,0x000000c3);
//	SETREG32(base_ddrp0+MR2,0x00000006);
//	SETREG32(base_ddrp0+MR3,0x00000002);
//	SETREG32(base_ddrp0+DTPR0,0x44d7abb2);
//	SETREG32(base_ddrp0+DTPR1,0x194610d8);
//	SETREG32(base_ddrp0+DTPR2,0x064790c8);
//	SETREG32(base_ddrp0+PGCR,0x01842e02);
//	SETREG32(base_ddrp0+DXCCR, 0x00000c40);
//	SETREG32(base_ddrp0+DSGCR ,0xfa00025f);
//	SETREG32(base_ddrp0+DX0DQSTR ,0x3db03000);
//	SETREG32(base_ddrp0+DX1DQSTR ,0x3db03000);
//	SETREG32(base_ddrp0+DX2DQSTR ,0x3db03000);
//	SETREG32(base_ddrp0+DX3DQSTR ,0x3db03000);
//
//	do
//	{
//	  get_value = GETREG32( base_ddrp0 + PGSR );
//	  pgsr_ok =  get_value   & 0x1;
//	}
//	while( pgsr_ok != 0x1 );
//
//	get_value = 0;
//	pgsr_ok = 0;
//	SETREG32(base_ddrp0+PIR, 0x000000c1);
//
//	do
//	{
//	  get_value = GETREG32( base_ddrp0 + PGSR );
//	  pgsr_ok =  get_value   & 0xf;
//	}
//	while( pgsr_ok != 0xf );
//
//}
//
///**
// * @brief To Initialize PHY SEGMENT_1 LPDDR3 module
// * @details To Initialize LPDDR3 module
// * @param[in] argv	: N/A
// * @param[in] argc	: N/A
// * @param[out]		: N/A
// */
//LOCAL void DDRPHY_init1 (void)
//{
//	unsigned int get_value,pgsr_ok;
//
//	 SETREG32(base_ddrp1+DCR,0x0000000d);
//	 SETREG32(base_ddrp1+MR0,0x00000c52);
//	 SETREG32(base_ddrp1+MR1,0x000000c3);
//	 SETREG32(base_ddrp1+MR2,0x00000006);
//	 SETREG32(base_ddrp1+MR3,0x00000002);
//	 SETREG32(base_ddrp1+DTPR0,0x44d7abb2);
//	 SETREG32(base_ddrp1+DTPR1,0x194610d8);
//	 SETREG32(base_ddrp1+DTPR2,0x064790c8);
//	 SETREG32(base_ddrp1+PGCR,0x01842e02);
//	 SETREG32(base_ddrp1+DXCCR, 0x00000c40);
//	 SETREG32(base_ddrp1+DSGCR ,0xfa00025f);
//	 SETREG32(base_ddrp1+DX0DQSTR ,0x3db03000);
//	 SETREG32(base_ddrp1+DX1DQSTR ,0x3db03000);
//	 SETREG32(base_ddrp1+DX2DQSTR ,0x3db03000);
//	 SETREG32(base_ddrp1+DX3DQSTR ,0x3db03000);
//
//	 do
//	 {
//		  get_value = GETREG32( base_ddrp1 + PGSR );
//		  pgsr_ok =  get_value   & 0x1;
//	 }
//	 while( pgsr_ok != 0x1 );
//
//	 get_value = 0;
//	 pgsr_ok = 0;
//	 SETREG32(base_ddrp1+PIR, 0x000000c1);
//	 do
//	 {
//	  get_value = GETREG32( base_ddrp1 + PGSR );
//	  pgsr_ok =  get_value   & 0xf;
//	 }
//	 while( pgsr_ok != 0xf );
//}
//
///**
// * @brief To Initialize MTCL LPDDR3 module
// * @details To Initialize LPDDR3 module
// * @param[in] argv	: N/A
// * @param[in] argc	: N/A
// * @param[out]		: N/A
// */
//LOCAL void DDR_MCTL_init_3 (void)
//{
//	unsigned int get_value,pgsr_ok;
//	SETREG32(base_DDRCs+0x01b0, 0x00000001);
//	SETREG32(base_DDRCs+0x0320, 0x00000001);
//	do {
//	  get_value = GETREG32(base_DDRCs + 0x0324 );
//	  pgsr_ok =  get_value   & 0x1;
//	} while( pgsr_ok != 0x1 );
//
//	do {
//	  get_value = GETREG32(base_DDRCs + 0x0004 );
//	  pgsr_ok =  get_value   & 0x1;
//	} while( pgsr_ok != 0x1 );
//
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	SETREG32(base_DDRCs+0x0030, 0x00000008);
//	GETREG32(base_DDRCs+0x0308);
//	GETREG32(base_DDRCs+0x0308);
//	SETREG32(base_DDRCs+0x0060, 0x00000001);
//
//}
//
/**
 * @brief To Initialize LPDDR3 module
 * @details To Initialize LPDDR3 module
 * @param[in] argv	: N/A
 * @param[in] argc	: N/A
 * @param[out]		: N/A
 */
LOCAL void DDR_INIT (void)
{
	/**! DDR CTRL INIT */
	SETREG32(base_SCU + 0x3c, 0x00007d00);
	SETREG32(base_SCU + 0x44, 0x000c0000);
	DDR_MCTL_init();
	SETREG32(base_SCU + 0x3c, 0x00000000);
	DDR_MCTL_init_2();
	/**! DDR PHY INIT */
	SETREG32(base_SCU + 0x44, 0x00080000);
	DDRPHY_init0();
	SETREG32(base_SCU + 0x44, 0x00000000);
	DDRPHY_init1();
	DDR_MCTL_init_3();
}
/**
 * @brief To verify write/read DDRAM
 * @details To verify write/read DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_ddr_001(char** argv, int argc);
/**
 * @brief To verify write speed from CPU to DDRAM
 * @details To verify write speed from CPU to DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 	0		: success
 * 			Others	: instruction failed
 */
LOCAL int it_hal_ddr_002(char** argv, int argc);
/**
 * @brief To verify read speed from CPU to DDRAM
 * @details To verify write speed from CPU to DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 	0		: success
 * 			Others	: instruction failed
 */
LOCAL int it_hal_ddr_003(char** argv, int argc);
/**
 *  @Brief: DDR module testing map
 */
LOCAL it_map_t it_gpio_tests_table[] = {
		{"DDR_001", it_hal_ddr_001},
		{"DDR_002", it_hal_ddr_002},
		{"DDR_003", it_hal_ddr_003},
		{"", NULL}
};

/**
 * @brief DDR module's testing handler
 * @details to DDR module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	: success
 *         Others: instruction failed
 */
int it_hal_ddr_handler(char** argv, int argc)
{
	/**! Check command counter */
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**! Get command index, mean test case name */
	int index = handler_parser(*argv, it_gpio_tests_table);
	if (-1 != index)
	{
		return it_gpio_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}
/**
 * @brief To verify write/read DDRAM
 * @details To verify write/read DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_ddr_001(char** argv, int argc)
{
	/**! Check command parameter */
	if ((3 != argc) && (argv == NULL))
	{
		log_printf("Error: Wrong number of parameters or Empty argument\r\n");
		return -1;
	}
	uint32_t address = 0x00;
	uint32_t size    = 0;
	uint32_t* ptr    = NULL;
	uint8_t select   = 0;
	/**! Get address physical memory DDR via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);
	/**! Get byte number write to memory*/
	size = (uint32_t)strtol(argv[1], NULL, 10);
	/**! Get variable select */
	select = (uint8_t)strtol(argv[2], NULL, 10);
	/**! Base address memory */
	ptr = (uint32_t*)(address);
	/**! Reset all test point to default */
	qc_assert_reset();
	/**! Initialize DDR RAM memory */
	if(!select)
	{
		//hal_ddr_init();
	}
	else
	{
		//DDR_INIT();
	}
	/**! Write data size 1M */
	for(uint32_t i = 0; i < size; i++)
	{
		ptr[i] = i;
	}
	/**! Read and verify data size 1M */
	for(uint32_t i = 0; i < size; i++)
	{
		if (ptr[i] != i)
		{
			/**! Notify failed test */
			qc_assert(FALSE);
			log_printf("Address error: %4x\r\n", &ptr[i]);
			break;
		}
	}
	/**! Do judgment */
	qc_report();
	/**! Indicate test success */
	return 0;
}
/**
 * @brief To verify write speed from CPU to DDRAM
 * @details To verify write speed from CPU to DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 	0		: success
 * 			Others	: instruction failed
 */
LOCAL int it_hal_ddr_002(char** argv, int argc)
{
	/**! Check command parameter */
	if ((3 != argc) && (argv == NULL))
	{
		log_printf("Error: Wrong number of parameters or Empty argument\r\n");
		return -1;
	}
	hal_gpio_t gpio  = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};
	uint32_t address = 0x00;
	uint32_t size    = 0;
	uint8_t select   = 0;
	volatile uint32_t* ptr    = NULL;
	/**! Get address physical memory DDR via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);
	/**! Get byte number write to memory*/
	size = (uint32_t)strtol(argv[1], NULL, 10);
	/**! Get variable select */
	select = (uint8_t)strtol(argv[2], NULL, 10);
	/**! Base address memory */
	ptr = (volatile uint32_t*)(address);
	/**! Reset all test point to default */
	qc_assert_reset();
	/**! Initialize GPIO */
	hal_gpio_init(&gpio);
	/**! Initialize DDR RAM memory */
	if(!select)
	{
		hal_ddr_init();
	}
	else
	{
		DDR_INIT();
	}
	//! Set gpio to HIGH
	hal_gpio_set_high(&gpio);
	/**! Write data size */
	for(uint32_t i = 0; i < size; i++)
	{
		ptr[i] = i;
	}
	//! Set gpio to LOW
	hal_gpio_set_low(&gpio);
	/**! Do judgment */
	qc_report();
	return 0;
}
/**
 * @brief To verify read speed from CPU to DDRAM
 * @details To verify write speed from CPU to DDRAM
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 	0		: success
 * 			Others	: instruction failed
 */
LOCAL int it_hal_ddr_003(char** argv, int argc)
{
	/**! Check command parameter */
	if ((3 != argc) && (argv == NULL))
	{
		log_printf("Error: Wrong number of parameters or Empty argument\r\n");
		return -1;
	}
	hal_gpio_t gpio  = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};
	uint32_t address = 0x00;
	uint32_t size    = 0;
	uint8_t select   = 0;
	volatile uint32_t* ptr    = NULL;
	volatile uint32_t data    = 0;
	/**! Get address physical memory DDR via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);
	/**! Get byte number write to memory*/
	size = (uint32_t)strtol(argv[1], NULL, 10);
	/**! Get variable select */
	select = (uint8_t)strtol(argv[2], NULL, 10);
	/**! Base address memory */
	ptr = (volatile uint32_t*)(address);
	/**! Reset all test point to default */
	qc_assert_reset();
	/**! Initialize GPIO */
	hal_gpio_init(&gpio);
	/**! Initialize DDR RAM memory */
	if(!select)
	{
		hal_ddr_init();
	}
	else
	{
		DDR_INIT();
	}
	//! Set gpio to HIGH
	hal_gpio_set_high(&gpio);
	/**! Read data size */
	for(uint32_t i = 0; i < size; i++)
	{
		data = ptr[i];
	}
	//! Set gpio to LOW
	hal_gpio_set_low(&gpio);
	// Dummy read to avoid build warning
	data = data;
	/**! Do judgment */
	qc_report();
	return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
