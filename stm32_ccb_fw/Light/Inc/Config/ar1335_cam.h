/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    ar1335_cam.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-29-2016
 * @brief   This file contains configuration for various type of camera stream
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef LIGHT_INC_AR1335_CAM_H_
#define LIGHT_INC_AR1335_CAM_H_

/* Using for new PLL configurations, if want to come back old setting, undefine it */
#define NEW_PLL_CONFIG

#include "types.h"
#define AR1335_SLAVE_ADDR		0x6C

typedef struct {
	uint16_t reg_addr;
	uint16_t reg_val;
} msm_camera_i2c_reg_array;

typedef struct {
	msm_camera_i2c_reg_array *regs;
	UInt16 reg_size;
	UInt16 data_size;
} cam_reg_array;

typedef struct {
	cam_reg_array *config;
	uint16_t width;
	uint16_t height;
	uint16_t fps;
	uint8_t sizeofconfig;
} msm_camera_stream_type;


msm_camera_i2c_reg_array start_reg_array[] =
{
	{0x0100, 0x01 },
};

msm_camera_i2c_reg_array enable_streaming[] =
{
	{ 0x3F3C, 0x0003 },
	{ 0x301A, 0x021C }
};

msm_camera_i2c_reg_array analog_control_on[] =
{
	{0x3F3C, 0x0003}
};

msm_camera_i2c_reg_array analog_control_off[] =
{
	{0x3F3C, 0x0002},
	{0x3FE0, 0x0001}
};

msm_camera_i2c_reg_array analog_control_reset[] =
{
	{0x3FE0, 0x0000}
};

msm_camera_i2c_reg_array ar1335_black_level_correction[] = {
		{ 0x3180, 0xD434 }
};

msm_camera_i2c_reg_array stop_reg_array[] =
{
	{0x0100, 0x00 },
};

msm_camera_i2c_reg_array groupon_reg_array[] =
{
	{0x0104, 0x01 },
};

msm_camera_i2c_reg_array groupoff_reg_array[] =
{
	{0x0104, 0x00 },
};

msm_camera_i2c_reg_array slave_mode_control_reg_array[] = {
	{ 0x301A, 0x0318 },	/* SW Standby */
	{ 0x3026, 0xFD7F },	/* Trigger pin select */
//	{ 0x315E, 0x0001 }, /* Global Trigger */
	{ 0x3158, 0xA000 },	/* Slave mode control */
	{ 0x301A, 0x031C },	/* Stream on */
	{ 0x315E, 0x0002 }, /* Global Trigger */
};

msm_camera_i2c_reg_array slave_mode_enable_reg_array[] = {
	{ 0x3158, 0xA000 },	/*!< Slave mode control */
};

msm_camera_i2c_reg_array slave_mode_disable_reg_array[] = {
	{ 0x3158, 0x0000 },	/*!< Slave mode control */
};

msm_camera_i2c_reg_array reset_reg_array[] = {
	{0x3F3C, 0x0003},
	{ 0x301A, 0x001C },
	{ 0x301A, 0x021C }
};

msm_camera_i2c_reg_array correction_recommend[] = {
	{ 0x3042, 0x1004 },
	{ 0x30D2, 0x0120 },
	{ 0x30D4, 0x0 },
	{ 0x3090, 0x0 },
	{ 0x30FC, 0x0060 },
	{ 0x30FE, 0x0060 },
	{ 0x31E0, 0x0781 },
	{ 0x3180, 0x9434 },
	{ 0x317C, 0xEFF4 },
	{ 0x30EE, 0x613E },
	{ 0x3F2C, 0x4428 }
};

msm_camera_i2c_reg_array pixel_timing_recommended[] = {
	{ 0x3D00, 0x0446 },
	{ 0x3D02, 0x4C66 },
	{ 0x3D04, 0xFFFF },
	{ 0x3D06, 0xFFFF },
	{ 0x3D08, 0x5E40 },
	{ 0x3D0A, 0x1146 },
	{ 0x3D0C, 0x5D41 },
	{ 0x3D0E, 0x1088 },
	{ 0x3D10, 0x8342 },
	{ 0x3D12, 0x00C0 },
	{ 0x3D14, 0x5580 },
	{ 0x3D16, 0x5B83 },
	{ 0x3D18, 0x6084 },
	{ 0x3D1A, 0x5A8D },
	{ 0x3D1C, 0x00C0 },
	{ 0x3D1E, 0x8342 },
	{ 0x3D20, 0x925A },
	{ 0x3D22, 0x8664 },
	{ 0x3D24, 0x1030 },
	{ 0x3D26, 0x801C },
	{ 0x3D28, 0x00A0 },
	{ 0x3D2A, 0x56B0 },
	{ 0x3D2C, 0x5788 },
	{ 0x3D2E, 0x5150 },
	{ 0x3D30, 0x824D },
	{ 0x3D32, 0x8D58 },
	{ 0x3D34, 0x58D2 },
	{ 0x3D36, 0x438A },
	{ 0x3D38, 0x4592 },
	{ 0x3D3A, 0x458A },
	{ 0x3D3C, 0x4389 },
	{ 0x3D3E, 0x51FF },
	{ 0x3D40, 0x8451 },
	{ 0x3D42, 0x8410 },
	{ 0x3D44, 0x0C88 },
	{ 0x3D46, 0x5959 },
	{ 0x3D48, 0x8A5F },
	{ 0x3D4A, 0xDA42 },
	{ 0x3D4C, 0x9361 },
	{ 0x3D4E, 0x8262 },
	{ 0x3D50, 0x8342 },
	{ 0x3D52, 0x8010 },
	{ 0x3D54, 0xC041 },
	{ 0x3D56, 0x64FF },
	{ 0x3D58, 0xFFB7 },
	{ 0x3D5A, 0x4081 },
	{ 0x3D5C, 0x4080 },
	{ 0x3D5E, 0x4180 },
	{ 0x3D60, 0x4280 },
	{ 0x3D62, 0x438D },
	{ 0x3D64, 0x44BA },
	{ 0x3D66, 0x4488 },
	{ 0x3D68, 0x4380 },
	{ 0x3D6A, 0x4241 },
	{ 0x3D6C, 0x8140 },
	{ 0x3D6E, 0x8240 },
	{ 0x3D70, 0x8041 },
	{ 0x3D72, 0x8042 },
	{ 0x3D74, 0x8043 },
	{ 0x3D76, 0x8D44 },
	{ 0x3D78, 0xBA44 },
	{ 0x3D7A, 0x875E },
	{ 0x3D7C, 0x4354 },
	{ 0x3D7E, 0x4241 },
	{ 0x3D80, 0x8140 },
	{ 0x3D82, 0x8120 },
	{ 0x3D84, 0x2881 },
	{ 0x3D86, 0x6026 },
	{ 0x3D88, 0x8055 },
	{ 0x3D8A, 0x8070 },
	{ 0x3D8C, 0x8040 },
	{ 0x3D8E, 0x4C81 },
	{ 0x3D90, 0x45C3 },
	{ 0x3D92, 0x4581 },
	{ 0x3D94, 0x4C40 },
	{ 0x3D96, 0x8070 },
	{ 0x3D98, 0x8040 },
	{ 0x3D9A, 0x4C85 },
	{ 0x3D9C, 0x6CA8 },
	{ 0x3D9E, 0x6C8C },
	{ 0x3DA0, 0x000E },
	{ 0x3DA2, 0xBE44 },
	{ 0x3DA4, 0x8844 },
	{ 0x3DA6, 0xBC78 },
	{ 0x3DA8, 0x0900 },
	{ 0x3DAA, 0x8904 },
	{ 0x3DAC, 0x8080 },
	{ 0x3DAE, 0x0240 },
	{ 0x3DB0, 0x8609 },
	{ 0x3DB2, 0x008E },
	{ 0x3DB4, 0x0900 },
	{ 0x3DB6, 0x8002 },
	{ 0x3DB8, 0x4080 },
	{ 0x3DBA, 0x0480 },
	{ 0x3DBC, 0x887C },
	{ 0x3DBE, 0xAA86 },
	{ 0x3DC0, 0x0900 },
	{ 0x3DC2, 0x877A },
	{ 0x3DC4, 0x000E },
	{ 0x3DC6, 0xC379 },
	{ 0x3DC8, 0x4C40 },
	{ 0x3DCA, 0xBF70 },
	{ 0x3DCC, 0x5E40 },
	{ 0x3DCE, 0x114E },
	{ 0x3DD0, 0x5D41 },
	{ 0x3DD2, 0x5383 },
	{ 0x3DD4, 0x4200 },
	{ 0x3DD6, 0xC055 },
	{ 0x3DD8, 0xA400 },
	{ 0x3DDA, 0xC083 },
	{ 0x3DDC, 0x4288 },
	{ 0x3DDE, 0x6083 },
	{ 0x3DE0, 0x5B80 },
	{ 0x3DE2, 0x5A64 },
	{ 0x3DE4, 0x1030 },
	{ 0x3DE6, 0x801C },
	{ 0x3DE8, 0x00A5 },
	{ 0x3DEA, 0x5697 },
	{ 0x3DEC, 0x57A5 },
	{ 0x3DEE, 0x5180 },
	{ 0x3DF0, 0x505A },
	{ 0x3DF2, 0x814D },
	{ 0x3DF4, 0x8358 },
	{ 0x3DF6, 0x8058 },
	{ 0x3DF8, 0xA943 },
	{ 0x3DFA, 0x8345 },
	{ 0x3DFC, 0xB045 },
	{ 0x3DFE, 0x8343 },
	{ 0x3E00, 0xA351 },
	{ 0x3E02, 0xE251 },
	{ 0x3E04, 0x8C59 },
	{ 0x3E06, 0x8059 },
	{ 0x3E08, 0x8A5F },
	{ 0x3E0A, 0xEC7C },
	{ 0x3E0C, 0xCC84 },
	{ 0x3E0E, 0x6182 },
	{ 0x3E10, 0x6283 },
	{ 0x3E12, 0x4283 },
	{ 0x3E14, 0x10CC },
	{ 0x3E16, 0x6496 },
	{ 0x3E18, 0x4281 },
	{ 0x3E1A, 0x41BB },
	{ 0x3E1C, 0x4082 },
	{ 0x3E1E, 0x407E },
	{ 0x3E20, 0xCC41 },
	{ 0x3E22, 0x8042 },
	{ 0x3E24, 0x8043 },
	{ 0x3E26, 0x8300 },
	{ 0x3E28, 0xC088 },
	{ 0x3E2A, 0x44BA },
	{ 0x3E2C, 0x4488 },
	{ 0x3E2E, 0x00C8 },
	{ 0x3E30, 0x8042 },
	{ 0x3E32, 0x4181 },
	{ 0x3E34, 0x4082 },
	{ 0x3E36, 0x4080 },
	{ 0x3E38, 0x4180 },
	{ 0x3E3A, 0x4280 },
	{ 0x3E3C, 0x4383 },
	{ 0x3E3E, 0x00C0 },
	{ 0x3E40, 0x8844 },
	{ 0x3E42, 0xBA44 },
	{ 0x3E44, 0x8800 },
	{ 0x3E46, 0xC880 },
	{ 0x3E48, 0x4241 },
	{ 0x3E4A, 0x8240 },
	{ 0x3E4C, 0x8140 },
	{ 0x3E4E, 0x8041 },
	{ 0x3E50, 0x8042 },
	{ 0x3E52, 0x8043 },
	{ 0x3E54, 0x8300 },
	{ 0x3E56, 0xC088 },
	{ 0x3E58, 0x44BA },
	{ 0x3E5A, 0x4488 },
	{ 0x3E5C, 0x00C8 },
	{ 0x3E5E, 0x8042 },
	{ 0x3E60, 0x4181 },
	{ 0x3E62, 0x4082 },
	{ 0x3E64, 0x4080 },
	{ 0x3E66, 0x4180 },
	{ 0x3E68, 0x4280 },
	{ 0x3E6A, 0x4383 },
	{ 0x3E6C, 0x00C0 },
	{ 0x3E6E, 0x8844 },
	{ 0x3E70, 0xBA44 },
	{ 0x3E72, 0x8800 },
	{ 0x3E74, 0xC880 },
	{ 0x3E76, 0x4241 },
	{ 0x3E78, 0x8140 },
	{ 0x3E7A, 0x9F5E },
	{ 0x3E7C, 0x8A54 },
	{ 0x3E7E, 0x8620 },
	{ 0x3E80, 0x2881 },
	{ 0x3E82, 0x6026 },
	{ 0x3E84, 0x8055 },
	{ 0x3E86, 0x8070 },
	{ 0x3E88, 0x0000 },
	{ 0x3E8A, 0x0000 },
	{ 0x3E8C, 0x0000 },
	{ 0x3E8E, 0x0000 },
	{ 0x3E90, 0x0000 },
	{ 0x3E92, 0x0000 },
	{ 0x3E94, 0x0000 },
	{ 0x3E96, 0x0000 },
	{ 0x3E98, 0x0000 },
	{ 0x3E9A, 0x0000 },
	{ 0x3E9C, 0x0000 },
	{ 0x3E9E, 0x0000 },
	{ 0x3EA0, 0x0000 },
	{ 0x3EA2, 0x0000 },
	{ 0x3EA4, 0x0000 },
	{ 0x3EA6, 0x0000 },
	{ 0x3EA8, 0x0000 },
	{ 0x3EAA, 0x0000 },
	{ 0x3EAC, 0x0000 },
	{ 0x3EAE, 0x0000 },
	{ 0x3EB0, 0x0000 },
	{ 0x3EB2, 0x0000 },
	{ 0x3EB4, 0x0000 }
};

msm_camera_i2c_reg_array analog_setup_recommended[] = {
	{ 0x3EB6, 0x4D   },
	{ 0x3EBC, 0xAA06 },
	{ 0x3EC0, 0x1E02 },
	{ 0x3EC2, 0x7700 },
	{ 0x3EC4, 0x1C08 },
	{ 0x3EC6, 0xEA44 },
	{ 0x3EC8, 0x0F0F },
	{ 0x3ECA, 0x0F4A },
	{ 0x3ECC, 0x0706 },
	{ 0x3ECE, 0x443B },
	{ 0x3ED0, 0x12F0 },
	{ 0x3ED2, 0x0039 },
	{ 0x3ED4, 0x862F },
	{ 0x3ED6, 0x4080 },
	{ 0x3ED8, 0x0523 },
	{ 0x3EDA, 0xF8AA },
	{ 0x3EDC, 0x5078 },
	{ 0x3EDE, 0x5005 }
};

msm_camera_i2c_reg_array mipi_timing_max[] = {
	{0x31B0, 0x0058},
	{0x31B2, 0x002C},
	{0x31B4, 0x23D0},
	{0x31B6, 0x140A},
	{0x31B8, 0x2413},
	{0x31BA, 0x1C70},
	{0x31BC, 0x860B}
};
msm_camera_i2c_reg_array mipi_timing_880M[] = {
	{0x31B0, 0x4D},
	{0x31B2, 0x28},
	{0x31B4, 0x230E},
	{0x31B6, 0x1348},
	{0x31B8, 0x1C12},
	{0x31BA, 0x185B},
	{0x31BC, 0x8509}
};
msm_camera_i2c_reg_array mipi_timing_880M_DPCM8[] = {
	{0x31B0, 0x5C},
	{0x31B2, 0x2E},
	{0x31B4, 0x2412},
	{0x31B6, 0x142A},
	{0x31B8, 0x2413},
	{0x31BA, 0x1C72},
	{0x31BC, 0x860B}
};
/*
 * //MIPIConfiguration	by Essen
REG=0x31B0, 0x31//Frame preamble 31
REG=0x31B2, 0x1E//Line preamble 1E
REG=0x31B4, 0x1188//MIPI timing0 1188
REG=0x31B6, 0x1165//MIPI timing1 1165
REG=0x31B8, 0x1012//MIPI timing2 1012
REG=0x31BA, 0xC28//MIPI timing3 C28
REG=0x31BC, 0x8284//MIPI timing4 8284
 *
 * */
/*essen rev1*/
msm_camera_i2c_reg_array mipi_timing_recommended[] = {
	{0x31B0, 0x0031},
	{0x31B2, 0x001E},
	{0x31B4, 0x1188},
	{0x31B6, 0x1165},
	{0x31B8, 0x1012},
	{0x31BA, 0x0C28},
	{0x31BC, 0x8284}
};

msm_camera_i2c_reg_array pll_setup_max[] = {
	{0x0304, 0x0101},
	{0x0306, 0x2C2C},
	{0x0302, 0x0001},
	{0x030A, 0x0001},
	{0x0300, 0x0005},
	{0x0308, 0x000A},
	{0x0112, 0x0A0A},
	{0x3016, 0x0101}
};

msm_camera_i2c_reg_array pll_setup_880M_DPCM8[] = {
	{0x0300, 0x4},
	{0x0302, 0x1},
	{0x0304, 0x505},
	{0x0306, 0xB0B0},
	{0x0308, 0x8},
	{0x030A, 0x1},
	{0x0112, 0x0A08},
	{0x3016, 0x0101}
};

msm_camera_i2c_reg_array pll_setup_880M[] = {
	{0x0300, 0x4},
	{0x0302, 0x1},
	{0x0304, 0x505},
	{0x0306, 0xB0B0},
	{0x0308, 0xA},
	{0x030A, 0x1},
	{0x0112, 0x0A0A},
	{0x3016, 0x0101}
};
#ifdef NEW_PLL_CONFIG
/*
//PLL Configuration (Ext=25MHz, vt_pix_clk=170MHz, op_pix_clk=40MHz)
REG= 0x0300, 0x5	//VT_PIX_CLK_DIV=5
REG= 0x0302, 0x1	//VT_SYS_CLK_DIV=1
REG= 0x0304, 0x201	//PRE_PLL_CLK_DIV2=2	// PRE_PLL_CLK_DIV1=1
REG= 0x0306, 0x2022	//PLL_MULTIPLIER2=32	// PLL_MULTIPLIER1=34
REG= 0x0308, 0xA 	//OP_PIX_CLK_DIV=10
REG= 0x030A, 0x1 	//OP_SYS_CLK_DIV=1
DELAY=1
*/
msm_camera_i2c_reg_array pll_setup_recommended[] = {
		{0x0300, 0x5},
		{0x0302, 0x1},
		{0x0304, 0x0101},
		{0x0306, 0x102C},
		{0x0308, 0xA},
		{0x030A, 0x1},
		{0x3016, 0x0101}
};
#else
/*
 * //PLL Configuration (Ext=25MHz, vt_pix_clk=80MHz, op_pix_clk=40MHz)
REG= 0x0300, 0x5 //VT_PIX_CLK_DIV=5
REG= 0x0302, 0x1 //VT_SYS_CLK_DIV=1
REG= 0x0304,  0x0202//PRE_PLL_CLK_DIV2=2// PRE_PLL_CLK_DIV1=2
REG= 0x0306,  0x2020//PLL_MULTIPLIER2=32// PLL_MULTIPLIER1=32
REG= 0x0308, 0xA //OP_PIX_CLK_DIV=10
REG= 0x030A, 0x1 //OP_SYS_CLK_DIV=1
DELAY=1
 * */
msm_camera_i2c_reg_array pll_setup_recommended[] = {
	{0x0300, 0x5},
	{0x0302, 0x1},
	{0x0304, 0x0101},
	{0x0306, 0x102C},
	{0x0308, 0xA},
	{0x030A, 0x1}
};
#endif

msm_camera_i2c_reg_array defect_correction[] = {
	//{0x30FE, 0x10},	/* Read, then write 0 to the mark */
	{0x31E0, 0x0781},
	{0x3F00, 0x004F},
	{0x3F02, 0x0125},
	{0x3F04, 0x20},
	{0x3F06, 0x40},
	{0x3F08, 0x70},
	{0x3F0A, 0x0101},
	{0x3F0C, 0x0302},
	{0x3F1E, 0x22},
	{0x3F1A, 0x01FF},
	{0x3F14, 0x0101},
	{0x3F44, 0x0707},
	{0x3F18, 0x011E},
	{0x3F12, 0x0303},
	{0x3F42, 0x1511},
	{0x3F16, 0x011E},
	{0x3F10, 0x0505},
	{0x3F40, 0x1511}
};


#ifdef NEW_PLL_CONFIG
/*
220MHz
*/
msm_camera_i2c_reg_array res13_4208_3120_recommended[] = {
		{0x0344, 0x10},
		{0x0348, 0x107F},
		{0x0346, 0x10},
		{0x034A, 0xC3F},
		{0x034C, 0x1070},
		{0x034E, 0xC30},
		{0x3040, 0x0041},
		{0x0112, 0xA0A},
		{0x3172, 0x0206},
		{0x317A, 0x416E},
		{0x3F3C, 0x0003},
		{0x0342, 0x3000},
		{0x0340, 0x0DFC},
		{0x0202, 0xC4D},
		{0x0400, 0x0},		//Scale Configuration
		{0x0404, 0x10}
};
#else
/*//Output size (Pixel address must start with EVEN and end with ODD!)
REG=0x0344, 0x10	//X_ADDR_START 16
REG=0x0348, 0x107F	//X_ADDR_END 4223
REG=0x0346, 0x10	//Y_ADDR_START 16
REG=0x034A, 0xC3F	//Y_ADDR_END 3135
REG=0x034C, 0x1070	//X_OUTPUT_SIZE 4208
REG=0x034E, 0xC30	//Y_OUTPUT_SIZE 3120
REG=0x3040, 0x41	//X_BIN, X_ODD_INC, Y_ODD_INC
REG=0x0112, 0xA0A	//CCP_DATA_FORMAT

//Timing Configuration
REG=0x0342, 0x13D4	//LINE_LENGTH_PCK 2538
REG=0x0340, 0xC4E	//FRAME_LENGTH_LINES 3150
REG=0x0202, 0x410 	//COARSE_INTEGRATION_TIME 1040
 *
 *
 * */
/*essen rev1*/
msm_camera_i2c_reg_array res13_4208_3120_recommended[] = {
	{0x0344, 0x10},
	{0x0348, 0x107F},
	{0x0346, 0x10},
	{0x034A, 0xC3F},
	{0x034C, 0x1070},
	{0x034E, 0xC30},
	{0x3040, 0x0041},
	{0x0112, 0xA0A},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x3000},
	{0x0340, 0x0DFC},
	{0x0202, 0xC4D},
	{0x0400, 0x0},		//Scale Configuration
	{0x0404, 0x10}
};


#endif


msm_camera_i2c_reg_array res13_4224_3136_border_reg_array[] = {
	{0x0344, 0x8},
	{0x0348, 0x1087},
	{0x0346, 0x8},
	{0x034A, 0xC47},
	{0x034C, 0x1080},
	{0x034E, 0xC40},
	{0x3040, 0x0041},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1230},
	{0x0340, 0xC52},
	{0x0202, 0xC2E}
};

msm_camera_i2c_reg_array res13_4208_3120_DPCM8_reg_array[] = {
	{0x0344, 0x10},
	{0x0348, 0x107F},
	{0x0346, 0x10},
	{0x034A, 0xC3F},
	{0x034C, 0x1070},
	{0x034E, 0xC30},
	{0x3040, 0x0041},
	{0x0112, 0x0A08},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1230},
	{0x0340, 0xC4E},
	{0x0202, 0xC3C}
};

msm_camera_i2c_reg_array res13_4208_3120_reg_array[] = {
	{0x0344, 0x10},
	{0x0348, 0x107F},
	{0x0346, 0x10},
	{0x034A, 0xC3F},
	{0x034C, 0x1070},
	{0x034E, 0xC30},
	{0x3040, 0x0041},
	{0x0112, 0x0A0A},
	{0x0112, 0x0A0A},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1230},
	{0x0340, 0xC4E},
	{0x0202, 0xC3C}
};

msm_camera_i2c_reg_array res3_4208_3120_Ybin2_Xscale2_reg_array[] = {
	{0x0344, 0x10  },
	{0x0348, 0x107D},
	{0x0346, 0x10},
	{0x034A, 0xC3D },
	{0x034C, 0x838 },
	{0x034E, 0x618 },
	{0x3040, 0x43},
	{0x3172, 0x0206},
	{0x317A, 0x516E},
	{0x3F3C, 0x0003},
	{0x0400, 0x1},
	{0x0404, 0x20},
	{0x0342, 0x1200},
	{0x0340, 0xC6E},
	{0x0202, 0xC4F}
};

#ifdef NEW_PLL_CONFIG
/**
 * Output size (Pixel address must start with EVEN and end with ODD!)
REG=0x0344, 0xC8 //X_ADDR_START 200
REG=0x0348, 0xFC5 //X_ADDR_END 4037
REG=0x0346, 0x1F0 //Y_ADDR_START 496
REG=0x034A, 0xA5D //Y_ADDR_END 2653
REG=0x034C, 0x780 //X_OUTPUT_SIZE 1920
REG=0x034E, 0x438 //Y_OUTPUT_SIZE 1080
REG=0x3040, 0x8C3 //X_BIN, X_ODD_INC, Y_ODD_INC
//Binning Configuration
REG=0x3172, 0x226 //DIGBIN_ENABLE
REG=0x317A, 0x516E //SF_BIN_ENABLE
REG=0x3F3C, 0x3 //SF_BIN_ENABLE
//Scale Configuration
REG=0x0400, 0x0 //Scaling Enabling: 0= disable, 1= x-dir
REG=0x0404, 0x10 //Scale_M = 16
//Timing Configuration
REG=0x0342, 0x121A//LINE_LENGTH_PCK 2317
REG=0x0340, 0xE54//FRAME_LENGTH_LINES 3668
REG=0x0202, 0x899
 */
msm_camera_i2c_reg_array res1080_3840_2160_Ybin2_Xscale2_reg_array[] = {
	{0x0344, 0xC8 },
	{0x0348, 0xFC7 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5D },
	{0x034C, 0x780 },
	{0x034E, 0x438 },
	{0x3040, 0x0043},
	{0x3172, 0x0206},
	{0x317A, 0x516E},
	{0x3F3C, 0x0003},
	{0x0400, 0x0001},
	{0x0404, 0x20},
	{0x0342, 0x176C},
	{0x0340, 0x0E55},
	{0x0202, 0x0E55}
};
#else
msm_camera_i2c_reg_array res1080_3840_2160_Ybin2_Xscale2_reg_array[] = {
	{0x0344, 0xC8 },
	{0x0348, 0xFC7 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5D },
	{0x034C, 0x780 },
	{0x034E, 0x438 },
	{0x3040, 0x43},
	{0x3172, 0x0206},
	{0x317A, 0x516E},
	{0x3F3C, 0x0003},
	{0x0400, 0x1},
	{0x0404, 0x20},
	{0x0342, 0x1200},
	{0x0340, 0xC6E},
	{0x0202, 0xC6E}
};
#endif

msm_camera_i2c_reg_array res3_4208_3120_Ybin2_XBin2_reg_array[] = {
	{0x0344, 0x10},
	{0x0348, 0x107D},
	{0x0346, 0x10},
	{0x034A, 0xC3D},
	{0x034C, 0x838},
	{0x034E, 0x618},
	{0x3040, 0x8C3},
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3},
	{0x0400, 0x0},
	{0x0404, 0x10}
};

msm_camera_i2c_reg_array res1080_3840_2160_Ybin2_XBin2_reg_array[] = {
	{0x0344, 0xC8 },
	{0x0348, 0xFC5},
	{0x0346, 0x1F0},
	{0x034A, 0xA5D},
	{0x034C, 0x780},
	{0x034E, 0x438},
	{0x3040, 0x8C3 },
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3},
	{0x0400, 0x0},
	{0x0404, 0x10},
	{0x0342, 0x1200},
	{0x0340, 0xC6E},
	{0x0202, 0xC6E}
};

msm_camera_i2c_reg_array res720_3840_2160_Yskip3_XScale3_reg_array[] = {
	{0x0344, 0xC8},
	{0x0348, 0xFC7 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5B },
	{0x034C, 0x500 },
	{0x034E, 0x2D0 },
	{0x3040, 0x45},
	{0x3172, 0x0206},
	{0x317A, 0x516E},
	{0x3F3C, 0x0003},
	{0x0400, 0x1},
	{0x0404, 0x30},
	{0x0342, 0x1200},
	{0x0340, 0xC6E},
	{0x0202, 0xC4F }
};

msm_camera_i2c_reg_array res720_3840_2160_Ybin3_Xbin3_reg_array[] = {
	{0x0344, 0xC8},
	{0x0348, 0xFC3},
	{0x0346, 0x1F0},
	{0x034A, 0xA5B},
	{0x034C, 0x500},
	{0x034E, 0x2D0},
	{0x3040, 0x945 },
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3},
	{0x0400, 0x0},
	{0x0404, 0x10},
	{0x0342, 0x1200},
	{0x0340, 0xC6E},
	{0x0202, 0xC4F}
};

msm_camera_i2c_reg_array res4k_3840_2160_reg_array[] = {
	{0x0344, 0xC8},
	{0x0348, 0xFC7 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5F },
	{0x034C, 0xF00 },
	{0x034E, 0x870 },
	{0x3040, 0x0041},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1512},
	{0x0340, 0x87E},
	{0x0202, 0x8AF}
};

msm_camera_i2c_reg_array res4k_4096_2160_reg_array[] = {
	{0x0344, 0x48},
	{0x0348, 0x1047},
	{0x0346, 0x1F0 },
	{0x034A, 0xA5F },
	{0x034C, 0x1000},
	{0x034E, 0x870 },
	{0x3040, 0x0041},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1512},
	{0x0340, 0x87E},
	{0x0202, 0x8AF }
};

cam_reg_array common_stream[] = {
	{ correction_recommend, ARRAY_COUNT(correction_recommend), 2 },
	{ pixel_timing_recommended, ARRAY_COUNT(pixel_timing_recommended), 2 },
	{ analog_setup_recommended, ARRAY_COUNT(analog_setup_recommended), 2 }
};
cam_reg_array stream_13m_30fps_border[] = {
	{ mipi_timing_max, ARRAY_COUNT(mipi_timing_max), 2 },
	{ pll_setup_max, ARRAY_COUNT(pll_setup_max), 2 },
	{ res13_4224_3136_border_reg_array, ARRAY_COUNT(res13_4224_3136_border_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

cam_reg_array stream_13m_30fps_DPCM8[] = {
	{ mipi_timing_880M_DPCM8, ARRAY_COUNT(mipi_timing_880M_DPCM8), 2 },
	{ pll_setup_880M_DPCM8, ARRAY_COUNT(pll_setup_880M_DPCM8), 2 },
	{ res13_4208_3120_DPCM8_reg_array, ARRAY_COUNT(res13_4208_3120_DPCM8_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

msm_camera_i2c_reg_array stream_13m_30fps_custom[] = {
	{0x0342, 0x1230},
	{0x0340, 0xC4E},
	{0x0202, 0xC2E}
};


cam_reg_array stream_13m_30fps[] = {
	{ mipi_timing_recommended, ARRAY_COUNT(mipi_timing_recommended), 2 },
	{ pll_setup_recommended, ARRAY_COUNT(pll_setup_recommended), 2 },
	{ res13_4208_3120_recommended, ARRAY_COUNT(res13_4208_3120_recommended), 2 },
	{slave_mode_control_reg_array,	ARRAY_COUNT(slave_mode_control_reg_array),2}
//	{ stream_13m_30fps_custom, ARRAY_COUNT(stream_13m_30fps_custom), 2 },
//	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};


msm_camera_i2c_reg_array stream_13m_24fps_custom[] = {
	{0x0342, 0x165C},
	{0x0340, 0xC80},
	{0x0202, 0xC50}
};
cam_reg_array stream_13m_24fps[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res13_4208_3120_reg_array, ARRAY_COUNT(res13_4208_3120_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_13m_24fps_custom, ARRAY_COUNT(stream_13m_24fps_custom), 2 }
};

msm_camera_i2c_reg_array stream_13m_15fps_custom[] = {
	{0x0342, 0x165C},
	{0x0340, 0x1402},
	{0x0202, 0x1404}
};
cam_reg_array stream_13m_15fps[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res13_4208_3120_reg_array, ARRAY_COUNT(res13_4208_3120_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_13m_15fps_custom, ARRAY_COUNT(stream_13m_15fps_custom), 2 }
};

cam_reg_array stream_3M_30fps_HQ[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res3_4208_3120_Ybin2_Xscale2_reg_array, ARRAY_COUNT(res3_4208_3120_Ybin2_Xscale2_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};


 /*
 * cam_reg_array CamOpenRegs[] = {
	{start_reg_array,			ARRAY_COUNT(start_reg_array),				1 },
	{stop_reg_array,			ARRAY_COUNT(stop_reg_array),				1 },
	{correction_recommend,		ARRAY_COUNT(correction_recommend),			2 },
	{pll_setup_recommended,		ARRAY_COUNT(pll_setup_recommended),			2 },
	{pixel_timing_recommended,	ARRAY_COUNT(pixel_timing_recommended),		2 },
	{analog_setup_recommended,	ARRAY_COUNT(analog_setup_recommended),		2 },
	{mipi_timing_recommended,	ARRAY_COUNT(mipi_timing_recommended),		2 },
	{res13_4208_3120_recommended,	ARRAY_COUNT(res13_4208_3120_recommended),		2 },
	//{defect_correction,			ARRAY_COUNT(defect_correction),				2 },
	//{reset_reg_array,			ARRAY_COUNT(reset_reg_array),				2 },
	{slave_mode_control_reg_array,	ARRAY_COUNT(slave_mode_control_reg_array),2}
};
 */

cam_reg_array stream_1080p_30fps_HQ[] = {
	//{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	//{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ mipi_timing_recommended, ARRAY_COUNT(mipi_timing_recommended), 2 },
	{ pll_setup_recommended, ARRAY_COUNT(pll_setup_recommended), 2 },
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array, ARRAY_COUNT(res1080_3840_2160_Ybin2_Xscale2_reg_array), 2 },
	{slave_mode_control_reg_array,	ARRAY_COUNT(slave_mode_control_reg_array),2}
	//{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

msm_camera_i2c_reg_array stream_1080p_60fps_HQ_custom[] = {
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array stream_1080p_60fps_HQ[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2},
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array, ARRAY_COUNT(res1080_3840_2160_Ybin2_Xscale2_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_1080p_60fps_HQ_custom, ARRAY_COUNT(stream_1080p_60fps_HQ_custom), 2 }
};

cam_reg_array stream_3M_30fps_LP[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res3_4208_3120_Ybin2_XBin2_reg_array, ARRAY_COUNT(res3_4208_3120_Ybin2_XBin2_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

cam_reg_array stream_1080p_30fps_LP[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res1080_3840_2160_Ybin2_XBin2_reg_array, ARRAY_COUNT(res1080_3840_2160_Ybin2_XBin2_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

msm_camera_i2c_reg_array stream_1080p_60fps_LP_custom[] = {
	{0x0342, 0x1200},
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array stream_1080p_60fps_LP[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res1080_3840_2160_Ybin2_XBin2_reg_array, ARRAY_COUNT(res1080_3840_2160_Ybin2_XBin2_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_1080p_60fps_LP_custom, ARRAY_COUNT(stream_1080p_60fps_LP_custom), 2 }
};

cam_reg_array stream_720p_30fps_HQ[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res720_3840_2160_Yskip3_XScale3_reg_array, ARRAY_COUNT(res720_3840_2160_Yskip3_XScale3_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

msm_camera_i2c_reg_array stream_720p_60fps_HQ_custom[] = {
	{0x0340, 0x636},
	{0x0202, 0x631}
};
cam_reg_array stream_720p_60fps_HQ[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res720_3840_2160_Yskip3_XScale3_reg_array, ARRAY_COUNT(res720_3840_2160_Yskip3_XScale3_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_720p_60fps_HQ_custom, ARRAY_COUNT(stream_720p_60fps_HQ_custom), 2 }
};

cam_reg_array stream_720p_30fps_LP[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res720_3840_2160_Ybin3_Xbin3_reg_array, ARRAY_COUNT(res720_3840_2160_Ybin3_Xbin3_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

msm_camera_i2c_reg_array stream_720p_60fps_LP_custom[] = {
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array stream_720p_60fps_LP[] = {
	{ mipi_timing_880M, ARRAY_COUNT(mipi_timing_880M), 2 },
	{ pll_setup_880M, ARRAY_COUNT(pll_setup_880M), 2 },
	{ res720_3840_2160_Ybin3_Xbin3_reg_array, ARRAY_COUNT(res720_3840_2160_Ybin3_Xbin3_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 },
	{ stream_720p_60fps_LP_custom, ARRAY_COUNT(stream_720p_60fps_LP_custom), 2 }
};

cam_reg_array stream_4k_uhd[] = {
	{ mipi_timing_recommended, ARRAY_COUNT(mipi_timing_recommended), 2 },
	{ pll_setup_recommended, ARRAY_COUNT(pll_setup_recommended), 2 },
	{ res4k_3840_2160_reg_array, ARRAY_COUNT(res4k_3840_2160_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};

cam_reg_array stream_4k_cinema[] = {
	{ mipi_timing_recommended, ARRAY_COUNT(mipi_timing_recommended), 2 },
	{ pll_setup_recommended, ARRAY_COUNT(pll_setup_recommended), 2 },
	{ res4k_4096_2160_reg_array, ARRAY_COUNT(res4k_4096_2160_reg_array), 2 },
	{ defect_correction, ARRAY_COUNT(defect_correction), 2 }
};
#endif /* LIGHT_INC_AR0835_35MM_CAM_H_ */
