/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    ar1335.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-16-2016
 * @brief   This file contains expand of the ar1335 driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "ar1335.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
msm_camera_i2c_reg_array_t start_reg_array[] =
{
	{ 0x0100, 0x01 },
};
msm_camera_i2c_reg_array_t enable_streaming[] =
{
	{ 0x3F3C, 0x0003 },
	{ 0x301A, 0x021C }
};
msm_camera_i2c_reg_array_t analog_control_on[] =
{
	{ 0x3F3C, 0x0003}
};
msm_camera_i2c_reg_array_t analog_control_off[] =
{
	{ 0x3F3C, 0x0002 },
	{ 0x3FE0, 0x0001 }
};
msm_camera_i2c_reg_array_t analog_control_reset[] =
{
	{ 0x3FE0, 0x0000}
};
msm_camera_i2c_reg_array_t stop_reg_array[] =
{
	{ 0x0100, 0x00 },
};
msm_camera_i2c_reg_array_t groupon_reg_array[] =
{
	{ 0x0104, 0x01 },
};
msm_camera_i2c_reg_array_t groupoff_reg_array[] =
{
	{ 0x0104, 0x00 },
};
msm_camera_i2c_reg_array_t slave_mode_control_reg_array[] =
{
	{ 0x301A, 0x0318 },	/* SW Standby */
	{ 0x3026, 0xFD7F },	/* Trigger pin select */
	{ 0x3158, 0xA000 },	/* Slave mode control */
	{ 0x301A, 0x031C },	/* Stream on */
};
msm_camera_i2c_reg_array_t slave_mode_enable_reg_array[] =
{
	{ 0x3158, 0xA000 },	/* Slave mode control */
};
msm_camera_i2c_reg_array_t slave_mode_disable_reg_array[] =
{
	{ 0x3158, 0x0000 },	/* Slave mode control */
};

iso_t iso_table[] =
{
	 {1.3125, 100},
	 {2.625, 200},
	 {5.25, 400},
	 {10.53, 800},
	 {21.05, 1600},
	 {42.1, 3200}
};

uint32_t iso_table_size = ARRAY_SIZE(iso_table);

msm_camera_i2c_reg_array_t reset_reg_array[] =
{
	{ 0x3F3C, 0x0003},
	{ 0x301A, 0x001C },
	{ 0x301A, 0x021C }
};
msm_camera_i2c_reg_array_t correction_recommend[] =
{
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
msm_camera_i2c_reg_array_t pixel_timing_recommended[] =
{
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
msm_camera_i2c_reg_array_t analog_setup_recommended_0[] =
{
	{ 0x3EB6, 0x4D   },
	{ 0x3EBC, 0xAA06 },
};

msm_camera_i2c_reg_array_t mipi_timing_max[] =
{
	{ 0x31B0, 0x0058 },
	{ 0x31B2, 0x002C },
	{ 0x31B4, 0x23D0 },
	{ 0x31B6, 0x140A },
	{ 0x31B8, 0x2413 },
	{ 0x31BA, 0x1C70 },
	{ 0x31BC, 0x860B }
};
msm_camera_i2c_reg_array_t mipi_timing_880M[] =
{
	{ 0x31B0, 0x4D },
	{ 0x31B2, 0x28 },
	{ 0x31B4, 0x230E },
	{ 0x31B6, 0x1348 },
	{ 0x31B8, 0x1C12 },
	{ 0x31BA, 0x185B },
	{ 0x31BC, 0x8509 }
};
msm_camera_i2c_reg_array_t mipi_timing_880M_DPCM8[] =
{
	{ 0x31B0, 0x5C },
	{ 0x31B2, 0x2E },
	{ 0x31B4, 0x2412 },
	{ 0x31B6, 0x142A },
	{ 0x31B8, 0x2413 },
	{ 0x31BA, 0x1C72 },
	{ 0x31BC, 0x860B }
};
msm_camera_i2c_reg_array_t mipi_timing_recommended[] =
{
	{ 0x31B0, 0x0031 },
	{ 0x31B2, 0x001E },
	{ 0x31B4, 0x1188 },
	{ 0x31B6, 0x1165 },
	{ 0x31B8, 0x1012 },
	{ 0x31BA, 0x0C28 },
	{ 0x31BC, 0x8284 }
};
msm_camera_i2c_reg_array_t pll_setup_max_0[] =
{
	{ 0x0300, 0x0005 },
	{ 0x0302, 0x0001 },
	{ 0x0304, 0x0101 },
	{ 0x0306, 0x2C2C },
	{ 0x0308, 0x000A },
	{ 0x030A, 0x0001 }
};
msm_camera_i2c_reg_array_t pll_setup_max_1[] =
{
	{ 0x0112, 0x0A0A },
	{ 0x3016, 0x0101 }
};
msm_camera_i2c_reg_array_t pll_setup_880M_DPCM8_0[] =
{
	{ 0x0300, 0x4 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x505 },
	{ 0x0306, 0xB0B0 },
	{ 0x0308, 0x8 },
	{ 0x030A, 0x1 }

};
msm_camera_i2c_reg_array_t pll_setup_880M_DPCM8_1[] =
{
	{ 0x0112, 0x0A08 },
	{ 0x3016, 0x0101 }
};
msm_camera_i2c_reg_array_t pll_setup_880M_0[] =
{
	{ 0x0300, 0x4 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x505 },
	{ 0x0306, 0xB0B0 },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};
msm_camera_i2c_reg_array_t pll_setup_880M_1[] =
{
	{ 0x0112, 0x0A0A },
	{ 0x3016, 0x0101 }
};
msm_camera_i2c_reg_array_t pll_setup_recommended[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x0202 },
	{ 0x0306, 0x2020 },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};
msm_camera_i2c_reg_array_t defect_correction_0[] =
{
	{ 0x30FE, 0x10 },	/* Read, then write 0 to the mark */
	{ 0x31E0, 0x0781 },
	{ 0x3F1E, 0x22   }
};
msm_camera_i2c_reg_array_t defect_correction_1[] =
{
	{ 0x3F00, 0x004F },
	{ 0x3F02, 0x0125 },
	{ 0x3F04, 0x20 },
	{ 0x3F06, 0x40 },
	{ 0x3F08, 0x70 },
	{ 0x3F0A, 0x0101 },
	{ 0x3F0C, 0x0302 }
};
msm_camera_i2c_reg_array_t defect_correction_2[] =
{
	{ 0x3F10, 0x0505 },
	{ 0x3F12, 0x0303 },
	{ 0x3F14, 0x0101 },
	{ 0x3F16, 0x011E },
	{ 0x3F18, 0x011E },
	{ 0x3F1A, 0x01FF }
};
msm_camera_i2c_reg_array_t defect_correction_3[] =
{
	{ 0x3F40, 0x1511 },
	{ 0x3F42, 0x1511 },
	{ 0x3F44, 0x0707 }
};
msm_camera_i2c_reg_array_t res13_4208_3120_recommended_0[] =
{
	{ 0x0340, 0xC4E },
	{ 0x0342, 0x13D4 },
	{ 0x0344, 0x10 },
	{ 0x0346, 0x10 },
	{ 0x0348, 0x107F },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1070 },
	{ 0x034E, 0xC30 },
};
msm_camera_i2c_reg_array_t res13_4208_3120_recommended_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x0112, 0xA0A },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0xC4D },
	{ 0x0400, 0x0 },		/* Scale Configuration */
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t res13_4224_3136_border_reg_array_0[] =
{
	{ 0x0340, 0xC52 },
	{ 0x0342, 0x1230 },
	{ 0x0344, 0x8 },
	{ 0x0346, 0x8 },
	{ 0x0348, 0x1087 },
	{ 0x034A, 0xC47 },
	{ 0x034C, 0x1080 },
	{ 0x034E, 0xC40 }
};
msm_camera_i2c_reg_array_t res13_4224_3136_border_reg_array_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0xC2E }
};
msm_camera_i2c_reg_array_t res13_4208_3120_DPCM8_reg_array_0[] =
{
	{ 0x0340, 0xC4E },
	{ 0x0342, 0x1230 },
	{ 0x0344, 0x10 },
	{ 0x0346, 0x10 },
	{ 0x0348, 0x107F },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1070 },
	{ 0x034E, 0xC30 }
};
msm_camera_i2c_reg_array_t res13_4208_3120_DPCM8_reg_array_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x0112, 0x0A08 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0xC3C }
};
msm_camera_i2c_reg_array_t res13_4208_3120_reg_array_0[] =
{
	{ 0x0340, 0xC4E },
	{ 0x0342, 0x1230 },
	{ 0x0344, 0x10 },
	{ 0x0346, 0x10 },
	{ 0x0348, 0x107F },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1070 },
	{ 0x034E, 0xC30 }
};
msm_camera_i2c_reg_array_t res13_4208_3120_reg_array_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x0112, 0x0A0A },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0xC3C }
};
msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_Xscale2_reg_array_0[] =
{
	{ 0x0340, 0xC6E },
	{ 0x0342, 0x1200 },
	{ 0x0344, 0x10 },
	{ 0x0346, 0x10 },
	{ 0x0348, 0x107D },
	{ 0x034A, 0xC3D },
	{ 0x034C, 0x838 },
	{ 0x034E, 0x618 }
};
msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_Xscale2_reg_array_1[] =
{
	{ 0x3040, 0x43 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x0003 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x0202, 0xC4F }
};
msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_Xscale2_reg_array_0[] =
{
	{ 0x0340, 0xC6E },
	{ 0x0342, 0x1200 },
	{ 0x0344, 0xC8 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0xFC7 },
	{ 0x034A, 0xA5D },
	{ 0x034C, 0x780 },
	{ 0x034E, 0x438 }
};
msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_Xscale2_reg_array_1[] =
{
	{ 0x3040, 0x43 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x0003 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x0202, 0xC6E }
};
msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_XBin2_reg_array_0[] =
{
	{ 0x0344, 0x10 },
	{ 0x0346, 0x10 },
	{ 0x0348, 0x107D },
	{ 0x034A, 0xC3D },
	{ 0x034C, 0x838 },
	{ 0x034E, 0x618 }
};
msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_XBin2_reg_array_1[] =
{
	{ 0x3040, 0x8C3 },
	{ 0x3172, 0x226 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_XBin2_reg_array_0[] =
{
	{ 0x0340, 0xC6E },
	{ 0x0342, 0x1200 },
	{ 0x0344, 0xC8 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0xFC5 },
	{ 0x034A, 0xA5D },
	{ 0x034C, 0x780 },
	{ 0x034E, 0x438 }
};
msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_XBin2_reg_array_1[] =
{
	{ 0x3040, 0x8C3 },
	{ 0x3172, 0x226 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 },
	{ 0x0202, 0xC6E }
};
msm_camera_i2c_reg_array_t res720_3840_2160_Yskip3_XScale3_reg_array_0[] =
{
	{ 0x0340, 0xC6E },
	{ 0x0342, 0x1200 },
	{ 0x0344, 0xC8 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0xFC7 },
	{ 0x034A, 0xA5B },
	{ 0x034C, 0x500 },
	{ 0x034E, 0x2D0 }
};
msm_camera_i2c_reg_array_t res720_3840_2160_Yskip3_XScale3_reg_array_1[] =
{
	{ 0x3040, 0x45 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x0003 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x30 },
	{ 0x0202, 0xC4F }
};
msm_camera_i2c_reg_array_t res720_3840_2160_Ybin3_Xbin3_reg_array_0[] =
{
	{ 0x0340, 0xC6E },
	{ 0x0342, 0x1200 },
	{ 0x0344, 0xC8 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0xFC3 },
	{ 0x034A, 0xA5B },
	{ 0x034C, 0x500 },
	{ 0x034E, 0x2D0 }
};
msm_camera_i2c_reg_array_t res720_3840_2160_Ybin3_Xbin3_reg_array_1[] =
{
	{ 0x3040, 0x945 },
	{ 0x3172, 0x226 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 },
	{ 0x0202, 0xC4F }
};
msm_camera_i2c_reg_array_t res4k_3840_2160_reg_array_0[] =
{
	{ 0x0340, 0x87E },
	{ 0x0342, 0x1512 },
	{ 0x0344, 0xC8 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0xFC7 },
	{ 0x034A, 0xA5F },
	{ 0x034C, 0xF00 },
	{ 0x034E, 0x870 }
};
msm_camera_i2c_reg_array_t res4k_3840_2160_reg_array_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0x8AF }
};
msm_camera_i2c_reg_array_t res4k_4096_2160_reg_array_0[] =
{
	{ 0x0340, 0x87E },
	{ 0x0342, 0x1512 },
	{ 0x0344, 0x48 },
	{ 0x0346, 0x1F0 },
	{ 0x0348, 0x1047 },
	{ 0x034A, 0xA5F },
	{ 0x034C, 0x1000 },
	{ 0x034E, 0x870 },
};
msm_camera_i2c_reg_array_t res4k_4096_2160_reg_array_1[] =
{
	{ 0x3040, 0x0041 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x0003 },
	{ 0x0202, 0x8AF }
};
msm_camera_i2c_reg_array_t analog_setup_recommended_1[] =
{
	{ 0x3EC0, 0x2E02 },
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
msm_camera_i2c_reg_array_t res13M_testing_recommended[] =
{
	{0x0344, 0x10  },
	{0x0348, 0x107F},
	{0x0346, 0x10  },
	{0x034A, 0xC3F },
	{0x034C, 0x1070},
	{0x034E, 0xC30 },
	{0x3040, 0x0041},
	{0x0112, 0x0A0A},
	{0x0112, 0x0A0A},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1230},
	{0x0340, 0xC4E },
	{0x0202, 0xC3C }
};
msm_camera_i2c_reg_array_t res1080_testing_recommended[] =
{
	{0x0344, 0xC8  },
	{0x0348, 0xFC5 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5D },
	{0x034C, 0x780 },
	{0x034E, 0x438 },
	{0x3040, 0x8C3 },
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3   },
	{0x0400, 0x0   },
	{0x0404, 0x10  },
	{0x0342, 0x1200},
	{0x0340, 0xC6E },
	{0x0202, 0xC6E }
};
msm_camera_i2c_reg_array_t res720_testing_recommended[] =
{
	{0x0344, 0xC8  },
	{0x0348, 0xFC3 },
	{0x0346, 0x1F0 },
	{0x034A, 0xA5B },
	{0x034C, 0x500 },
	{0x034E, 0x2D0 },
	{0x3040, 0x945 },
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3   },
	{0x0400, 0x0   },
	{0x0404, 0x10  },
	{0x0342, 0x1200},
	{0x0340, 0xC6E },
	{0x0202, 0xC4F }
};
msm_camera_i2c_reg_array_t res3M_testing_recommended[] =
{
	{0x0344, 0x10  },
	{0x0348, 0x107D},
	{0x0346, 0x10  },
	{0x034A, 0xC3D },
	{0x034C, 0x838 },
	{0x034E, 0x618 },
	{0x3040, 0x8C3 },
	{0x3172, 0x226 },
	{0x317A, 0x516E},
	{0x3F3C, 0x3   },
	{0x0400, 0x0   },
	{0x0404, 0x10  },
	{0x0342, 0x1200},
	{0x0340, 0xC6E },
	{0x0202, 0xC4F }
};
msm_camera_i2c_reg_array_t res4KUHD_testing_recommended[] =
{
	{0x0344, 0xC8  },
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
	{0x0340, 0x87E },
	{0x0202, 0x8AF }
};
msm_camera_i2c_reg_array_t res4KCinema_testing_recommended[] =
{
	{0x0344, 0x48  },
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
	{0x0340, 0x87E },
	{0x0202, 0x8AF }
};
#ifndef P2
cam_reg_array_t resolution_testing_reg_array[] =
{
	{res13M_testing_recommended, ARRAY_SIZE(res13M_testing_recommended),
		2, CAM_REG_NONCONTINUOUS},
	{res1080_testing_recommended, ARRAY_SIZE(res1080_testing_recommended),
		2, CAM_REG_NONCONTINUOUS},
	{res720_testing_recommended, ARRAY_SIZE(res720_testing_recommended),
		2, CAM_REG_NONCONTINUOUS},
	{res3M_testing_recommended, ARRAY_SIZE(res3M_testing_recommended),
		2, CAM_REG_NONCONTINUOUS},
	{res4KUHD_testing_recommended, ARRAY_SIZE(res4KUHD_testing_recommended),
		2, CAM_REG_NONCONTINUOUS},
	{res4KCinema_testing_recommended,
		ARRAY_SIZE(res4KCinema_testing_recommended), 2, CAM_REG_NONCONTINUOUS}
};
#endif

cam_reg_array_t stream_13m_30fps_border[] =
{
	{ mipi_timing_max, ARRAY_SIZE(mipi_timing_max),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_max_0, ARRAY_SIZE(pll_setup_max_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_max_1, ARRAY_SIZE(pll_setup_max_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res13_4224_3136_border_reg_array_0,
			ARRAY_SIZE(res13_4224_3136_border_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res13_4224_3136_border_reg_array_1,
			ARRAY_SIZE(res13_4224_3136_border_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS }
};
cam_reg_array_t stream_13m_30fps_DPCM8[] =
{
	{ mipi_timing_880M_DPCM8, ARRAY_SIZE(mipi_timing_880M_DPCM8),
			2, CAM_REG_CONTINUOUS },
	{ pll_setup_880M_DPCM8_0, ARRAY_SIZE(pll_setup_880M_DPCM8_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_DPCM8_1, ARRAY_SIZE(pll_setup_880M_DPCM8_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res13_4208_3120_DPCM8_reg_array_0, ARRAY_SIZE(res13_4208_3120_DPCM8_reg_array_0),
			2, CAM_REG_CONTINUOUS },
	{ res13_4208_3120_DPCM8_reg_array_1, ARRAY_SIZE(res13_4208_3120_DPCM8_reg_array_1),
				2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS }
};

msm_camera_i2c_reg_array_t stream_13m_30fps_custom[] =
{
	{0x0342, 0x1230},
	{0x0340, 0xC4E},
	{0x0202, 0xC2E}
};
cam_reg_array_t stream_13m_30fps[] =
{
	{ mipi_timing_max, ARRAY_SIZE(mipi_timing_max),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_max_0, ARRAY_SIZE(pll_setup_max_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_max_1, ARRAY_SIZE(pll_setup_max_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res13_4208_3120_reg_array_0, ARRAY_SIZE(res13_4208_3120_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res13_4208_3120_reg_array_1, ARRAY_SIZE(res13_4208_3120_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ stream_13m_30fps_custom, ARRAY_SIZE(stream_13m_30fps_custom),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS }
};
msm_camera_i2c_reg_array_t stream_13m_24fps_custom[] =
{
	{0x0342, 0x165C},
	{0x0340, 0xC80},
	{0x0202, 0xC50}
};
cam_reg_array_t stream_13m_24fps[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res13_4208_3120_reg_array_0, ARRAY_SIZE(res13_4208_3120_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res13_4208_3120_reg_array_1, ARRAY_SIZE(res13_4208_3120_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS },
	{ stream_13m_24fps_custom, ARRAY_SIZE(stream_13m_24fps_custom),
			2, CAM_REG_NONCONTINUOUS}
};
msm_camera_i2c_reg_array_t stream_13m_15fps_custom[] =
{
	{0x0342, 0x165C},
	{0x0340, 0x1402},
	{0x0202, 0x1404}
};
cam_reg_array_t stream_13m_15fps[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res13_4208_3120_reg_array_0, ARRAY_SIZE(res13_4208_3120_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res13_4208_3120_reg_array_1, ARRAY_SIZE(res13_4208_3120_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS },
	{ stream_13m_15fps_custom, ARRAY_SIZE(stream_13m_15fps_custom),
			2, CAM_REG_NONCONTINUOUS}
};
cam_reg_array_t stream_3M_30fps_HQ[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res3_4208_3120_Ybin2_Xscale2_reg_array_0,
			ARRAY_SIZE(res3_4208_3120_Ybin2_Xscale2_reg_array_0),
			2, CAM_REG_CONTINUOUS },
	{ res3_4208_3120_Ybin2_Xscale2_reg_array_1,
			ARRAY_SIZE(res3_4208_3120_Ybin2_Xscale2_reg_array_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS },
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS },
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS }
};
cam_reg_array_t stream_1080p_30fps_HQ[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array_0,
		ARRAY_SIZE(res1080_3840_2160_Ybin2_Xscale2_reg_array_0),
		2, CAM_REG_CONTINUOUS},
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array_1,
		ARRAY_SIZE(res1080_3840_2160_Ybin2_Xscale2_reg_array_1),
		2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
msm_camera_i2c_reg_array_t stream_1080p_60fps_HQ_custom[] =
{
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array_t stream_1080p_60fps_HQ[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array_0,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_Xscale2_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res1080_3840_2160_Ybin2_Xscale2_reg_array_1,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_Xscale2_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS},
	{ stream_1080p_60fps_HQ_custom,
		ARRAY_SIZE(stream_1080p_60fps_HQ_custom),
		2, CAM_REG_NONCONTINUOUS}
};

cam_reg_array_t stream_3M_30fps_LP[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res3_4208_3120_Ybin2_XBin2_reg_array_0,
			ARRAY_SIZE(res3_4208_3120_Ybin2_XBin2_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res3_4208_3120_Ybin2_XBin2_reg_array_1,
			ARRAY_SIZE(res3_4208_3120_Ybin2_XBin2_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
cam_reg_array_t stream_1080p_30fps_LP[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res1080_3840_2160_Ybin2_XBin2_reg_array_0,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_XBin2_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res1080_3840_2160_Ybin2_XBin2_reg_array_1,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_XBin2_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
msm_camera_i2c_reg_array_t stream_1080p_60fps_LP_custom[] =
{
	{0x0342, 0x1200},
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array_t stream_1080p_60fps_LP[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res1080_3840_2160_Ybin2_XBin2_reg_array_0,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_XBin2_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res1080_3840_2160_Ybin2_XBin2_reg_array_1,
			ARRAY_SIZE(res1080_3840_2160_Ybin2_XBin2_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS},
	{ stream_1080p_60fps_LP_custom,
		ARRAY_SIZE(stream_1080p_60fps_LP_custom),
		2, CAM_REG_NONCONTINUOUS}
};

cam_reg_array_t stream_720p_30fps_HQ[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res720_3840_2160_Yskip3_XScale3_reg_array_0,
			ARRAY_SIZE(res720_3840_2160_Yskip3_XScale3_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res720_3840_2160_Yskip3_XScale3_reg_array_1,
			ARRAY_SIZE(res720_3840_2160_Yskip3_XScale3_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
msm_camera_i2c_reg_array_t stream_720p_60fps_HQ_custom[] =
{
	{0x0340, 0x636},
	{0x0202, 0x631}
};
cam_reg_array_t stream_720p_60fps_HQ[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res720_3840_2160_Yskip3_XScale3_reg_array_0,
			ARRAY_SIZE(res720_3840_2160_Yskip3_XScale3_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res720_3840_2160_Yskip3_XScale3_reg_array_1,
			ARRAY_SIZE(res720_3840_2160_Yskip3_XScale3_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
				2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS},
	{ stream_720p_60fps_HQ_custom, ARRAY_SIZE(stream_720p_60fps_HQ_custom),
			2, CAM_REG_NONCONTINUOUS}
};
cam_reg_array_t stream_720p_30fps_LP[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res720_3840_2160_Ybin3_Xbin3_reg_array_0,
			ARRAY_SIZE(res720_3840_2160_Ybin3_Xbin3_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res720_3840_2160_Ybin3_Xbin3_reg_array_1,
			ARRAY_SIZE(res720_3840_2160_Ybin3_Xbin3_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
msm_camera_i2c_reg_array_t stream_720p_60fps_LP_custom[] =
{
	{0x0340, 0x636},
	{0x0202, 0x636}
};
cam_reg_array_t stream_720p_60fps_LP[] =
{
	{ mipi_timing_880M, ARRAY_SIZE(mipi_timing_880M),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_0, ARRAY_SIZE(pll_setup_880M_0),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_880M_1, ARRAY_SIZE(pll_setup_880M_1),
			2, CAM_REG_NONCONTINUOUS},
	{ res720_3840_2160_Ybin3_Xbin3_reg_array_0,
			ARRAY_SIZE(res720_3840_2160_Ybin3_Xbin3_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res720_3840_2160_Ybin3_Xbin3_reg_array_1,
			ARRAY_SIZE(res720_3840_2160_Ybin3_Xbin3_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS},
	{ stream_720p_60fps_LP_custom, ARRAY_SIZE(stream_720p_60fps_LP_custom),
			2, CAM_REG_NONCONTINUOUS}
};

cam_reg_array_t stream_4k_uhd[] =
{
	{ mipi_timing_recommended, ARRAY_SIZE(mipi_timing_recommended),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_recommended, ARRAY_SIZE(pll_setup_recommended),
			2, CAM_REG_CONTINUOUS},
	{ res4k_3840_2160_reg_array_0, ARRAY_SIZE(res4k_3840_2160_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res4k_3840_2160_reg_array_1, ARRAY_SIZE(res4k_3840_2160_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
cam_reg_array_t stream_4k_cinema[] =
{
	{ mipi_timing_recommended, ARRAY_SIZE(mipi_timing_recommended),
			2, CAM_REG_CONTINUOUS},
	{ pll_setup_recommended, ARRAY_SIZE(pll_setup_recommended),
			2, CAM_REG_CONTINUOUS},
	{ res4k_4096_2160_reg_array_0, ARRAY_SIZE(res4k_4096_2160_reg_array_0),
			2, CAM_REG_CONTINUOUS},
	{ res4k_4096_2160_reg_array_1, ARRAY_SIZE(res4k_4096_2160_reg_array_1),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_0, ARRAY_SIZE(defect_correction_0),
			2, CAM_REG_NONCONTINUOUS},
	{ defect_correction_1, ARRAY_SIZE(defect_correction_1),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_2, ARRAY_SIZE(defect_correction_2),
			2, CAM_REG_CONTINUOUS},
	{ defect_correction_3, ARRAY_SIZE(defect_correction_3),
			2, CAM_REG_CONTINUOUS}
};
#ifdef BLOCK_WRITE
cam_reg_array_t cam_open_default[] =
{
	{ start_reg_array, ARRAY_SIZE(start_reg_array),
			1, CAM_REG_NONCONTINUOUS},
	{ stop_reg_array, ARRAY_SIZE(stop_reg_array),
			1, CAM_REG_NONCONTINUOUS},
	{ correction_recommend, ARRAY_SIZE(correction_recommend),
			2, CAM_REG_NONCONTINUOUS},
	{ pll_setup_recommended, ARRAY_SIZE(pll_setup_recommended),
			2, CAM_REG_CONTINUOUS},
	{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended),
			2, CAM_REG_CONTINUOUS},
	{ analog_setup_recommended_0, ARRAY_SIZE(analog_setup_recommended_0),
			2, CAM_REG_NONCONTINUOUS},
	{ analog_setup_recommended_1, ARRAY_SIZE(analog_setup_recommended_1),
			2, CAM_REG_CONTINUOUS},
	{ mipi_timing_recommended, ARRAY_SIZE(mipi_timing_recommended),
			2, CAM_REG_CONTINUOUS},
	{ res13_4208_3120_recommended_0, ARRAY_SIZE(res13_4208_3120_recommended_0),
			2, CAM_REG_CONTINUOUS},
	{ res13_4208_3120_recommended_1, ARRAY_SIZE(res13_4208_3120_recommended_1),
			2, CAM_REG_NONCONTINUOUS},
	{ slave_mode_control_reg_array, ARRAY_SIZE(slave_mode_control_reg_array),
			2, CAM_REG_NONCONTINUOUS}
};
#else
msm_camera_i2c_reg_array_t analog_setup_recommended[] = {
	{ 0x3EB6, 0x4D   },
	{ 0x3EBC, 0xAA06 },
	{ 0x3EC0, 0x2E02 },
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
#ifdef P2
	{ 0x3EDE, 0x5005 }
#endif
};
msm_camera_i2c_reg_array_t res13_4208_3120_recommended[] = {
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
	{0x0342, 0x13D4},
	{0x0340, 0xC4E},
	{0x0202, 0xC4D},
	{0x0400, 0x0},		/*Scale Configuration. */
	{0x0404, 0x10}
};
msm_camera_i2c_reg_array_t pll_1200Mbps[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x0101 },
	{ 0x0306, 0x2C2C },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 },
	{ 0x0112, 0x0A0A },
	{ 0x3016, 0x0101 }
};

msm_camera_i2c_reg_array_t mipi_1200Mbs[] =
{
	{ 0x31B0, 0x0058 },
	{ 0x31B2, 0x002C },
	{ 0x31B4, 0x23D0 },
	{ 0x31B6, 0x140A },
	{ 0x31B8, 0x2413 },
	{ 0x31BA, 0x1C70 },
	{ 0x31BC, 0x860B }
};

msm_camera_i2c_reg_array_t output_1200Mbs[] =
{
		{ 0x0342, 0x1230 },
		{ 0x0340, 0xC4E }
};

msm_camera_i2c_reg_array_t pll_800Mbps[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x101 },
	{ 0x0306, 0x202C },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 },
	{ 0x0112, 0x0A0A }
};

msm_camera_i2c_reg_array_t mipi_800Mbs[] =
{
	{ 0x31B0, 0x0048 },
	{ 0x31B2, 0x0026 },
	{ 0x31B4, 0x130E },
	{ 0x31B6, 0x12E8 },
	{ 0x31B8, 0x1C12 },
	{ 0x31BA, 0x1452 },
	{ 0x31BC, 0x8488 }
};

msm_camera_i2c_reg_array_t output_800Mbs[] =
{
	{ 0x0342, 0x1230 },
	{ 0x0340, 0xC4E }
};
cam_reg_array_t cam_open_default[] =
{
	{ start_reg_array, ARRAY_SIZE(start_reg_array),
			1, CAM_REG_NONCONTINUOUS},
	{ stop_reg_array, ARRAY_SIZE(stop_reg_array),
			1, CAM_REG_NONCONTINUOUS},
	{ correction_recommend, ARRAY_SIZE(correction_recommend),
			2, CAM_REG_NONCONTINUOUS},
//	{ pll_setup_recommended, ARRAY_SIZE(pll_setup_recommended),
//			2, CAM_REG_NONCONTINUOUS},
	{ pll_800Mbps, ARRAY_SIZE(pll_800Mbps), 2, CAM_REG_NONCONTINUOUS },
	{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended),
			2, CAM_REG_NONCONTINUOUS},
	{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended),
			2, CAM_REG_NONCONTINUOUS},
//	{ mipi_timing_recommended, ARRAY_SIZE(mipi_timing_recommended),
//			2, CAM_REG_NONCONTINUOUS},
	{ mipi_800Mbs, ARRAY_SIZE(mipi_800Mbs), 2, CAM_REG_NONCONTINUOUS },
	{ res13_4208_3120_recommended, ARRAY_SIZE(res13_4208_3120_recommended),
			2, CAM_REG_NONCONTINUOUS},
	{ output_800Mbs, ARRAY_SIZE(output_800Mbs), 2, CAM_REG_NONCONTINUOUS },
};
#endif
uint32_t cam_open_default_size = ARRAY_SIZE(cam_open_default);

msm_camera_stream_type_t stream_types[] =
{
	{stream_13m_30fps,			4208,	3120,	30,	ARRAY_SIZE(stream_13m_30fps)},
	{stream_13m_30fps_border,	4224,	3136,	30,	ARRAY_SIZE(stream_13m_30fps_border)},
	{stream_13m_30fps_DPCM8,	4208,	3120,	30,	ARRAY_SIZE(stream_13m_30fps_DPCM8)},
	{stream_13m_24fps,			4208,	3120,	24,	ARRAY_SIZE(stream_13m_24fps)},
	{stream_13m_15fps,			4208,	3120,	15,	ARRAY_SIZE(stream_13m_15fps)},
	{stream_3M_30fps_HQ,		2104,	1560,	30,	ARRAY_SIZE(stream_3M_30fps_HQ)},
	{stream_3M_30fps_LP,		2104,	1560,	30,	ARRAY_SIZE(stream_3M_30fps_LP)},
	{stream_1080p_30fps_HQ,		1920,	1080,	30,	ARRAY_SIZE(stream_1080p_30fps_HQ)},
	{stream_1080p_30fps_LP,		1920,	1080,	30,	ARRAY_SIZE(stream_1080p_30fps_LP)},
	{stream_1080p_60fps_HQ,		1920,	1080,	60,	ARRAY_SIZE(stream_1080p_60fps_HQ)},
	{stream_1080p_60fps_LP,		1920,	1080,	60,	ARRAY_SIZE(stream_1080p_60fps_LP)},
	{stream_720p_30fps_HQ,		1280,	720,	30,	ARRAY_SIZE(stream_720p_30fps_HQ)},
	{stream_720p_30fps_LP,		1280,	720,	30,	ARRAY_SIZE(stream_720p_30fps_LP)},
	{stream_720p_60fps_HQ,		1280,	720,	60,	ARRAY_SIZE(stream_720p_60fps_HQ)},
	{stream_720p_60fps_LP,		1280,	720,	60,	ARRAY_SIZE(stream_720p_60fps_LP)},
	{stream_4k_uhd,				3840,	2160,	30,	ARRAY_SIZE(stream_4k_uhd)},
	{stream_4k_cinema,			4096,	2160,	30,	ARRAY_SIZE(stream_4k_cinema)},
};

uint32_t stream_types_size = ARRAY_SIZE(stream_types);
msm_camera_i2c_reg_array_t defect_correction[] =
{
	/*TODO: Before using this setting, a READ-MODIFY-WRITE is execute on
	 * 0x30FE, Bit 4 with value 0
	 */
	{ 0x31E0, 0x0781 },
	{ 0x3F00, 0x004F },
	{ 0x3F02, 0x0125 },
	{ 0x3F04, 0x20 },
	{ 0x3F06, 0x40 },
	{ 0x3F08, 0x70 },
	{ 0x3F0A, 0x0101 },
	{ 0x3F0C, 0x0302 },
	{ 0x3F1E, 0x22 },
	{ 0x3F1A, 0x01FF },
	{ 0x3F14, 0x0101 },
	{ 0x3F44, 0x0707 },
	{ 0x3F18, 0x011E },
	{ 0x3F12, 0x0303 },
	{ 0x3F42, 0x1511 },
	{ 0x3F16, 0x011E },
	{ 0x3F10, 0x0505 },
	{ 0x3F40, 0x1511 }
};
uint32_t defect_correction_size = ARRAY_SIZE(defect_correction);


msm_camera_i2c_reg_array_t defect_correction_panchromatic[] =
{
	{0x31E0, 0x0781},
	{0x3F00, 0x004F}, //BM_T0
	{0x3F02, 0x0125},//BM_T1
	{0x3F04, 0x20}, // if Ana_gain<2, use noise_floor0, multiply 2x gain by 64 sacle factor
	{0x3F06, 0x40}, // if 2<Ana_gain<4, use noise_floor1
	{0x3F08, 0x70}, // if 4<Ana_gain<7 use noise_floor2 and Ana_gain>7, use noise_floor3
	{0x3F0A, 0x0101}, // Define noise_floor0(low address) and noise_floor1(high address)
	{0x3F0C, 0x0302}, // Define noise_floor2 and noise_floor3
	{0x3F1E, 0x22 },
	//(low 7.75x*66ms < g*t)
	{0x3F1A, 0x0103}, //cross factor 2
	{0x3F14, 0x0101}, //single k factor 2
	{0x3F44, 0x0707}, //couple k factor 2
	//(medium 2x*66ms < g*t < 7.75x*66ms)
	{0x3F18, 0x0103}, //cross factor 1
	{0x3F12, 0x0303},//single k factor 1
	{0x3F42, 0x1511}, //couple k factor 1
	//(high g*t < 2x*66ms)
	{0x3F16, 0x0103}, //cross factor 0
	{0x3F10, 0x0505},//single k factor 0
	{0x3F40, 0x1511}, //couple k factor 0
};

uint32_t pan_defect_correction_size =
		ARRAY_SIZE(defect_correction_panchromatic);

msm_camera_i2c_reg_array_t corrections_recommended[] =
{
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
/* The ORG */
msm_camera_i2c_reg_array_t pll_220_40[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x201 },
	{ 0x0306, 0x212E },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};
msm_camera_i2c_reg_array_t pll_220_40_1[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x101 },
	{ 0x0306, 0x102E },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};

msm_camera_i2c_reg_array_t pll_220_110_1000[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x101 },
	{ 0x0306, 0x2A2E },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};

msm_camera_i2c_reg_array_t mipi_timing_1000[] =
{
	{ 0x31B0, 0x56 },
	{ 0x31B2, 0x2B },
	{ 0x31B4, 0x4310 },
	{ 0x31B6, 0x3389 },
	{ 0x31B8, 0x2013 },
	{ 0x31BA, 0x1C68 },
	{ 0x31BC, 0x860A }
};

msm_camera_i2c_reg_array_t fll_1000[] =
{
	{ 0x0342, 0x138C },
	{ 0x0340, 0xC4E }
};



msm_camera_i2c_reg_array_t pll_220_110_600[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x101 },
	{ 0x0306, 0x192E },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};

msm_camera_i2c_reg_array_t mipi_timing_600[] =
{
	{ 0x31B0, 0x3C },
	{ 0x31B2, 0x22 },
	{ 0x31B4, 0x220A },
	{ 0x31B6, 0x2206 },
	{ 0x31B8, 0x1412 },
	{ 0x31BA, 0x1041 },
	{ 0x31BC, 0x8386 }
};

msm_camera_i2c_reg_array_t fll_600[] =
{
	{ 0x0342, 0x2016 },
	{ 0x0340, 0xC5A }
};

msm_camera_i2c_reg_array_t pll_220_110[] =
{
	{ 0x0300, 0x5 },
	{ 0x0302, 0x1 },
	{ 0x0304, 0x101 },
	{ 0x0306, 0x2E2E },
	{ 0x0308, 0xA },
	{ 0x030A, 0x1 }
};
/* The ORG */
msm_camera_i2c_reg_array_t mipi_timing_220_40[] =
{
	{ 0x31B0, 0x31 },
	{ 0x31B2, 0x1E },
	{ 0x31B4, 0x1188 },
	{ 0x31B6, 0x1165 },
	{ 0x31B8, 0x1012 },
	{ 0x31BA, 0xC28 },
	{ 0x31BC, 0x8284 },
};

msm_camera_i2c_reg_array_t mipi_timing_220_40_2[] =
{
		{ 0x31B0, 0x31 },
		{ 0x31B2, 0x1E },
		{ 0x31B4, 0x1188 },

		{ 0x31B6, 0x142A },
		{ 0x31B8, 0x2413 },
		{ 0x31BA, 0x1C70 },
		{ 0x31BC, 0x868B },
};

msm_camera_i2c_reg_array_t mipi_timing_220_40_1[] =
{
	{ 0x31B0, 0x31},
	{ 0x31B2, 0x1E},
	{ 0x31B4, 0x1188},
	{ 0x31B6, 0x1164},
	{ 0x31B8, 0xC12},
	{ 0x31BA, 0xC28},
	{ 0x31BC, 0x8284}
};

msm_camera_i2c_reg_array_t mipi_timing_220_110[] =
{
//	{ 0x31B0, 0x5C },
//	{ 0x31B2, 0x2D },
//	{ 0x31B4, 0x4392 },
//	{ 0x31B6, 0x43CA },
//	{ 0x31B8, 0x2413 },
//	{ 0x31BA, 0x1C70 },
//	{ 0x31BC, 0x868B }
//
	{0x31B0, 0x64},
	{0x31B2, 0x2D},
	{0x31B4, 0x5392},
	{0x31B6, 0x53CA},
	{0x31B8, 0x2423},
	{0x31BA, 0x1C70},
	{0x31BC, 0x868B}
};

msm_camera_i2c_reg_array_t output_2048x1536_18fps[] =
{
	{ 0x0344, 0x48 },
	{ 0x0348, 0x1047 },
	{ 0x0346, 0x28 },
	{ 0x034A, 0xC25 },
	{ 0x034C, 0x800 },
	{ 0x034E, 0x600 },
	{ 0x3040, 0x43 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1924 },
	{ 0x0340, 0xEE2 },
	{ 0x0202, 0xC98 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x306E, 0x9090 }
};
msm_camera_i2c_reg_array_t output_2048x1536_30fps[] =
{
	{ 0x0344, 0x48 },
	{ 0x0348, 0x1047 },
	{ 0x0346, 0x28 },
	{ 0x034A, 0xC25 },
	{ 0x034C, 0x800 },
	{ 0x034E, 0x600 },
	{ 0x3040, 0x43 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1204 },
	{ 0x0340, 0xC76 },
	{ 0x0202, 0xBB8 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x306E, 0x9090 }
};
msm_camera_i2c_reg_array_t output_4160x3120_10fps_4208x3120[] =
{
	{ 0x0344, 0x10 },
	{ 0x0348, 0x107F },
	{ 0x0346, 0x10 },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1070 },
	{ 0x034E, 0xC30 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x3124 },
	{ 0x0340, 0xDFE },
	{ 0x0202, 0xC98 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
/* 4096x3120 */
/*
 * { 0x0344, 0x28 },
	{ 0x0348, 0x1027 },
	{ 0x0346, 0x10 },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1000 },
	{ 0x034E, 0xC30 },
 */
msm_camera_i2c_reg_array_t output_4160x3120_10fps[] =
{
	{ 0x0344, 0x28 },
	{ 0x0348, 0x1067 },
	{ 0x0346, 0x10 },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1040 },
	{ 0x034E, 0xC30 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x3024 },
	{ 0x0340, 0xDFE },
	//{ 0x0340, 0x6FF },
	//{ 0x0340, 0x1BFC },
	//{ 0x0340, 0x37F8 },
	{ 0x0202, 0xC98 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t output_4208x3120_10fps[] =
{
	{0x0344, 0x10  },
	{0x0348, 0x107F},
	{0x0346, 0x10  },
	{0x034A, 0xC3F },
	{0x034C, 0x1070},
	{0x034E, 0xC30 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x3024 },
	{ 0x0340, 0xDFE },
	//{ 0x0340, 0x6FF },
	//{ 0x0340, 0x1BFC },
	//{ 0x0340, 0x37F8 },
	{ 0x0202, 0xC98 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t output_4160x3120_30fps[] =
{
	{ 0x0344, 0x28 },
	{ 0x0348, 0x1067 },
	{ 0x0346, 0x10 },
	{ 0x034A, 0xC3F },
	{ 0x034C, 0x1040 },
	{ 0x034E, 0xC30 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1204 }, /* Single preview */
//	{ 0x0342, 0x2408 }, /* Dual preview */
	{ 0x0340, 0xC76 },
	{ 0x0202, 0xBB8 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};

msm_camera_i2c_reg_array_t output_4208x3120_30fps[] =
{
	{0x0344, 0x10  },
	{0x0348, 0x107F},
	{0x0346, 0x10  },
	{0x034A, 0xC3F },
	{0x034C, 0x1070},
	{0x034E, 0xC30 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1220 }, /* Single preview */
//	{ 0x0342, 0x2408 }, /* Dual preview */
	{ 0x0340, 0xC64 },
	{ 0x0202, 0xBB8 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t output_3840x2160_30fps[] =
{
	{ 0x0344, 0xC8 },
	{ 0x0348, 0xFC7 },
	{ 0x0346, 0x1F0 },
	{ 0x034A, 0xA5F },
	{ 0x034C, 0xF00 },
	{ 0x034E, 0x870 },
	{ 0x3040, 0x41 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1204 },
	{ 0x0340, 0xC76 },
	{ 0x0202, 0xBB8 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x0 },
	{ 0x0404, 0x10 }
};
msm_camera_i2c_reg_array_t output_1920x1080_30fps[] =
{
	{ 0x0344, 0xC8 },
	{ 0x0348, 0xFC7 },
	{ 0x0346, 0x1F0 },
	{ 0x034A, 0xA5D },
	{ 0x034C, 0x780 },
	{ 0x034E, 0x438 },
	{ 0x3040, 0x43 },
	{ 0x0112, 0xA0A },
	{ 0x0342, 0x1204 },
	{ 0x0340, 0xC76 },
	{ 0x0202, 0xBB8 },
	{ 0x3172, 0x206 },
	{ 0x317A, 0x416E },
	{ 0x3F3C, 0x3 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x306E, 0x9090 }
};


msm_camera_i2c_reg_array_t power_up[] =
{
		{ 0x0100, 0x01 },
		{ 0x0100, 0x00 }
};

msm_camera_i2c_reg_array_t num_of_lane[] =
{
	{ 0x31AE, 0x0204 }
};
msm_camera_i2c_reg_array_t config_new_1080p[] =
{
	{ 0x0344, 0xC8 },
	{ 0x0348, 0xFC7 },
	{ 0x0346, 0x1F0 },
	{ 0x034A, 0xA5D },
	{ 0x034C, 0x780 },
	{ 0x034E, 0x438 },
	{ 0x3040,  0x43 },
	{ 0x3172, 0x0206 },
	{ 0x317A, 0x516E },
	{ 0x3F3C, 0x0003 },
	{ 0x0400, 0x1 },
	{ 0x0404, 0x20 },
	{ 0x0342, 0x187C },
	{ 0x0340, 0x1B84 },
	{ 0x0202, 0xDC2 }
};

cam_reg_array_t config_4160x3120_10fps_mipi_400Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_40, ARRAY_SIZE(pll_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_40, ARRAY_SIZE(mipi_timing_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ output_4160x3120_10fps, ARRAY_SIZE(output_4160x3120_10fps), 2, CAM_REG_NONCONTINUOUS },
};
cam_reg_array_t config_4208x3120_10fps_mipi_400Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_40, ARRAY_SIZE(pll_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_40, ARRAY_SIZE(mipi_timing_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ output_4208x3120_10fps, ARRAY_SIZE(output_4208x3120_10fps), 2, CAM_REG_NONCONTINUOUS },
};
uint32_t config_13MP_400Mbs_size = ARRAY_SIZE(config_4160x3120_10fps_mipi_400Mbs);
cam_reg_array_t config_2048x1536_18fps_mipi_400Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_40, ARRAY_SIZE(pll_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_40, ARRAY_SIZE(mipi_timing_220_40), 2, CAM_REG_NONCONTINUOUS },
		{ output_2048x1536_18fps, ARRAY_SIZE(output_2048x1536_18fps), 2, CAM_REG_NONCONTINUOUS },
		{ defect_correction, ARRAY_SIZE(defect_correction), 2, CAM_REG_NONCONTINUOUS }
};

cam_reg_array_t config_4160x3120_30fps_mipi_1200Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_110, ARRAY_SIZE(pll_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_110, ARRAY_SIZE(mipi_timing_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ output_4160x3120_30fps, ARRAY_SIZE(output_4160x3120_30fps), 2, CAM_REG_NONCONTINUOUS },
};
cam_reg_array_t config_4208x3120_30fps_mipi_1200Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_110, ARRAY_SIZE(pll_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_110, ARRAY_SIZE(mipi_timing_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ output_4208x3120_30fps, ARRAY_SIZE(output_4208x3120_30fps), 2, CAM_REG_NONCONTINUOUS },
};
uint32_t config_13MP_1200Mbs_size = ARRAY_SIZE(config_4160x3120_30fps_mipi_1200Mbs);
cam_reg_array_t config_2048x1536_30fps_mipi_1200Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_110, ARRAY_SIZE(pll_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_110, ARRAY_SIZE(mipi_timing_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ output_2048x1536_30fps, ARRAY_SIZE(output_2048x1536_30fps), 2, CAM_REG_NONCONTINUOUS },
		{ defect_correction, ARRAY_SIZE(defect_correction), 2, CAM_REG_NONCONTINUOUS }
};

cam_reg_array_t config_3840x2160_30fps_mipi_1200Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_110, ARRAY_SIZE(pll_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_110, ARRAY_SIZE(mipi_timing_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ output_3840x2160_30fps, ARRAY_SIZE(output_3840x2160_30fps), 2, CAM_REG_NONCONTINUOUS },
		{ defect_correction, ARRAY_SIZE(defect_correction), 2, CAM_REG_NONCONTINUOUS }
};

cam_reg_array_t config_1920x1080_30fps_mipi_1200Mbs[] =
{
		{ power_up, ARRAY_SIZE(power_up), 1, CAM_REG_NONCONTINUOUS },
		{ corrections_recommended, ARRAY_SIZE(corrections_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ pll_220_110, ARRAY_SIZE(pll_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ pixel_timing_recommended, ARRAY_SIZE(pixel_timing_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ analog_setup_recommended, ARRAY_SIZE(analog_setup_recommended), 2, CAM_REG_NONCONTINUOUS },
		{ mipi_timing_220_110, ARRAY_SIZE(mipi_timing_220_110), 2, CAM_REG_NONCONTINUOUS },
		{ output_1920x1080_30fps, ARRAY_SIZE(output_1920x1080_30fps), 2, CAM_REG_NONCONTINUOUS },
		{ defect_correction, ARRAY_SIZE(defect_correction), 2, CAM_REG_NONCONTINUOUS }
};
#ifdef P2
#ifdef MIPI_SPEED_1500MHZ
cam_reg_array_t resolution_testing_reg_array[] =
{
	{ output_4160x3120_30fps, ARRAY_SIZE(output_4160x3120_30fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_2048x1536_30fps, ARRAY_SIZE(output_2048x1536_30fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_3840x2160_30fps, ARRAY_SIZE(output_3840x2160_30fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_1920x1080_30fps, ARRAY_SIZE(output_1920x1080_30fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_4208x3120_30fps, ARRAY_SIZE(output_4208x3120_30fps), 2, CAM_REG_NONCONTINUOUS }
};
#else /* MIPI_SPEED_400MHZ */
cam_reg_array_t resolution_testing_reg_array[] =
{
	{ output_4160x3120_10fps, ARRAY_SIZE(output_4160x3120_10fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_2048x1536_18fps, ARRAY_SIZE(output_2048x1536_18fps), 2, CAM_REG_NONCONTINUOUS },
	{ output_4208x3120_10fps, ARRAY_SIZE(output_4208x3120_10fps), 2, CAM_REG_NONCONTINUOUS },
};
#endif
#endif /* P2 */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
