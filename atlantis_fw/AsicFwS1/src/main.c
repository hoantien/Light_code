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
 * @brief   This file contains expand of the main application.
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hal_com.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "hal_ddr.h"
#include "hal_cache.h"
#include "flash.h"
#include "sb.h"

/* Exported define -----------------------------------------------------------*/
#define ASICFW_S2_ADDR			0x02280000
#define TCM_BASE_ADDR			0x03300000
#define	ACTIVE_FW_ID			0x060000
#define FLASH_SPACE_1			0x010000
#define FLASH_SPACE_2			0x070000
#define FLASH_SPACE_3			0x0C0000
#define SYSIMG_HDR_MAX_SIZE		512
#define SYSIMG_HDR_SIZE			340
#define	EFUSE_PUB_KEY			0x02031008
extern void udelay( unsigned long usec );
/* Private typedef -----------------------------------------------------------*/
/**
 * @brief firmware_header_t
 */
typedef struct s2_section_header
{
	uint32_t	ext_linear_addr;
	uint32_t	data_size;
} s2_section_header_t;

static const qspi_ctrl_mode_t qspi_ctrl =
{
	.pio1_ctrl.div_sclk			= 4,
	.pio1_ctrl.chip_select		= 1,
	.pio1_ctrl.sout1_default	= 1,
	.pio1_ctrl.sout2_default	= 1,
	.pio1_ctrl.sout3_default	= 1,
	.pio1_ctrl.sclk_default		= 1,
	.pio1_ctrl.post_cycle		= 1,
	.pio1_ctrl.pre_cycle		= 2,
	.pio1_ctrl.idle_cycle		= 2,

	.pp_ctrl.div_sclk			= 1,
	.pp_ctrl.chip_select		= 1,
	.pp_ctrl.sout1_default		= 1,
	.pp_ctrl.sout2_default		= 1,
	.pp_ctrl.sout3_default		= 1,
	.pp_ctrl.sclk_default		= 1,
	.pp_ctrl.post_cycle			= 1,
	.pp_ctrl.pre_cycle			= 2,
	.pp_ctrl.idle_cycle			= 2
};

/* Global function -----------------------------------------------------------*/
extern void jump_to_SRAM(void);
extern void timestamp_start(void);
extern unsigned int timestamp_ms(void);
extern void timerN_enable( int n );
extern void timerN_config( int n, unsigned int rate, int enable_int);
/* Local function ------------------------------------------------------------*/
/* Polynomial Used : a001 (hex) */

static uint8_t usart_data;
/* Local com driver configure */
static hal_com_t uart =
{
	.port_name		= COM1,
	.baudrate		= COM_BAUD_115200,
	.irq_handler	= NULL,
	.data			= &usart_data
};

unsigned long src_addr;

/*
 * _write
 * Retarget printf function with USART IP
 */
int _write(int file, char *ptr, int len)
{
	int n;
	for (n = 0; n < len; n++)
	{
		*(uart.data) = *ptr++;
		hal_com_sendbyte(&uart);
	}
	return n;
}
#ifndef BYPASS_AUTHENTICATE
static int proxy_read_flash(unsigned char *dst, unsigned long long src,
															unsigned long len)
{
	if(src >= ASICFW_S2_ADDR)
	{
		sb_proxy_read(dst, src, len);
	}
	else
	{
		if(flash_read(src, len, (uint8_t *)dst, QSPI_MODE) != FLASH_SUCCESS)
			return SB_CANNOT_READ;
	}
	return SB_OK;
}

static int proxy_write_flash(unsigned long long dst, const unsigned char *src,
															unsigned long len)
{
	return sb_proxy_write(dst, src, len);
}
#endif

static int authenticate_s2fw_flash(void)
{
	unsigned char active_fw_id;
	unsigned long s1_length;
	int stat = 1;
	unsigned char page_buf[512], sys_buf[1024];
	unsigned char *eFuse = (unsigned char *)EFUSE_PUB_KEY;
	unsigned char pub_key[260];
	memcpy(pub_key, eFuse, 128);
	memcpy(pub_key + 256, eFuse + 128, 4);
	/* Read firmware S1 header */
	flash_read(ACTIVE_FW_ID, 1, (unsigned char *) &active_fw_id, QSPI_MODE);
	printf("ACTIVE_FW_ID: %d\r\n", (unsigned int)active_fw_id);

	if(active_fw_id == 1)
		src_addr = FLASH_SPACE_1;
	else if (active_fw_id == 2)
		src_addr = FLASH_SPACE_2;
	else
		return 0;

	flash_read(src_addr + 24, 4, page_buf, QSPI_MODE);

	s1_length = page_buf[0] << 24;
	s1_length |= page_buf[1] << 16;
	s1_length |= page_buf[2] << 8;
	s1_length |= page_buf[3];
	printf("S1 firmware length: %x\r\n", (unsigned int)s1_length);
	/* memory map:
	 * +0			  +340           +S1 size       +340
	 * [S1 sys header][    S1 fw    ][S2 sys header][    S2 fw     ]
	 */
	printf("S2 offset: %x\r\n", (unsigned int)(src_addr + s1_length + 340));

	memset(page_buf, 0, sizeof(page_buf));
	memset(sys_buf, 0, sizeof(sys_buf));
#ifndef BYPASS_AUTHENTICATE
	unsigned long long first_address = 0;
	unsigned long version = 0;
	int elp_ret = elp_secureboot_phase0b(proxy_read_flash, proxy_write_flash,
									NULL,
									pub_key, sizeof(pub_key),
									(unsigned long long)(src_addr + s1_length + 340),
									&first_address,
									&version,
									&stat,
									page_buf, sizeof(page_buf),
									sys_buf, sizeof(sys_buf)
									);
	if(elp_ret != SB_OK)
	{
		printf("Authentication failed. SB error code [%x] \r\n", elp_ret);
		return 0;
	}
#endif
	return stat;
}

int load_fws2(void)
{
	unsigned char       s2_fw_header[4];
	unsigned char       s1_fw_header[4];

	unsigned int		s2_fw_offset = 0, s2_hdr_offset = 0;
	unsigned int        s1_fw_size = 0;
	unsigned int        s2_fw_size = 0;
	uint8_t				*code_ptr = (uint8_t *)ASICFW_S2_ADDR;

	/* Read firmware S1 header */
	flash_read(src_addr + 24, sizeof(s1_fw_header), s1_fw_header, QSPI_MODE);

	s1_fw_size |= (uint32_t)((uint32_t)(s1_fw_header[0]) << 24);
	s1_fw_size |= (uint32_t)((uint32_t)(s1_fw_header[1]) << 16);
	s1_fw_size |= (uint32_t)((uint32_t)(s1_fw_header[2]) << 8);
	s1_fw_size |= (uint32_t)((uint32_t)(s1_fw_header[3]));

	s2_hdr_offset = src_addr + SYSIMG_HDR_SIZE + s1_fw_size;

	printf("S1 firmware size: %x\r\n", s1_fw_size);
	printf("S2 header offset: %x\r\n", s2_hdr_offset);
	/* Read firmware S2 header*/
	flash_read(s2_hdr_offset + 24, sizeof(s2_fw_header), s2_fw_header, QSPI_MODE);

	s2_fw_size |= (uint32_t)((uint32_t)(s2_fw_header[0]) << 24);
	s2_fw_size |= (uint32_t)((uint32_t)(s2_fw_header[1]) << 16);
	s2_fw_size |= (uint32_t)((uint32_t)(s2_fw_header[2]) << 8);
	s2_fw_size |= (uint32_t)((uint32_t)(s2_fw_header[3]));

	printf("S2 firmware size: %x\r\n", s2_fw_size);

	memset(code_ptr, 0, s2_fw_size);

	s2_fw_offset = s2_hdr_offset + SYSIMG_HDR_SIZE;

	printf("S2 firmware offset without header: %x\r\n", s2_fw_size);
	/* Start Load S2 Firmware */
	s2_section_header_t section_header;
	uint32_t     flash_offset		= s2_fw_offset;
	uint8_t      tmp_buf[8]			= {0x00};
	int          section_count		= 0;
	unsigned int section_total_size	= 0;
	int          is_first_section	= 0;

	while(1)
	{
		memset(&section_header, 0x00, sizeof(s2_section_header_t));

		flash_read(flash_offset, sizeof(s2_section_header_t), \
			(uint8_t *) &tmp_buf, QSPI_MODE);

		if(!is_first_section)
		{
			is_first_section = 0xFFFF;
			section_header.ext_linear_addr = ASICFW_S2_ADDR;
		}
		else
		{
			section_header.ext_linear_addr |= (uint32_t)(tmp_buf[0] << 24);
			section_header.ext_linear_addr |= (uint32_t)(tmp_buf[1] << 16);
			section_header.ext_linear_addr |= (uint32_t)(tmp_buf[2] << 8);
			section_header.ext_linear_addr |= (uint32_t)(tmp_buf[3]);
		}

		section_header.data_size |= (uint32_t)(tmp_buf[4] << 24);
		section_header.data_size |= (uint32_t)(tmp_buf[5] << 16);
		section_header.data_size |= (uint32_t)(tmp_buf[6] << 8);
		section_header.data_size |= (uint32_t)(tmp_buf[7]);

		printf("Programming section %d address 0x%08X size %d\r\n", section_count,
								(unsigned int)section_header.ext_linear_addr,
								(unsigned int)section_header.data_size);


		flash_offset += sizeof(s2_section_header_t);
		code_ptr = (uint8_t *)(section_header.ext_linear_addr);

		flash_read(flash_offset, section_header.data_size, (uint8_t *) code_ptr, QSPI_MODE);

		flash_offset += section_header.data_size;
		/* increment section counter */
		section_count++;
		section_total_size += sizeof(s2_section_header_t) + section_header.data_size;

		/* check all section are loaded */
		if(section_total_size >= s2_fw_size)
		{
			printf("Done\r\n");
			break;
		}
	}
	printf("Loaded Time %d ms\r\n", timestamp_ms());
	printf("Jump to Firmware S2\r\n");

	/* Jump to firmware S2 */
	jump_to_SRAM();

	return 0;
}

/* Main function ------------------------------------------------------------*/
int main(void)
{
	int loop = 0;
	int ret = 0;
	int tmp = 0;

	timerN_config(2, 0, 0);
	timerN_enable(2);
#ifndef REWORK
	/* Bit [7:8] of OxA8 register have to be high */
	uint32_t scu_val = readl(SCU_BASE + 0xA8);
	scu_val |= 0x00000180;
	writel(SCU_BASE + 0xA8, scu_val);
#endif
	/* Power on sequence for ASIC2, 3 */
/*	hal_gpio_t asic2_pwr_on_seq;
	asic2_pwr_on_seq.port      = GPIO_PORTC;
	asic2_pwr_on_seq.pin       = GPIO_PIN_10;
	asic2_pwr_on_seq.direction = GPIO_DIR_OUT;
	hal_pwm_set_low(PWM1_CH1, PWM_PIN_POS);
	hal_pwm_set_low(PWM1_CH3, PWM_PIN_POS);
	hal_gpio_init(&asic2_pwr_on_seq);
	hal_gpio_set_high(&asic2_pwr_on_seq);
	udelay(11000);
	hal_pwm_set_high(PWM1_CH1, PWM_PIN_POS);
	udelay(1000);
	hal_pwm_set_high(PWM1_CH3, PWM_PIN_POS);


	hal_gpio_t asic3_pwr_on_seq;
	asic3_pwr_on_seq.port      = GPIO_PORTC;
	asic3_pwr_on_seq.pin       = GPIO_PIN_11;
	asic3_pwr_on_seq.direction = GPIO_DIR_OUT;
	hal_pwm_set_low(PWM1_CH1, PWM_PIN_NEG);
	hal_pwm_set_low(PWM1_CH3, PWM_PIN_NEG);
	hal_gpio_init(&asic3_pwr_on_seq);
	hal_gpio_set_high(&asic3_pwr_on_seq);
	udelay(11000);
	hal_pwm_set_high(PWM1_CH1, PWM_PIN_NEG);
	udelay(1000);
	hal_pwm_set_high(PWM1_CH3, PWM_PIN_NEG);
*/
	hal_gpio_t			gpio_irq;
	gpio_irq.direction	= GPIO_DIR_OUT;
	gpio_irq.port		= GPIO_PORTA;
	gpio_irq.pin		= 0;
	hal_gpio_init(&gpio_irq);
	hal_gpio_set_low(&gpio_irq);



	dcache_disable();
	hal_ddr_init();
	dcache_invalidate();
	dcache_enable();

    /* Power on sequence for ASIC2, 3 */
    hal_gpio_t asic2_pwr_on_seq;
    asic2_pwr_on_seq.port      = GPIO_PORTC;
    asic2_pwr_on_seq.pin       = GPIO_PIN_10;
    asic2_pwr_on_seq.direction = GPIO_DIR_OUT;
    hal_pwm_set_low(PWM1_CH1, PWM_PIN_POS);
    hal_pwm_set_low(PWM1_CH3, PWM_PIN_POS);
    hal_gpio_init(&asic2_pwr_on_seq);
    hal_gpio_set_high(&asic2_pwr_on_seq);
    udelay(11000);
    hal_pwm_set_high(PWM1_CH1, PWM_PIN_POS);
    udelay(1000);
    hal_pwm_set_high(PWM1_CH3, PWM_PIN_POS);


    hal_gpio_t asic3_pwr_on_seq;
    asic3_pwr_on_seq.port      = GPIO_PORTC;
    asic3_pwr_on_seq.pin       = GPIO_PIN_11;
    asic3_pwr_on_seq.direction = GPIO_DIR_OUT;
    hal_pwm_set_low(PWM1_CH1, PWM_PIN_NEG);
    hal_pwm_set_low(PWM1_CH3, PWM_PIN_NEG);
    hal_gpio_init(&asic3_pwr_on_seq);
    hal_gpio_set_high(&asic3_pwr_on_seq);
    udelay(11000);
    hal_pwm_set_high(PWM1_CH1, PWM_PIN_NEG);
    udelay(1000);
    hal_pwm_set_high(PWM1_CH3, PWM_PIN_NEG);

	/* Initialize UART, QSPI, FLASH */
	qspi_init((qspi_ctrl_mode_t *)&qspi_ctrl);
	flash_init();

	/* check build included Secure Boot or not */
	/* ISP mode, we need to verify firmware on flash */
	timestamp_start();
	ret = authenticate_s2fw_flash();
	tmp = timestamp_ms();
	if(ret)
	{
		printf("Authentication Time %d ms\r\n", tmp);
		printf("Loading S2 Firmware ...\r\n");
		/* Start load S2 firmware */
		load_fws2();

		printf("Failed\r\n");

	}
	printf("Authentication Failed !!!\r\n");
	/* Infinitive loop  */
	while(1)
		loop++;

	return 0;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
