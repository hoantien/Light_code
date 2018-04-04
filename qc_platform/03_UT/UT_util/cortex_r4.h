/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    cortex_r4.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-28-2016
 * @brief   This file contains definitions of the ARM Cortex-R4 CPU
 *
 ******************************************************************************/

#ifndef __CORTEX_R4_H__
#define __CORTEX_R4_H__

/******************************************************************************/
/**
 * Mock memory for unit test
 */
unsigned int MOCK_SPI0_BASE[32];
unsigned int MOCK_SPI1_BASE[32];
unsigned int MOCK_SPI2_BASE[32];

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define SCU_BASE		((uint32_t)0x02000000) /*!< SCU (4KB) base address */
#define TIMER_BASE		((uint32_t)0x02001000) /*!< TIMER (4KB) base address */
#define WDT_BASE		((uint32_t)0x02002000) /*!< WDT (4KB) base address */
#define UART_BASE		((uint32_t)0x02003000) /*!< UART (4KB) base address */
#define I2C0_BASE		((uint32_t)0x02004000) /*!< I2C0 (4KB) base address */
#define I2C1_BASE		((uint32_t)0x02005000) /*!< I2C1 (4KB) base address */
#define I2C2_BASE		((uint32_t)0x02006000) /*!< I2C2 (4KB) base address */
#define I2C3_BASE		((uint32_t)0x02007000) /*!< I2C3 (4KB) base address */
#define I2C4_BASE		((uint32_t)0x02008000) /*!< I2C4 (4KB) base address */
#define I2C5_BASE		((uint32_t)0x02009000) /*!< I2C5 (4KB) base address */
#define I2C6_BASE		((uint32_t)0x0200A000) /*!< I2C6 (4KB) base address */
#define I2C7_BASE		((uint32_t)0x0200B000) /*!< I2C7 (4KB) base address */
#define I2C8_BASE		((uint32_t)0x0200C000) /*!< I2C8 (4KB) base address */
#define I2C9_BASE		((uint32_t)0x0200D000) /*!< I2C9 (4KB) base address */
#define I2C10_BASE		((uint32_t)0x0200E000) /*!< I2C10 (4KB) base address */
#define I2C11_BASE		((uint32_t)0x0200F000) /*!< I2C11 (4KB) base address */
#define I2C12_BASE		((uint32_t)0x02010000) /*!< I2C12 (4KB) base address */
#define I2C13_BASE		((uint32_t)0x02011000) /*!< I2C13 (4KB) base address */
#define I2C14_BASE		((uint32_t)0x02012000) /*!< I2C14 (4KB) base address */
#define I2C15_BASE		((uint32_t)0x02013000) /*!< I2C15 (4KB) base address */
#define I2C16_BASE		((uint32_t)0x02014000) /*!< I2C16 (4KB) base address */
#define I2C17_BASE		((uint32_t)0x02015000) /*!< I2C17 (4KB) base address */
#define I2C18_BASE		((uint32_t)0x02016000) /*!< I2C18 (4KB) base address */
#define I2CS0_BASE		((uint32_t)0x02017000) /*!< I2CS0 (4KB) base address */
#define I2CS1_BASE		((uint32_t)0x02018000) /*!< I2CS1 (4KB) base address */
#define PWM0_BASE		((uint32_t)0x02019000) /*!< PWM0 (4KB) base address */
#define PWM1_BASE		((uint32_t)0x0201A000) /*!< PWM1 (4KB) base address */
#define PPG_BASE		((uint32_t)0x0201B000) /*!< PPG (4KB) base address */
#define SPI0_BASE		MOCK_SPI0_BASE /*!< SPI0 (4KB) base address */
#define SPI1_BASE		MOCK_SPI1_BASE /*!< SPI1 (4KB) base address */
#define SPI2_BASE		MOCK_SPI2_BASE /*!< SPI2 (4KB) base address */
#define CSLHS0_BASE		((uint32_t)0x0201F000) /*!< CSLHS0 (4KB) base address */
#define CSLHS1_BASE		((uint32_t)0x02020000) /*!< CSLHS1 (4KB) base address */
#define CSLHS2_BASE		((uint32_t)0x02021000) /*!< CSLHS2 (4KB) base address */
#define CSLHS3_BASE		((uint32_t)0x02022000) /*!< CSLHS3 (4KB) base address */
#define CSLHS4_BASE		((uint32_t)0x02023000) /*!< CSLHS4 (4KB) base address */
#define CSLHS5_BASE		((uint32_t)0x02024000) /*!< CSLHS5 (4KB) base address */
#define CSLDS0_BASE		((uint32_t)0x02025000) /*!< CSLDS0 (4KB) base address */
#define CSLDS1_BASE		((uint32_t)0x02026000) /*!< CSLDS1 (4KB) base address */
#define MIPI2AXI0_BASE	((uint32_t)0x02027000) /*!< MIPI2AXI0 (4KB) base address*/
#define MIPI2AXI1_BASE	((uint32_t)0x02028000) /*!< MIPI2AXI1 (4KB) base address*/
#define MIPI2AXI2_BASE	((uint32_t)0x02029000) /*!< MIPI2AXI2 (4KB) base address*/
#define MIPI2AXI3_BASE	((uint32_t)0x0202A000) /*!< MIPI2AXI3 (4KB) base address*/
#define MIPI2AXI4_BASE	((uint32_t)0x0202B000) /*!< MIPI2AXI4 (4KB) base address*/
#define MIPI2AXI5_BASE	((uint32_t)0x0202C000) /*!< MIPI2AXI5 (4KB) base address*/
#define AXI2MIPI0_BASE	((uint32_t)0x0202D000) /*!< AXI2MIPI0 (4KB) base address*/
#define AXI2MIPI1_BASE	((uint32_t)0x0202E000) /*!< AXI2MIPI1 (4KB) base address*/
#define GPIO_BASE		((uint32_t)0x0202F000) /*!< GPIO (4KB) base address */
#define DMAC_BASE		((uint32_t)0x02030000) /*!< DMAC (4KB) base address */
#define EFUSE_BASE		((uint32_t)0x02031000) /*!< EFUSE (4KB) base address */
#define VIC0_BASE		((uint32_t)0x02032000) /*!< VIC0 (4KB) base address */
#define VIC1_BASE		((uint32_t)0x02033000) /*!< VIC1 (4KB) base address */
#define DAPSYS_BASE		((uint32_t)0x02034000) /*!< DAPSYS (8KB) base address */
#define DDRP0_BASE		((uint32_t)0x02036000) /*!< DDRP (4KB) base address */
#define DDRP1_BASE		((uint32_t)0x02037000) /*!< DDRP (4KB) base address */
#define TXS0_BASE		((uint32_t)0x02040000) /*!< TXS0 (16KB) base address */
#define TXS1_BASE		((uint32_t)0x02050000) /*!< TXS1 (16KB) base address */
#define GPV2_BASE		((uint32_t)0x02100000) /*!< GPV2 (1MB) base address */
#define ROM_BASE		((uint32_t)0x02200000) /*!< ROM (512KB) base address */
#define RAM_BASE		((uint32_t)0x02280000) /*!< RAM (512KB) base address */
#define QSPI_BASE		((uint32_t)0x02300000) /*!< QSPI (16MB) base address */
#define CR4S_BASE		((uint32_t)0x03300000) /*!< CR4 (13MB) base address */
#define DDRC_BASE		((uint32_t)0x03700000) /*!< DDRC (4KB) base address */
#define DDR_BASE		((uint32_t)0x04000000) /*!< DDR (3832MB) base address */
#define DDR1_BASE		((uint32_t)0x04000000) /*!< DDR1 (3832MB) base address*/
#define DDR2_BASE		((uint32_t)0x04000000) /*!< DDR2 (3832MB) base address*/
#define DDR3_BASE		((uint32_t)0x04000000) /*!< DDR3 (3832MB) base address*/
#define DDR4_BASE		((uint32_t)0x04000000) /*!< DDR4 (3832MB) base address*/

/**
  * @}
  */

#endif /* __CORTEX_R4_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
