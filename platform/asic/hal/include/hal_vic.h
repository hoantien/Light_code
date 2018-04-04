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
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __HAL_VIC_H__
#define __HAL_VIC_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief irq_n_type_t
 *
 * Interrupt line type
 */
typedef enum irq_n_type
{
	SPI_IRQn		= 0,
	DMA_IRQn		= 1,
	QSPI_IRQn		= 2,
	TIMER_IRQn		= 3,
	PWM0_IRQn		= 5,
	PWM1_IRQn		= 4,
	PPG_IRQn		= 6,
	SPIS1_IRQn		= 7,
	SPIS0_IRQn		= 8,
	I2CS1_IRQn		= 9,
	I2CS0_IRQn		= 10,
	I2C18_IRQn		= 11,
	I2C17_IRQn		= 12,
	I2C16_IRQn		= 13,
	I2C15_IRQn		= 14,
	I2C14_IRQn		= 15,
	I2C13_IRQn		= 16,
	I2C12_IRQn		= 17,
	I2C11_IRQn		= 18,
	I2C10_IRQn		= 19,
	I2C9_IRQn		= 20,
	I2C8_IRQn		= 21,
	I2C7_IRQn		= 22,
	I2C6_IRQn		= 23,
	I2C5_IRQn		= 24,
	I2C4_IRQn		= 25,
	I2C3_IRQn		= 26,
	I2C2_IRQn		= 27,
	I2C1_IRQn		= 28,
	I2C0_IRQn		= 29,
	CSID0_IRQn		= 30,
	CSID1_IRQn		= 31,
	CSIH0_IRQn		= 32,
	CSIH1_IRQn		= 33,
	CSIH2_IRQn		= 34,
	CSIH3_IRQn		= 35,
	CSIH4_IRQn		= 36,
	CSIH5_IRQn		= 37,
	MIPI0_TX_IRQn	= 38,
	MIPI1_TX_IRQn	= 39,
	MIPI0_RX_IRQn	= 40,
	MIPI1_RX_IRQn	= 41,
	MIPI2_RX_IRQn	= 42,
	MIPI3_RX_IRQn	= 43,
	MIPI4_RX_IRQn	= 44,
	MIPI5_RX_IRQn	= 45,
	EFUSEWD_IRQn	= 46,
	EFUSERD_IRQn	= 47,
	GPIO_IRQn		= 48,
	UART_IRQn		= 49,
	WDT_IRQn		= 50,
	WAKEUP_IRQn		= 51,
	IRQn_MAX_IDX	= 52
} irq_n_type_t;

typedef void (*irq_handler)(void);
/* Exported functions --------------------------------------------------------*/
/*
 * @brief vic_enable_irq
 * Enable the interrupt controller.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @return	none
 */
void vic_enable_irq(irq_n_type_t irq_n);
/*
 * @brief vic_disable_irq
 * Disable the interrupt controller.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @return	none
 */
void vic_disable_irq(irq_n_type_t irq_n);
/*
 * @brief vic_register_irq
 * Register a interrupt line which is mapped to peripheral.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @param handler	interrupt handler function.
 * @return	none
 */
void vic_register_irq(irq_n_type_t irq_n, irq_handler handler);
/*
 * @brief vic_unregister_irq
 * Unregister a interrupt line.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @return	none
 */
void vic_unregister_irq(irq_n_type_t irq_n);
/*
 * @brief vic_set_priority_irq
 * Set priority for a interrupt line.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @param prio	priority of a interrupt line.
 * @return	none
 */
void vic_set_priority_irq(irq_n_type_t irq_n, uint8_t prio);
/*
 * @brief vic_get_priority_irq
 * Get priority of a interrupt line.
 * @param irq_n	interrupt line is mapped to peripheral.
 * @return	priority of a interrupt line.
 */
uint8_t vic_get_priority_irq(irq_n_type_t irq_n);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_VIC_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
