/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_com.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Jan-28-2016
 * @brief	This file contains expand of hal_com
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_com.h"
#include "board_config.h"
#include "assert.h"
#include "hal_vic.h"

/* Private typedef -----------------------------------------------------------*/
/*
 * @brief uart_reg_t
 *
 * COM UART registers
 */
typedef struct uart_reg
{
	union
	{
		__I  uint32_t RBR;		/* Receiver Buffer Register */
		__O  uint32_t THR;		/* Transmitter Holding Register */
		__IO uint32_t DLL;		/* Divisor Latch (Low) Register */
	};
	union
	{
		__IO uint32_t DLH;		/* Divisor Latch (High) Register */
		__IO uint32_t IER;		/* Interrupt Enable Register */
	};
	union
	{
		__I  uint32_t IIR;		/* Interrupt Identification Register */
		__O  uint32_t FCR;		/* FIFO Control Register */
	};
		__IO uint32_t LCR;		/* Line Control Register */
		__IO uint32_t MCR;		/* Modem Control Register */
		__I  uint32_t LSR;		/* Line Status Register */
		__I  uint32_t MSR;		/* Modem Status Register */
		__IO uint32_t SCR;		/* Scratchpad Register */
		__IO uint32_t LPDLL;	/* Low Power Divisor Latch (Low) Register */
		__IO uint32_t LPDLH;	/* Low Power Divisor Latch (High) Register */
		__I  uint32_t Rsvd1[2];	/* Reserved */
	union
	{
		__I  uint32_t SRBR[16];	/* Shadow Receive Buffer Register */
		__O  uint32_t STHR[16];	/* Shadow Transmit Holding Register */
	};
		__IO uint32_t FAR;		/* FIFO Access Register */
		__I  uint32_t TFR;		/* Transmit FIFO Read Register */
		__O  uint32_t RFW;		/* Receive FIFO Write Register */
		__I  uint32_t USR;		/* UART Status Register */
		__I  uint32_t TFL;		/* Transmit FIFO Level Register */
		__I  uint32_t RFL;		/* Receive FIFO Level Register */
		__O  uint32_t SRR;		/* Software Reset Register */
		__IO uint32_t SRTS;		/* Shadow Request to Send Register */
		__IO uint32_t SBCR;		/* Shadow Break Control Register */
		__IO uint32_t SDMAM;	/* Shadow DMA Mode Register */
		__IO uint32_t SFE;		/* Shadow FIFO Enable Register */
		__IO uint32_t SRT;		/* Shadow RCVR Trigger Register */
		__IO uint32_t STET;		/* Shadow TX Empty Trigger Register */
		__IO uint32_t HTX;		/* Halt TX Register */
		__O  uint32_t DMASA;	/* DMA Software Acknowledge Register */
		__IO uint32_t TCR;		/* Transceiver Control Register Register */
		__IO uint32_t DE_EN;	/* Driver Output Enable Register */
		__IO uint32_t RE_EN;	/* Receiver Output Enable Register */
		__IO uint32_t DET;		/* Driver Output Enable Timing Register */
		__IO uint32_t TAT;		/* TurnAround Timing Register */
		__IO uint32_t DLF;		/* Divisor Latch Fractional Value Register */
		__IO uint32_t RAR;		/* Receive Address Register */
		__IO uint32_t TAR;		/* Transmit Address Register */
		__IO uint32_t LCR_EXT;	/* Line Extended Control Register */
		__I  uint32_t Rsvd2[9];	/* Reserved */
		__I  uint32_t CPR;		/* Component Parameter Register */
		__I  uint32_t UCV;		/* UART Component Version Register */
		__I  uint32_t CTR;		/* Component Type Register */
} uart_reg_t;

/*
 * @brief uart_config_t
 *
 * COM configuration structure
 */
typedef struct uart_config
{
	uart_reg_t		*handle;	/* UART handle */
	void			(*data_received_handler)(uint8_t c); /* Interrupt handler */
} uart_config_t;

/* Private define ------------------------------------------------------------*/

/* Define for LCR register */
#define COM_LCR_DLAB				BIT7
#define COM_LCR_DLS_MASK			(BIT0 | BIT1)
#define COM_LCR_DLS_8BITS			0x03
/* Define for IER register */
#define COM_IER_RXNE				BIT0
#define COM_IER_THRE				BIT1
/* Define for LSR register */
#define COM_LSR_RXNE				BIT0
#define COM_LSR_THRE				BIT5
/* Transceiver timeout setting */
#define HAL_COM_TIMEOUT				10000000

/* UART0 handle */
#define UART0						((uart_reg_t *)UART_BASE)

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void UART0_IRQHandler(void);
static void hal_com_irq(uint8_t port_name);

/* Private variables ---------------------------------------------------------*/

/* COM configurations */
static uart_config_t	uart_configs[COM_MAX_IDX] =
{
	/* COM1 */
	{
		.handle = UART0,
		.data_received_handler = NULL_PTR
	}
};

/* Testing variable - --------------------------------------------------------*/
int hal_com_sendbyte_count;
int hal_com_readbyte_count;
int hal_com_init_count;
/* ---------------------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/*
 * @brief hal_com_init
 * Initializes independent COM channel
 */
void hal_com_init(hal_com_t *com)
{
	hal_com_init_count ++;
}

/*
 * @brief hal_com_sendbyte
 * Send one byte to hardware
 */
hal_com_return_t hal_com_sendbyte(hal_com_t *com)
{
	hal_com_sendbyte_count ++;
}

/*
 * @brief hal_com_readbyte
 * Read one byte from hardware
 */
hal_com_return_t hal_com_readbyte(hal_com_t *com)
{
	hal_com_readbyte_count++;
}

/*
 * @brief hal_com_enable_irq
 * Enable UART interrupt
 */
void hal_com_enable_irq(hal_com_t *com)
{
}

/*
 * @brief hal_com_disable_irq
 * Disable UART interrupt
 */
void hal_com_disable_irq(hal_com_t *com)
{
}

/*
 * @brief UART0_IRQHandler
 */
static void UART0_IRQHandler(void)
{
	hal_com_irq(COM1);
}

/* Private functions -------------------------------------------------------- */

/**
 * @brief hal_com_irq
 * Processes UART interrupt
 */
static void hal_com_irq(uint8_t port_name)
{
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
