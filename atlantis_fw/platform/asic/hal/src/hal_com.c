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
#include "cortex_r4.h"
#include "board_config.h"
#include "assert.h"
#include "hal_vic.h"
#include "hal_com.h"

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

/* Exported functions ------------------------------------------------------- */

/*
 * @brief hal_com_init
 * Initializes independent COM channel
 */
void hal_com_init(hal_com_t *com)
{
	 uart_reg_t	*uart;
	 uint32_t	baud_rate_divisor;

	 /* Assert input parameters */
	 assert_param(NULL_PTR != com);
	 assert_param(IS_COM_PORT(com->port_name));

	 uart = uart_configs[com->port_name].handle;

	 /* Calculate divisor */
	 /* Get the integer part of baud rate divisor */
	 baud_rate_divisor = CLOCK_133MHZ / (16 * com->baudrate);
	 /* Accommodate divisor for more accurate baud rate */
	 if((CLOCK_133MHZ - (baud_rate_divisor * 16 * com->baudrate)) >
	 	(8 * com->baudrate))
	 {
	 	baud_rate_divisor++;
	 }
	 /* Set DLAB bit to access DLL, DLH */
	 uart->LCR |= COM_LCR_DLAB;
	 /* Set baud rate divisor for the DLL register */
	 uart->DLL = baud_rate_divisor & 0xFF;
	 /* Set baud rate divisor for the DLH register */
	 uart->DLH = baud_rate_divisor >> 8;
	 /* Clear DLAB bit to access other registers */
	 uart->LCR &= (~COM_LCR_DLAB);
	 /* Disable FIFO to avoid Character Timeout interrupt */
	 uart->FCR = 0;

	 /* Set data length as 8 bits */
	 uart->LCR &= (~COM_LCR_DLS_MASK);
	 uart->LCR |= COM_LCR_DLS_8BITS;
}

/*
 * @brief hal_com_sendbyte
 * Send one byte to hardware
 */
hal_com_return_t hal_com_sendbyte(hal_com_t *com)
{
	uart_reg_t			*uart;
	uint32_t			timeout = HAL_COM_TIMEOUT;
	hal_com_return_t	ret;

	/* Assert input parameters */
	assert_param(NULL_PTR != com);
	assert_param(IS_COM_PORT(com->port_name));
	assert_param(NULL_PTR != com->data);

	uart = uart_configs[com->port_name].handle;

	/* Wait until THR is ready for a new character */
	while ((COM_LSR_THRE != (uart->LSR & COM_LSR_THRE)) && (--timeout));

	/* Check whether last transmission timeout */
	if (0 == timeout)
	{
		ret = COM_TIMEOUT;
	}
	else
	{
		/* Transmit data */
		uart->THR = (uint8_t)(*(com->data));
		ret = COM_OK;
	}

	return ret;
}

/*
 * @brief hal_com_readbyte
 * Read one byte from hardware
 */
hal_com_return_t hal_com_readbyte(hal_com_t *com)
{
	uart_reg_t			*uart;
	uint32_t			timeout = HAL_COM_TIMEOUT;
	hal_com_return_t	ret;

	/* Assert input parameters */
	assert_param(NULL_PTR != com);
	assert_param(IS_COM_PORT(com->port_name));
	assert_param(NULL_PTR != com->data);

	uart = uart_configs[com->port_name].handle;

	/* Wait until a new character is received */
	while ((COM_LSR_RXNE != (uart->LSR & COM_LSR_RXNE)) && (--timeout));

	/* Check whether last reception timeout */
	if (0 == timeout)
	{
		ret = COM_TIMEOUT;
	}
	else
	{
		/* Read data */
		*(com->data) = (uint8_t)uart->RBR;
		ret = COM_OK;
	}

	return ret;
}

/*
 * @brief hal_com_enable_irq
 * Enable UART interrupt
 */
void hal_com_enable_irq(hal_com_t *com)
{
	uart_reg_t	*uart;

	/* Assert input parameters */
	assert_param(NULL_PTR != com);
	assert_param(IS_COM_PORT(com->port_name));

	uart = uart_configs[com->port_name].handle;

	/* Clear interrupt pending */
	uart->USR;
	uart->RBR;
	/* Set interrupt handler */
	uart_configs[com->port_name].data_received_handler = com->irq_handler;
	/* Register interrupt handler*/
	vic_register_irq(UART_IRQn, UART0_IRQHandler);
	vic_set_priority_irq(UART_IRQn, 5);
	/* Enable RX interrupt */
	uart->IER |= COM_IER_RXNE;
}

/*
 * @brief hal_com_disable_irq
 * Disable UART interrupt
 */
void hal_com_disable_irq(hal_com_t *com)
{
	uart_reg_t	*uart;

	/* Assert input parameters */
	assert_param(NULL_PTR != com);
	assert_param(IS_COM_PORT(com->port_name));

	uart = uart_configs[com->port_name].handle;

	/* Disable RX interrupt */
	uart->IER &= (~COM_IER_RXNE);
	/* Clear interrupt handler */
	uart_configs[com->port_name].data_received_handler = NULL_PTR;
	/* Unregister interrupt handler*/
	vic_unregister_irq(UART_IRQn);
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
	if(NULL_PTR != uart_configs[port_name].data_received_handler)
	{
		/* Read data and clear interrupt pending */
		uint8_t c = (uint8_t)(uart_configs[port_name].handle->RBR);
		/* Call back function to transfer data to upper module */
		(*(uart_configs[port_name].data_received_handler))(c);
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
