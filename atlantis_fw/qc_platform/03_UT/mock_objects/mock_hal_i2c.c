/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_i2c.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-5-2016
 * @brief   This file contains functions of the I2C driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <malloc.h>
#include "cortex_r4.h"
#include "hal_i2c.h"
#include "hal_vic.h"
#include "assert.h"
#include "board_config.h"

/* Private define ------------------------------------------------------------*/
/* I2C clock speed definition */
#define I2C_CLK										BOARD_PCLOCK
/* I2C timeout */
#define I2C_TIMEOUT									5000000
/* I2C FIFO buffer size */
#define I2C_SOFT_FIFO_BUFFER_SIZE					((uint16_t)512)

/* Definition of bits in Control Register */
#define I2C_CON_TX_EMPTY_CONTROL_MASK				0x00000100
#define I2C_CON_SLAVE_DISABLE_MASK					0x00000040
#define I2C_CON_RESTART_EN_MASK						0x00000020
#define I2C_CON_10BITADDR_MASTER_MASK				0x00000010
#define I2C_CON_10BITADDR_SLAVE_MASK				0x00000008
#define I2C_CON_SPEED_MASK							0x00000006
#define I2C_CON_SPEED_STANDARD						0x00000002
#define I2C_CON_SPEED_FAST							0x00000004
#define I2C_CON_SPEED_HIGH							0x00000006
#define I2C_CON_MASTER_MODE_MASK					0x00000001

/* Definition of bits in Target Address Register */
#define I2C_TAR_10BITADDR_MASTER_MASK				0x00001000
#define I2C_TAR_GC_OR_START_MASK					0x00000400
#define I2C_TAR_TAR_MASK							0x000003FF

/* Definition of bits in Rx/Tx Data Buffer and Command Register */
#define I2C_DATA_CMD_RESTART_MASK					0x00000400
#define I2C_DATA_CMD_STOP_MASK						0x00000200
#define I2C_DATA_CMD_CMD_MASK						0x00000100
#define I2C_DATA_CMD_DAT_MASK						0x000000FF

/* Definition of bits in Interrupt Status Register */
#define I2C_INTR_STAT_STOP_DET_MASK					0x00000200
#define I2C_INTR_STAT_RX_DONE_MASK					0x00000080
#define I2C_INTR_STAT_TX_ABRT_MASK					0x00000040
#define I2C_INTR_STAT_RD_REQ_MASK					0x00000020
#define I2C_INTR_STAT_TX_EMPTY_MASK					0x00000010
#define I2C_INTR_STAT_TX_OVER_MASK					0x00000008
#define I2C_INTR_STAT_RX_FULL_MASK					0x00000004
#define I2C_INTR_STAT_RX_OVER_MASK					0x00000002
#define I2C_INTR_STAT_RX_UNDER_MASK					0x00000001

/* Definition of bits in Interrupt Mask Register */
#define I2C_INTR_MASK_SCL_STUCK_AT_LOW_MASK			0x00004000
#define I2C_INTR_MASK_MASTER_ON_HOLD_MASK			0x00002000
#define I2C_INTR_MASK_RESTART_DET_MASK				0x00001000
#define I2C_INTR_MASK_GEN_CALL_MASK					0x00000800
#define I2C_INTR_MASK_START_DET_MASK				0x00000400
#define I2C_INTR_MASK_STOP_DET_MASK					0x00000200
#define I2C_INTR_MASK_ACTIVITY_MASK					0x00000100
#define I2C_INTR_MASK_RX_DONE_MASK					0x00000080
#define I2C_INTR_MASK_TX_ABRT_MASK					0x00000040
#define I2C_INTR_MASK_RD_REQ_MASK					0x00000020
#define I2C_INTR_MASK_TX_EMPTY_MASK					0x00000010
#define I2C_INTR_MASK_TX_OVER_MASK					0x00000008
#define I2C_INTR_MASK_RX_FULL_MASK					0x00000004
#define I2C_INTR_MASK_RX_OVER_MASK					0x00000002
#define I2C_INTR_MASK_RX_UNDER_MASK					0x00000001

/* Definition of bits in Status Register */
#define I2C_STATUS_MST_ACTIVE_MASK					0x00000020
#define I2C_STATUS_RFF_MASK							0x00000010
#define I2C_STATUS_RFNE_MASK						0x00000008
#define I2C_STATUS_TFE_MASK							0x00000004
#define I2C_STATUS_TFNF_MASK						0x00000002
#define I2C_STATUS_ACTIVITY_MASK					0x00000001

/* Definition of bits in Component Parameter Register 1 */
#define I2C_COMP_PARAM_TXBUF_DEPTH_MASK				0x00FF0000
#define I2C_COMP_PARAM_RXBUF_DEPTH_MASK				0x0000FF00

/*
 * Macro used for asserting channel ID
 */
#define IS_I2C_CHANNEL(CHANNEL)		((CHANNEL >= I2C_CH0) && \
									(CHANNEL < I2C_CH_MAX_IDX))

/*
 * Macro used for asserting operation mode
 */
#define IS_I2C_OPMODE(CHANNEL, OPMODE)	(((OPMODE == I2C_MASTER) && \
										(CHANNEL >= I2C_CH0) && \
										(CHANNEL <= I2C_CH18)) || \
										((OPMODE == I2C_SLAVE) && \
										(CHANNEL >= I2CS_CH0) && \
										(CHANNEL <= I2CS_CH1)))

/*
 * Macro used for asserting address mode
 */
#define IS_I2C_ADDRMODE(ADDRMODE)	((ADDRMODE == I2C_7BIT) || \
									(ADDRMODE == I2C_10BIT))

/*
 * Macro used for asserting speed mode
 */
#define IS_I2C_SPEEDMODE(SPEEDMODE)	((SPEEDMODE == I2C_SPEED_100KHz) || \
									(SPEEDMODE == I2C_SPEED_400KHz))

/*
 * Macro used for asserting address value
 */
#define IS_I2C_ADDRESS(ADDRMODE, ADDRVALUE)		\
			(((I2C_7BIT == ADDRMODE) && (128 > ADDRVALUE)) ||	\
			((I2C_10BIT == ADDRMODE) && (1024 > ADDRVALUE)))

/*
 * Macro used for initialization i2c object
 */
#define I2C_MASTER_INIT(n)	{\
								.i2c_base = (i2c_reg_t *)I2C##n##_BASE, \
								.irq_handler = NULL, \
								.init_status = I2C_INIT_UNINITIALIZED, \
								.state_machine = I2C_IDLE \
							}

#define I2C_SLAVE_INIT(n)	{\
								.i2c_base = (i2c_reg_t *)I2CS##n##_BASE, \
								.irq_handler = NULL, \
								.init_status = I2C_INIT_UNINITIALIZED, \
								.state_machine = I2C_IDLE \
							}

#define I2C_NUM_OF_SFIFO_BUFS			((I2C_SOFT_FIFO_BUFFER_SIZE / 32) + 2)


/* Private typedef -----------------------------------------------------------*/

/*
 * @brief i2c_reg_t
 *
 * I2C registers structure
 */
typedef struct i2c_reg
{
	__IO uint32_t	CON;				/* Control Register */
	__IO uint32_t	TAR;				/* Target Address Register */
	__IO uint32_t	SAR;				/* Slave Address Register */
	__IO uint32_t	HS_MADDR;			/* Master Mode Code Address Register */
	__IO uint32_t	DATA_CMD;			/* Rx/Tx Data Buffer/Command Register*/
	__IO uint32_t	SS_SCL_HCNT;		/* Standard Speed Clock High Register */
	__IO uint32_t	SS_SCL_LCNT;		/* Standard Speed Clock Low Register */
	__IO uint32_t	FS_SCL_HCNT;		/* Fast Speed Clock High Register */
	__IO uint32_t	FS_SCL_LCNT;		/* Fast Speed Clock Low Register */
	__IO uint32_t	HS_SCL_HCNT;		/* High Speed Clock High Register */
	__IO uint32_t	HS_SCL_LCNT;		/* High Speed Clock Low Register */
	__I  uint32_t	INTR_STAT;			/* Interrupt Status Register */
	__IO uint32_t	INTR_MASK;			/* Interrupt Mask Register */
	__I  uint32_t	RAW_INTR_STAT;		/* Raw Interrupt Status Register */
	__IO uint32_t	RX_TL;				/* Receive FIFO Threshold Register */
	__IO uint32_t	TX_TL;				/* Transmit FIFO Threshold Register */
	__I  uint32_t	CLR_INTR;			/* Clear Combined Interrupt Register */
	__I  uint32_t	CLR_RX_UNDER;		/* Clear RX_UNDER Interrupt Register */
	__I  uint32_t	CLR_RX_OVER;		/* Clear RX_OVER Interrupt Register */
	__I  uint32_t	CLR_TX_OVER;		/* Clear TX_OVER Interrupt Register */
	__I  uint32_t	CLR_RD_REQ;			/* Clear RX_REQ Interrupt Register */
	__I  uint32_t	CLR_TX_ABRT;		/* Clear TX_ABRT Interrupt Register */
	__I  uint32_t	CLR_RX_DONE;		/* Clear RX_DONE Interrupt Register */
	__I  uint32_t	CLR_ACTIVITY;		/* Clear ACTIVITY Interrupt Register */
	__I  uint32_t	CLR_STOP_DET;		/* Clear STOP_DET Interrupt Register */
	__I  uint32_t	CLR_START_DET;		/* Clear RX_OVER Interrupt Register */
	__I  uint32_t	CLR_GEN_CALL;		/* Clear GEN_CAL Interrupt Register */
	__IO uint32_t	EN;				/* Enable Register */
	__I  uint32_t	STATUS;				/* Status Register */
	__I  uint32_t	TXFLR;				/* Transmit FIFO Level Register */
	__I  uint32_t	RXFLR;				/* Receive FIFO Level Register */
	__IO uint32_t	SDA_HOLD;			/* SDA Hold Time Length Register */
	__I  uint32_t	TX_ABRT_SOURCE;		/* Transmit Abort Status Register */
	__IO uint32_t	SLV_DATA_NACK_ONLY;	/* Generate SLV_DATA_NACK Register */
	__IO uint32_t	DMA_CR;				/* DMA Control Register */
	__IO uint32_t	DMA_TDLR;			/* DMA Transmit Data Level Register */
	__IO uint32_t	DMA_RDLR;			/* DMA Receive Data Level Register */
	__IO uint32_t	SDA_SETUP;			/* SDA Setup Register */
	__IO uint32_t	ACK_GENERAL_CALL;	/* ACK General Call Register */
	__I  uint32_t	Resrvd[22];			/* Reserved */
	__IO uint32_t	COM_PARAM;			/* Component Parameter Register */
} i2c_reg_t;

/*
 * @brief hal_i2c_init_status_t
 *
 * I2C initialization status
 */
typedef enum
{
	I2C_INIT_UNINITIALIZED = 0,
	I2C_INIT_INITIALIZED
} hal_i2c_init_status_t;

/*
 * @brief hal_i2c_config_t
 *
 * I2C configuration structure
 */
typedef struct hal_i2c_config
{
	/* Used to store I2Cx base address */
	i2c_reg_t				*i2c_base;
	/* Used for slave address validation check */
	int							address_mode;
	/* Initialization status, used for initialization validation check */
	hal_i2c_init_status_t		init_status;
	/* State machine */
	hal_i2c_status_t			state_machine;

	/* Used for interrupt handling */

	/* Soft Tx FIFO buffers */
	hal_i2c_buffer_t			tx_soft_fifo[I2C_NUM_OF_SFIFO_BUFS];
	uint16_t					tx_soft_fifo_current_index;

	/* Soft Rx FIFO buffers */
	hal_i2c_buffer_t			rx_soft_fifo[I2C_NUM_OF_SFIFO_BUFS];
	uint16_t					rx_soft_fifo_current_index;

	hal_i2c_operation_mode_t	operation_mode;		/* operation mode */

	/* Interrupt handler pointing to upper layer function */
	void						(*irq_handler)(hal_i2c_status_t status);
} hal_i2c_config_t;

/* Private variables ---------------------------------------------------------*/

/* I2C configurations */
static hal_i2c_config_t i2c_configs[] =
{
	/* Init param for I2C Master channel 0 -> 18*/
	I2C_MASTER_INIT(0),
	I2C_MASTER_INIT(1),
	I2C_MASTER_INIT(2),
	I2C_MASTER_INIT(3),
	I2C_MASTER_INIT(4),
	I2C_MASTER_INIT(5),
	I2C_MASTER_INIT(6),
	I2C_MASTER_INIT(7),
	I2C_MASTER_INIT(8),
	I2C_MASTER_INIT(9),
	I2C_MASTER_INIT(10),
	I2C_MASTER_INIT(11),
	I2C_MASTER_INIT(12),
	I2C_MASTER_INIT(13),
	I2C_MASTER_INIT(14),
	I2C_MASTER_INIT(15),
	I2C_MASTER_INIT(16),
	I2C_MASTER_INIT(17),
	I2C_MASTER_INIT(18),

	/* Init param for I2C Slave channel 0 & 1 */
	I2C_SLAVE_INIT(0),
	I2C_SLAVE_INIT(1)
};

/* Private functions prototypes ----------------------------------------------*/

static void i2c_master_tx(int chid);
static void i2c_master_rx(int chid);
static void i2c_slave_tx(int chid);
static void i2c_set_next_tx_threshold(int chid);
static void i2c_push_to_sfifo(int chid, hal_i2c_buffer_t *sbuf,
														hal_i2c_buffer_t *dbuf);
static void i2c_isr_handler(int chid);
/* Interrupt handlers */
static void I2C0_IRQHandler(void);
static void I2C1_IRQHandler(void);
static void I2C2_IRQHandler(void);
static void I2C3_IRQHandler(void);
static void I2C4_IRQHandler(void);
static void I2C5_IRQHandler(void);
static void I2C6_IRQHandler(void);
static void I2C7_IRQHandler(void);
static void I2C8_IRQHandler(void);
static void I2C9_IRQHandler(void);
static void I2C10_IRQHandler(void);
static void I2C11_IRQHandler(void);
static void I2C12_IRQHandler(void);
static void I2C13_IRQHandler(void);
static void I2C14_IRQHandler(void);
static void I2C15_IRQHandler(void);
static void I2C16_IRQHandler(void);
static void I2C17_IRQHandler(void);
static void I2C18_IRQHandler(void);
static void I2CS0_IRQHandler(void);
static void I2CS1_IRQHandler(void);

/* Testing variable - --------------------------------------------------------*/
hal_i2c_t test_hal_i2c_init;
hal_i2c_t test_hal_i2c_enable_irq;
int hal_i2c_init_count;
int hal_i2c_enable_irq_count;
int hal_i2c_slave_writefifo_count;
int hal_i2c_slave_readfifo_count;
int hal_i2c_master_tx_count ;
int hal_i2c_master_tx_ret;
int hal_i2c_master_rx_count;
int hal_i2c_master_rx_ret;
hal_i2c_buffer_t hal_i2c_master_rx_buf;
int hal_i2c_slave_tx_count;
hal_i2c_buffer_t *buf_test;
int hal_i2c_master_tx_for_unit_read_eeprom;
int hal_i2c_master_rx_for_unit_read_eeprom;
int hal_i2c_master_rx_for_unit_drv_vcm_init;
/* ---------------------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief hal_i2c_init
 * Initializes I2C independent channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_init(hal_i2c_t *i2c)
{
	test_hal_i2c_init = *i2c;
	hal_i2c_init_count ++;
}

/*
 * @brief hal_i2c_init
 * De-initializes I2C independent channel
 * @param chid I2C channel ID
 * @return None
 */
void hal_i2c_deinit(hal_i2c_channel_t chid)
{
}

/*
 * @brief hal_i2c_master_tx
 * Transmits data to another I2C slave
 * @param chid I2C channel ID
 * @param slave_addr Slave address want to send data
 * @param buf Data buffer pointer
 * @return Transmits status
 */
int hal_i2c_master_tx(hal_i2c_channel_t chid,
							uint16_t slave_addr, hal_i2c_buffer_t *buf)
{
	hal_i2c_master_tx_count ++;
	buf_test = malloc(sizeof(buf));
	buf_test->bytes = malloc(sizeof(buf->bytes));
	buf_test->length = buf->length;
	for (int i =0;i<buf_test->length;i++)
	{
		*(buf_test->bytes +i) = *(buf->bytes+i);
	}
	if (hal_i2c_master_tx_for_unit_read_eeprom == 1)
	{
		if (hal_i2c_master_tx_count ==2)
			hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	}
	return hal_i2c_master_tx_ret;
}

/*
 * @brief hal_i2c_master_rx
 * Receives data from another I2C slave
 * @param chid I2C channel ID
 * @param slave_addr Slave address want to receive from
 * @param buf Data buffer pointer
 * @return Receive status
 */
int hal_i2c_master_rx(hal_i2c_channel_t chid,
							uint16_t slave_addr, hal_i2c_buffer_t *buf)
{
	hal_i2c_master_rx_count ++;
	for (int i =0; i < buf->length; i++)
	{
		*(buf->bytes +i) = *(hal_i2c_master_rx_buf.bytes +i);
	}
	if (hal_i2c_master_rx_for_unit_read_eeprom == 1)
	{
		if (hal_i2c_master_rx_count ==2)
			hal_i2c_master_rx_ret = I2C_TIMED_OUT;
	}
	if ((slave_addr== 0x72)&&(\
			(hal_i2c_master_rx_for_unit_drv_vcm_init&0x01) == 1))
	{
		*buf->bytes = 0x42;
	}

	if ((slave_addr== 0x72)&&((hal_i2c_master_rx_for_unit_drv_vcm_init\
			&0x02) == 2)&&(hal_i2c_master_rx_count == 6))
	{
		*buf->bytes = 0;
	}
	return hal_i2c_master_rx_ret;
}

/*
 * @brief hal_i2c_slave_tx
 * Transmits data (responds) to I2C master
 * @param chid I2C channel ID
 * @param buf Data buffer pointer
 * @return Transmit status
 */
int hal_i2c_slave_tx(hal_i2c_channel_t chid, hal_i2c_buffer_t *buf)
{
	hal_i2c_slave_tx_count++;
}

/*
 * @brief hal_i2c_slave_rx
 * Receives data from I2C master
 * @param chid I2C channel ID
 * @param buf Data buffer pointer
 * @return Receive status
 */
int hal_i2c_slave_rx(hal_i2c_channel_t chid, hal_i2c_buffer_t *buf)
{
}

/*
 * @brief hal_i2c_enable_irq
 * Enables interrupt for I2C channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_enable_irq(hal_i2c_t *i2c)
{
	hal_i2c_enable_irq_count ++;
	test_hal_i2c_enable_irq = *i2c;
}

/*
 * @brief hal_i2c_disable_irq
 * Disables interrupt for I2C channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_disable_irq(hal_i2c_channel_t chid)
{
}

/*
 * @brief hal_i2c_get_status
 * @param chid I2C channel ID
 * @return I2C current status
 */
hal_i2c_status_t hal_i2c_get_status(hal_i2c_channel_t chid)
{
}

/*
 * @brief hal_i2c_slave_writefifo
 * The function sends one byte to fifo hardware
 * @param chid I2C channel ID
 * @param byte Databyte want to send out to FIFO
 * @return I2C current status
 */
int hal_i2c_slave_writefifo(int chid, uint8_t byte)
{
	hal_i2c_slave_writefifo_count++;
}

/*
 * @brief hal_i2c_slave_readfifo
 * The function reads one byte from hardware fifo
 * @param chid I2C channel ID
 * @param byte Databyte pointer want to read in from FIFO
 * @return I2C current status
 */
int hal_i2c_slave_readfifo(int chid, uint8_t *byte)
{
	hal_i2c_slave_readfifo_count ++;
}

/* Static functions ----------------------------------------------------------*/

/*
 * i2c_master_tx
 * Internal function, used to transmits the current data
 * stored in soft Tx FIFO buffers to slave
 */
static void i2c_master_tx(int chid)
{
}

/*
 * i2c_master_rx
 * Internal function, used to transmits the current read requests
 * stored in soft Tx FIFO buffers to slave
 */
static void i2c_master_rx(int chid)
{
}

/*
 * i2c_slave_tx
 * Internal function, used to transmits the current data
 * stored in soft Tx FIFO buffers to master
 */
static void i2c_slave_tx(int chid)
{
}

/*
 * @brief i2c_isr_handler
 * Internal interrupt service routine
 */
static void i2c_isr_handler(int chid)
{
}

/*
 * i2c_push_to_sfifo
 * Sets Tx FIFO threshold for triggering next TX_EMPTY interrupt
 */
static void i2c_set_next_tx_threshold(int chid)
{
}

/*
 * i2c_push_to_sfifo
 * Pushes to source buffer into the destination buffer
 */
static void i2c_push_to_sfifo(int chid, hal_i2c_buffer_t *sbuf,
														hal_i2c_buffer_t *dbuf)
{
}

/*
 * I2C0_IRQHandler
 * Internal I2C channel 0 interrupt handler
 */
static void I2C0_IRQHandler(void)
{
}

/*
 * I2C1_IRQHandler
 * Internal I2C channel 1 interrupt handler
 */
static void I2C1_IRQHandler(void)
{
}

/*
 * I2C2_IRQHandler
 * Internal I2C channel 2 interrupt handler
 */
static void I2C2_IRQHandler(void)
{
}

/*
 * I2C3_IRQHandler
 * Internal I2C channel 3 interrupt handler
 */
static void I2C3_IRQHandler(void)
{
}

/*
 * I2C4_IRQHandler
 * Internal I2C channel 4 interrupt handler
 */
static void I2C4_IRQHandler(void)
{
}

/*
 * I2C5_IRQHandler
 * Internal I2C channel 5 interrupt handler
 */
static void I2C5_IRQHandler(void)
{
}

/*
 * I2C6_IRQHandler
 * Internal I2C channel 6 interrupt handler
 */
static void I2C6_IRQHandler(void)
{
}

/*
 * I2C7_IRQHandler
 * Internal I2C channel 7 interrupt handler
 */
static void I2C7_IRQHandler(void)
{
}

/*
 * I2C8_IRQHandler
 * Internal I2C channel 8 interrupt handler
 */
static void I2C8_IRQHandler(void)
{
}

/*
 * I2C9_IRQHandler
 * Internal I2C channel 9 interrupt handler
 */
static void I2C9_IRQHandler(void)
{
}

/*
 * I2C10_IRQHandler
 * Internal I2C channel 10 interrupt handler
 */
static void I2C10_IRQHandler(void)
{
}

/*
 * I2C11_IRQHandler
 * Internal I2C channel 11 interrupt handler
 */
static void I2C11_IRQHandler(void)
{
}

/*
 * I2C12_IRQHandler
 * Internal I2C channel 12 interrupt handler
 */
static void I2C12_IRQHandler(void)
{
}

/*
 * I2C13_IRQHandler
 * Internal I2C channel 13 interrupt handler
 */
static void I2C13_IRQHandler(void)
{
}

/*
 * I2C14_IRQHandler
 * Internal I2C channel 14 interrupt handler
 */
static void I2C14_IRQHandler(void)
{
}

/*
 * I2C15_IRQHandler
 * Internal I2C channel 15 interrupt handler
 */
static void I2C15_IRQHandler(void)
{
}

/*
 * I2C16_IRQHandler
 * Internal I2C channel 16 interrupt handler
 */
static void I2C16_IRQHandler(void)
{
}

/*
 * I2C17_IRQHandler
 * Internal I2C channel 17 interrupt handler
 */
static void I2C17_IRQHandler(void)
{
}

/*
 * I2C18_IRQHandler
 * Internal I2C channel 18 interrupt handler
 */
static void I2C18_IRQHandler(void)
{
}

/*
 * I2CS0_IRQHandler
 * Internal I2C slave channel 0 interrupt handler
 */
static void I2CS0_IRQHandler(void)
{
}

/*
 * I2CS1_IRQHandler
 * Internal I2C slave channel 1 interrupt handler
 */
static void I2CS1_IRQHandler(void)
{
}

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE *******/
