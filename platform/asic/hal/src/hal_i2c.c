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
#include "std_type.h"
#include "cortex_r4.h"
#include "hal_i2c.h"
#include "hal_vic.h"
#include "assert.h"
#include "board_config.h"

/* Private define ------------------------------------------------------------*/
/* I2C clock speed definition */
#define I2C_CLK										CLOCK_133MHZ
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
#define I2C_INTR_STAT_RESTART_DET_MASK				0x00001000
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
 * Macro used for asserting operation mode
 */
#define IS_I2C_OPMODE(CHANNEL, OPMODE)	((CHANNEL >= I2C_CH0) && \
										(CHANNEL < I2C_CH_MAX_IDX))

/*
 * Macro used for asserting address mode
 */
#define IS_I2C_ADDRMODE(ADDRMODE)	((ADDRMODE == I2C_7BIT) || \
									(ADDRMODE == I2C_10BIT))

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
								.address_mode = I2C_7BIT, \
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
	__IO uint32_t	EN;					/* Enable Register */
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
	__I  uint32_t	Resrvd1[3];			/* Reserved */
	__I  uint32_t	CLR_RESTART_DET;	/* Clear RESTART Interrupt Register */
	__I  uint32_t	Resrvd2[18];		/* Reserved */
	__IO uint32_t	COM_PARAM;			/* Component Parameter Register */
} i2c_reg_t;

/*
 * @brief i2c_init_status_t
 *
 * I2C initialization status
 */
typedef enum
{
	I2C_INIT_UNINITIALIZED = 0,
	I2C_INIT_INITIALIZED
} i2c_init_status_t;

/*
 * @brief i2c_config_t
 *
 * I2C configuration structure
 */
typedef struct i2c_config
{
	/* Used to store I2Cx base address */
	i2c_reg_t					*i2c_base;
	/* Used for slave address validation check */
	int							address_mode;
	/* Initialization status, used for initialization validation check */
	i2c_init_status_t			init_status;
	/* State machine */
	volatile hal_i2c_status_t	state_machine;

	/* Used for interrupt handling */

	/* Soft Tx FIFO buffers */
	hal_i2c_buffer_t			tx_soft_fifo[I2C_NUM_OF_SFIFO_BUFS];
	uint16_t					tx_soft_fifo_current_index;

	/* Soft Rx FIFO buffers */
	hal_i2c_buffer_t			rx_soft_fifo[I2C_NUM_OF_SFIFO_BUFS];
	uint16_t					rx_soft_fifo_current_index;

	hal_i2c_operation_mode_t	operation_mode;		/* operation mode */
	hal_i2c_clockspeed_t		speed;				/* I2C speed */

	/* Interrupt handler pointing to upper layer function */
	void		(*irq_handler)(hal_i2c_channel_t chid, hal_i2c_status_t status);
} i2c_config_t;

/* Private variables ---------------------------------------------------------*/

/* I2C configurations */
static i2c_config_t i2c_configs[] =
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

static void i2c_master_tx(hal_i2c_channel_t chid);
static void i2c_master_rx(hal_i2c_channel_t chid);
static void i2c_slave_tx(hal_i2c_channel_t chid);
static void i2c_set_next_tx_threshold(hal_i2c_channel_t chid);
static void i2c_push_to_sfifo(hal_i2c_channel_t chid, hal_i2c_buffer_t *sbuf,
														hal_i2c_buffer_t *dbuf);
static void i2c_isr_handler(hal_i2c_channel_t chid);
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

/* Exported functions --------------------------------------------------------*/

/*
 * @brief hal_i2c_init
 * Initializes I2C independent channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_init(hal_i2c_t *i2c)
{
	i2c_config_t	*i2c_config;
	i2c_reg_t		*i2c_reg;

	/* Assert input parameters */
	assert_param(NULL != i2c);
	assert_param(IS_I2C_CHANNEL(i2c->chid));
	assert_param(IS_I2C_OPMODE(i2c->chid, i2c->operation_mode));
	assert_param(IS_I2C_ADDRMODE(i2c->address_mode));
	assert_param(IS_I2C_SPEEDMODE(i2c->clock_speed));
	assert_param(IS_I2C_ADDRESS(i2c->address_mode, i2c->owner_addr));

	i2c_config = &(i2c_configs[i2c->chid]);

	/* Do nothing if the I2C channel has already been initialized */
	if (i2c_config->init_status == I2C_INIT_INITIALIZED)
		return;

	/* Set clock speed mode */
	hal_i2c_set_speed(i2c->chid, i2c->clock_speed);

	i2c_reg = i2c_config->i2c_base;

	/* Disable the selected i2c channel */
	i2c_reg->EN = 0;

	/* Set operation mode */
	if (I2C_MASTER == i2c->operation_mode)
	{
		/* Enable master mode */
		i2c_reg->CON |= I2C_CON_MASTER_MODE_MASK;
		/* Disable slave mode */
		i2c_reg->CON |= I2C_CON_SLAVE_DISABLE_MASK;
	}
	else
	{
		/* Enable slave mode */
		i2c_reg->CON &= (~I2C_CON_SLAVE_DISABLE_MASK);
		/* Disable master mode */
		i2c_reg->CON &= (~I2C_CON_MASTER_MODE_MASK);
		/* Set slave address */
		i2c_reg->SAR = i2c->owner_addr;
	}
	i2c_config->operation_mode = i2c->operation_mode;

	/* Set address mode */
	if (I2C_7BIT == i2c->address_mode)
	{
		i2c_reg->CON &= (~I2C_CON_10BITADDR_MASTER_MASK);
		i2c_reg->CON &= (~I2C_CON_10BITADDR_SLAVE_MASK);
		i2c_reg->TAR &= (~I2C_TAR_10BITADDR_MASTER_MASK);
	}
	else
	{
		i2c_reg->CON |= I2C_CON_10BITADDR_MASTER_MASK;
		i2c_reg->CON |= I2C_CON_10BITADDR_SLAVE_MASK;
		i2c_reg->TAR |= I2C_TAR_10BITADDR_MASTER_MASK;
	}
	i2c_config->address_mode = i2c->address_mode;

	/* Disable start byte */
	i2c_reg->TAR &= (~I2C_TAR_GC_OR_START_MASK);
	/* Enable RESTART condition */
	i2c_reg->CON |= I2C_CON_RESTART_EN_MASK;
	/* Disable all interrupts */
	i2c_reg->INTR_MASK = 0;

	/* Set initialization status */
	i2c_config->init_status = I2C_INIT_INITIALIZED;
	/* Set state machine to idle */
	i2c_config->state_machine = I2C_IDLE;

	/* Set Tx/Rx FIFO threshold levels to 1 entry */
	i2c_reg->TX_TL = 0;
	i2c_reg->RX_TL = 0;
	/* Clear soft FIFO buffers */
	i2c_config->tx_soft_fifo[0].length = 0;
	i2c_config->rx_soft_fifo[0].length = 0;
	i2c_config->tx_soft_fifo_current_index = 0;
	i2c_config->rx_soft_fifo_current_index = 0;

	/* Enable the selected i2c channel */
	i2c_reg->EN = 1;
}

/*
 * @brief hal_i2c_set_speed
 * Update I2C master speed update
 * @param chid I2C channel ID
 * @param speed I2C master speed
 * @return None
 */
void hal_i2c_set_speed(hal_i2c_channel_t chid, hal_i2c_clockspeed_t speed)
{
	i2c_config_t	*i2c_config;
	i2c_reg_t		*i2c_reg;
	uint32_t		input_clock_mhz;

	/* Assert input parameters */
	assert_param(IS_I2C_CHANNEL(chid));
	assert_param(IS_I2C_SPEEDMODE(speed));

	i2c_config = &(i2c_configs[chid]);
	i2c_reg = i2c_config->i2c_base;
	input_clock_mhz = I2C_CLK / 1000;

	/* Disable the selected i2c channel */
	i2c_reg->EN = 0;

	/*
	 * tHIGH = (HCNT + IC_*_SPKLEN + 3) * (1 / ic_freq) + tFALL
	 * tLOW = LCNT * (1 / ic_freq) - tFALL + tRAISE
	 */
	if (I2C_SPEED_100KHz == speed)
	{
		i2c_reg->CON = (i2c_reg->CON & (~I2C_CON_SPEED_MASK)) |
						I2C_CON_SPEED_STANDARD;
		/*
		 * tHIGHmin = 4us, tLOWmin = 4.7us.
		 * tHIGH = 4.8us, tLOW = 5.2us, tFALL = 0.002us, tRAISE = 0.18us.
		 * tHIGH + tLOW = 4.8us + 5.2us = 10us => output frequency = 100kHz.
		 */
//		i2c_reg->SS_SCL_HCNT =
//						(input_clock_mhz * (4800 - 2) + 500) / 1000 - 4;
//		i2c_reg->SS_SCL_LCNT =
//						(input_clock_mhz * (5200 + 2 - 180) + 500) / 1000;
		i2c_reg->SS_SCL_HCNT = (input_clock_mhz * 43 + 5000) / 10000 - 8;//(input_clock_mhz * (6 + 3) + 5000) / 10000 - 3 ;

		i2c_reg->SS_SCL_LCNT = ((input_clock_mhz * (52 + 1) + 5000) / 10000) - 1 ;
	}
	else if (I2C_SPEED_400KHz == speed)
	{
		i2c_reg->CON = (i2c_reg->CON & (~I2C_CON_SPEED_MASK)) |
						I2C_CON_SPEED_FAST;
		/*
		 * tHIGHmin = 0.6us, tLOWmin = 1.3us.
		 * tHIGH = 1.1us, tLOW = 1.4us, tFALL = 0.002us, tRAISE = 0.18us.
		 * tHIGH + tLOW = 1.1us + 1.4us = 2.5us => output frequency = 400kHz.
		 */
//		i2c_reg->FS_SCL_HCNT =
//						(input_clock_mhz * (500 - 2) + 500) / 1000;
//		i2c_reg->FS_SCL_LCNT =
//						(input_clock_mhz * (1700 + 2 - 180) + 500) / 1000;
#ifdef EVT3_REWORK
		i2c_reg->FS_SCL_HCNT = (input_clock_mhz * (6 + 2) + 5000) / 10000 - 2;//(input_clock_mhz * (6 + 3) + 5000) / 10000 - 3 ;

		i2c_reg->FS_SCL_LCNT = ((input_clock_mhz * (13 + 3) + 5000) / 10000) - 7;
#else
		i2c_reg->FS_SCL_HCNT = (input_clock_mhz * 6 + 5000) / 10000 - 8;//(input_clock_mhz * (6 + 3) + 5000) / 10000 - 3 ;
		i2c_reg->FS_SCL_LCNT = ((input_clock_mhz * (13 + 2) + 5000) / 10000) - 1 ;
#endif

	}
	else /* if (I2C_SPEED_1MHz == speed) */
	{
		i2c_reg->CON = (i2c_reg->CON & (~I2C_CON_SPEED_MASK)) |
						I2C_CON_SPEED_FAST;
		/*
		 * tHIGHmin = 0.26us, tLOWmin = 0.5us.
		 * tHIGH = 0.4us, tLOW = 0.6us, tFALL = 0.002us, tRAISE = 0.13us
		 * tHIGH + tLOW = 0.4us + 0.6us = 1us => output frequency = 1MHz.
		 */
//		i2c_reg->FS_SCL_HCNT =
//						(input_clock_mhz * (400 - 2) + 500) / 1000 - 4;
//		i2c_reg->FS_SCL_LCNT =
//						(input_clock_mhz * (600 + 2 - 130) + 500) / 1000;

		/* FIXME: This generates clock speed of 781 KHz  instead of 1MHz*/
#ifdef EVT3_REWORK
		i2c_reg->FS_SCL_HCNT = (((input_clock_mhz * 4) ) + 5000) / 10000 - 10;//(input_clock_mhz * (6 + 3) + 5000) / 10000 - 3 ;
		i2c_reg->FS_SCL_LCNT = ((input_clock_mhz * (5) + 5000) / 10000) - 0 ;
#else
		i2c_reg->FS_SCL_HCNT = (((input_clock_mhz * 3) ) + 5000) / 10000 - 8;//(input_clock_mhz * (6 + 3) + 5000) / 10000 - 3 ;
		i2c_reg->FS_SCL_LCNT = ((input_clock_mhz * (5 ) + 5000) / 10000) - 1 ;
#endif
	}
	i2c_config->speed = speed;

	/* Enable the selected i2c channel */
	i2c_reg->EN = 1;
}

/*
 * @brief hal_i2c_update_speed
 * Update I2C master speed update
 * @param chid I2C channel ID
 * @param speed
 * @return Current I2C master speed
 */
hal_i2c_clockspeed_t hal_i2c_get_speed(hal_i2c_channel_t chid)
{
	i2c_config_t *i2c_config = &(i2c_configs[chid]);
	return i2c_config->speed;
}

/*
 * @brief hal_i2c_init
 * De-initializes I2C independent channel
 * @param chid I2C channel ID
 * @return None
 */
void hal_i2c_deinit(hal_i2c_channel_t chid)
{
	/* Assert input parameters */
	assert_param(IS_I2C_CHANNEL(chid));

	/* Disable i2c channel */
	i2c_configs[chid].i2c_base->EN = 0;
	/* Disable interrupt */
	hal_i2c_disable_irq(chid);
	/* Update initialization status */
	i2c_configs[chid].init_status = I2C_INIT_UNINITIALIZED;
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
	i2c_config_t	*i2c_config;
	int					timeout;
	int					ret = I2C_OK;

	/* Assert input parameters */
	assert_param(NULL != buf);
	assert_param(IS_I2C_CHANNEL(chid));

	i2c_config = &(i2c_configs[chid]);

	assert_param(IS_I2C_ADDRESS(i2c_config->address_mode, slave_addr));
	assert_param(I2C_INIT_INITIALIZED == i2c_config->init_status);

	/* Wait for the completion of the previous transmission */
	timeout = I2C_TIMEOUT;
	while (((I2C_TX_TRASMITTING == i2c_config->state_machine) ||
		(I2C_RX_RECEIVING == i2c_config->state_machine)) && timeout--);
	if (0 > timeout)
		ret = I2C_TIMED_OUT;
	else
	{
		i2c_reg_t	*i2c_reg = i2c_config->i2c_base;

		/* Push all data into the soft Tx FIFO */
		i2c_push_to_sfifo(chid, buf, i2c_config->tx_soft_fifo);

		/* Reset soft Tx FIFO current index before transmission */
		i2c_config->tx_soft_fifo_current_index = 0;
		/* Set state machine */
		i2c_config->state_machine = I2C_TX_TRASMITTING;
		/* Set Tx FIFO threshold for triggering next TX_EMPTY interrupt */
		i2c_set_next_tx_threshold(chid);

		/* Disable the selected i2c channel for writing target address */
		i2c_reg->EN = 0;
		/* Set slave address */
		i2c_reg->TAR &= (~I2C_TAR_TAR_MASK);
		i2c_reg->TAR |= slave_addr;
		/* Enable the selected i2c channel */
		i2c_reg->EN = 1;
		/*
		 * Push the first soft Tx FIFO buffer into hard Tx FIFO
		 * to transmit them and wait to get Tx completion interrupt
		 */
		i2c_master_tx(chid);
	}

	return ret;
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
	i2c_config_t	*i2c_config;
	int					timeout;
	int					ret = I2C_OK;

	/* Assert input parameters */
	assert_param(NULL != buf);
	assert_param(IS_I2C_CHANNEL(chid));

	i2c_config = &(i2c_configs[chid]);

	assert_param(IS_I2C_ADDRESS(i2c_config->address_mode, slave_addr));
	assert_param(I2C_INIT_INITIALIZED == i2c_config->init_status);

	/* Wait for the completion of the previous transmission */
	timeout = I2C_TIMEOUT;
	while (((I2C_TX_TRASMITTING == i2c_config->state_machine) ||
		(I2C_RX_RECEIVING == i2c_config->state_machine)) && timeout--);
	if (0 > timeout)
		ret = I2C_TIMED_OUT;
	else
	{
		i2c_reg_t	*i2c_reg = i2c_config->i2c_base;

		/* Push all read requests into the soft Tx FIFO */
		i2c_push_to_sfifo(chid, buf, i2c_config->tx_soft_fifo);
		/* Reset soft Tx FIFO current index before transmission */
		i2c_config->tx_soft_fifo_current_index = 0;
		/* Push buffer into the soft Rx FIFO */
		i2c_push_to_sfifo(chid, buf, i2c_config->rx_soft_fifo);
		/* Reset soft Tx FIFO current index before transmission */
		i2c_config->rx_soft_fifo_current_index = 0;

		/* Set state machine */
		i2c_config->state_machine = I2C_RX_RECEIVING;
		/* Set Tx FIFO threshold for triggering next TX_EMPTY interrupt */
		i2c_set_next_tx_threshold(chid);
		/* Set Rx FIFO threshold for triggering RX_FULL interrupt */
		i2c_reg->RX_TL = i2c_config->rx_soft_fifo[0].length - 1;

		/* Start to request data */

		/* Disable the selected i2c channel for writing target address */
		i2c_reg->EN = 0;
		/* Set slave address */
		i2c_reg->TAR &= (~I2C_TAR_TAR_MASK);
		i2c_reg->TAR |= slave_addr;
		/* Enable the selected i2c channel */
		i2c_reg->EN = 1;

		i2c_master_rx(chid);
	}

	return ret;
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
	i2c_config_t	*i2c_config;
	int					timeout;
	int					ret = I2C_OK;

	/* Assert input parameters */
	assert_param(NULL != buf);
	assert_param(IS_I2C_CHANNEL(chid));

	i2c_config = &(i2c_configs[chid]);

	assert_param(I2C_INIT_INITIALIZED == i2c_config->init_status);

	/* Wait for the completion of the previous transmission */
	timeout = I2C_TIMEOUT;
	while ((I2C_TX_TRASMITTING == i2c_config->state_machine)
			&& timeout--);
	if (0 > timeout)
		ret = I2C_TIMED_OUT;
	else
	{
		i2c_reg_t	*i2c_reg = i2c_config->i2c_base;

		/* Push all data into the soft Tx FIFO */
		i2c_push_to_sfifo(chid, buf, i2c_config->tx_soft_fifo);

		/* Reset soft Tx FIFO current index before transmission */
		i2c_config->tx_soft_fifo_current_index = 0;
		/* Set state machine */
		i2c_config->state_machine = I2C_TX_TRASMITTING;
		/* Set Tx FIFO threshold for triggering next TX_EMPTY interrupt */
		if (0 < i2c_config->tx_soft_fifo[1].length)
			i2c_reg->TX_TL = i2c_config->tx_soft_fifo[0].length - 1;
		else
			i2c_reg->TX_TL = 0;

		/*
		 * Push the first soft Tx FIFO buffer into hard Tx FIFO
		 * to transmit them and wait to get Tx completion interrupt
		 */
		i2c_slave_tx(chid);
	}

	return ret;
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
	i2c_reg_t		*i2c_reg;
	int					timeout;
	uint16_t			i;
	int					ret = I2C_OK;

	/* Assert input parameters */
	assert_param(NULL != buf);
	assert_param(IS_I2C_CHANNEL(chid));
	assert_param(I2C_INIT_INITIALIZED == i2c_configs[chid].init_status);

	i2c_reg = i2c_configs[chid].i2c_base;

	for (i = 0; i < buf->length; i++)
	{
		/* Wait for the completion of the previous reception */
		timeout = I2C_TIMEOUT;
		while ((I2C_STATUS_RFNE_MASK !=
				(i2c_reg->STATUS & I2C_STATUS_RFNE_MASK)) && timeout--);
		if (0 > timeout)
		{
			ret = I2C_TIMED_OUT;
			break;
		}
		else
		{
			/* Read data from hardware Rx FIFO */
			buf->bytes[i] = i2c_reg->DATA_CMD & 0xFF;
		}
	}

	return ret;
}

/*
 * @brief hal_i2c_enable_irq
 * Enables interrupt for I2C channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_enable_irq(hal_i2c_t *i2c)
{
	i2c_reg_t		*i2c_reg;

	/* Assert input parameters */
	assert_param(NULL != i2c);
	assert_param(IS_I2C_CHANNEL(i2c->chid));
	assert_param(I2C_INIT_INITIALIZED == i2c_configs[i2c->chid].init_status);

	i2c_reg = i2c_configs[i2c->chid].i2c_base;

	/* Enable RX_DONE interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_RX_DONE_MASK;
	/* Enable TX_ABRT interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_TX_ABRT_MASK;
	/* Enable RD_REQ interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_RD_REQ_MASK;
	/* Enable RX_FULL interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_RX_FULL_MASK;
	/* Enable STOP Detection interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_STOP_DET_MASK;
	/* Enable RESTART Detection interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_RESTART_DET_MASK;
	/*
	 * Generate TX_EMPTY interrupt when Tx shift register is not empty for
	 * faster transmission
	 */
	i2c_reg->CON |= I2C_CON_TX_EMPTY_CONTROL_MASK;

	/* Set interrupt handler */
	i2c_configs[i2c->chid].irq_handler = i2c->irq_handler;

	/* Register interrupt handler */
	switch (i2c->chid)
	{
		case I2C_CH0:
			vic_register_irq(I2C0_IRQn, I2C0_IRQHandler);
			vic_set_priority_irq(I2C0_IRQn, 10);
			break;
		case I2C_CH1:
			vic_register_irq(I2C1_IRQn, I2C1_IRQHandler);
			vic_set_priority_irq(I2C1_IRQn, 10);
			break;
		case I2C_CH2:
			vic_register_irq(I2C2_IRQn, I2C2_IRQHandler);
			vic_set_priority_irq(I2C2_IRQn, 10);
			break;
		case I2C_CH3:
			vic_register_irq(I2C3_IRQn, I2C3_IRQHandler);
			vic_set_priority_irq(I2C3_IRQn, 10);
			break;
		case I2C_CH4:
			vic_register_irq(I2C4_IRQn, I2C4_IRQHandler);
			vic_set_priority_irq(I2C4_IRQn, 10);
			break;
		case I2C_CH5:
			vic_register_irq(I2C5_IRQn, I2C5_IRQHandler);
			vic_set_priority_irq(I2C5_IRQn, 10);
			break;
		case I2C_CH6:
			vic_register_irq(I2C6_IRQn, I2C6_IRQHandler);
			vic_set_priority_irq(I2C6_IRQn, 10);
			break;
		case I2C_CH7:
			vic_register_irq(I2C7_IRQn, I2C7_IRQHandler);
			vic_set_priority_irq(I2C7_IRQn, 10);
			break;
		case I2C_CH8:
			vic_register_irq(I2C8_IRQn, I2C8_IRQHandler);
			vic_set_priority_irq(I2C8_IRQn, 10);
			break;
		case I2C_CH9:
			vic_register_irq(I2C9_IRQn, I2C9_IRQHandler);
			vic_set_priority_irq(I2C9_IRQn, 10);
			break;
		case I2C_CH10:
			vic_register_irq(I2C10_IRQn, I2C10_IRQHandler);
			vic_set_priority_irq(I2C10_IRQn, 10);
			break;
		case I2C_CH11:
			vic_register_irq(I2C11_IRQn, I2C11_IRQHandler);
			vic_set_priority_irq(I2C11_IRQn, 10);
			break;
		case I2C_CH12:
			vic_register_irq(I2C12_IRQn, I2C12_IRQHandler);
			vic_set_priority_irq(I2C12_IRQn, 10);
			break;
		case I2C_CH13:
			vic_register_irq(I2C13_IRQn, I2C13_IRQHandler);
			vic_set_priority_irq(I2C13_IRQn, 10);
			break;
		case I2C_CH14:
			vic_register_irq(I2C14_IRQn, I2C14_IRQHandler);
			vic_set_priority_irq(I2C14_IRQn, 10);
			break;
		case I2C_CH15:
			vic_register_irq(I2C15_IRQn, I2C15_IRQHandler);
			vic_set_priority_irq(I2C15_IRQn, 10);
			break;
		case I2C_CH16:
			vic_register_irq(I2C16_IRQn, I2C16_IRQHandler);
			vic_set_priority_irq(I2C16_IRQn, 10);
			break;
		case I2C_CH17:
			vic_register_irq(I2C17_IRQn, I2C17_IRQHandler);
			vic_set_priority_irq(I2C17_IRQn, 10);
			break;
		case I2C_CH18:
			vic_register_irq(I2C18_IRQn, I2C18_IRQHandler);
			vic_set_priority_irq(I2C18_IRQn, 10);
			break;
		case I2CS_CH0:
			vic_register_irq(I2CS0_IRQn, I2CS0_IRQHandler);
			vic_set_priority_irq(I2CS0_IRQn, 10);
			break;
		case I2CS_CH1:
			vic_register_irq(I2CS1_IRQn, I2CS1_IRQHandler);
			vic_set_priority_irq(I2CS1_IRQn, 10);
			break;
		default:
			break;
	}
}

/*
 * @brief hal_i2c_disable_irq
 * Disables interrupt for I2C channel
 * @param i2c I2C object pointer
 * @return None
 */
void hal_i2c_disable_irq(hal_i2c_channel_t chid)
{
	i2c_reg_t		*i2c_reg;

	/* Assert input parameters */
	assert_param(IS_I2C_CHANNEL(chid));
	assert_param(I2C_INIT_INITIALIZED == i2c_configs[chid].init_status);

	i2c_reg = i2c_configs[chid].i2c_base;

	/* Disable all interrupts */
	i2c_reg->INTR_MASK = 0;
	/* Clear all pending interrupts */
	i2c_reg->CLR_INTR;
	/* Clear interrupt handler */
	i2c_configs[chid].irq_handler = NULL;
	/* Unregister interrupt handler */
	vic_unregister_irq(I2CS1_IRQn + (I2CS_CH1 - chid));
}

/*
 * @brief hal_i2c_get_status
 * @param chid I2C channel ID
 * @return I2C current status
 */
hal_i2c_status_t hal_i2c_get_status(hal_i2c_channel_t chid)
{
	assert_param(IS_I2C_CHANNEL(chid));

	return i2c_configs[chid].state_machine;
}

/* Static functions ----------------------------------------------------------*/

/*
 * i2c_master_tx
 * Internal function, used to transmits the current data
 * stored in soft Tx FIFO buffers to slave
 */
static void i2c_master_tx(hal_i2c_channel_t chid)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;
	hal_i2c_buffer_t	*stxbuf = i2c_config->tx_soft_fifo;
	uint16_t			tx_current_index;
	uint16_t			i;

	tx_current_index = i2c_config->tx_soft_fifo_current_index;

	for (i = 0; i < stxbuf[tx_current_index].length; i++)
	{
		/* Check if this is the last byte for transmission */
		if (((stxbuf[tx_current_index].length - 1) == i) &&
			(0 == stxbuf[tx_current_index + 1].length))
		{
			/* Write data to FIFO and generate STOP condition */
			i2c_reg->DATA_CMD = stxbuf[tx_current_index].bytes[i] |
								I2C_DATA_CMD_STOP_MASK;
		}
		else
		{
			/* Write data to FIFO */
			i2c_reg->DATA_CMD = stxbuf[tx_current_index].bytes[i];
		}
	}

	/* Enable TX_EMPTY interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_TX_EMPTY_MASK;
}

/*
 * i2c_master_rx
 * Internal function, used to transmits the current read requests
 * stored in soft Tx FIFO buffers to slave
 */
static void i2c_master_rx(hal_i2c_channel_t chid)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;
	hal_i2c_buffer_t	*stxbuf = i2c_config->tx_soft_fifo;
	uint16_t			tx_current_index;
	uint16_t			i;

	tx_current_index = i2c_config->tx_soft_fifo_current_index;

	for (i = 0; i < stxbuf[tx_current_index].length; i++)
	{
		/* Check if this is the last read request */
		if (((stxbuf[tx_current_index].length - 1) == i) &&
			(0 == stxbuf[tx_current_index + 1].length))
		{
			/* Read data from slave and generate STOP condition */
			i2c_reg->DATA_CMD = I2C_DATA_CMD_CMD_MASK | I2C_DATA_CMD_STOP_MASK;
		}
		else
		{
			/* Request data from slave */
			i2c_reg->DATA_CMD = I2C_DATA_CMD_CMD_MASK;
		}
	}

	/* Enable TX_EMPTY interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_TX_EMPTY_MASK;
}

/*
 * i2c_slave_tx
 * Internal function, used to transmits the current data
 * stored in soft Tx FIFO buffers to master
 */
static void i2c_slave_tx(hal_i2c_channel_t chid)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;
	hal_i2c_buffer_t	*stxbuf = i2c_config->tx_soft_fifo;
	uint16_t			tx_current_index;
	uint16_t			i;

	tx_current_index = i2c_config->tx_soft_fifo_current_index;

	/* Write data to FIFO */
	for (i = 0; i < stxbuf[tx_current_index].length; i++)
		i2c_reg->DATA_CMD = stxbuf[tx_current_index].bytes[i];

	/* Enable TX_EMPTY interrupt */
	i2c_reg->INTR_MASK |= I2C_INTR_MASK_TX_EMPTY_MASK;
}

/*
 * @brief i2c_isr_handler
 * Internal interrupt service routine
 */
static void i2c_isr_handler(hal_i2c_channel_t chid)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;

	/* Check if TX_EMPTY interrupt occurred */
	if (i2c_reg->INTR_STAT & I2C_INTR_STAT_TX_EMPTY_MASK)
	{
		hal_i2c_buffer_t	*stxbuf = i2c_config->tx_soft_fifo;
		uint16_t			tx_next_index;

		/* Disable TX_EMPTY interrupt */
		i2c_reg->INTR_MASK &= (~I2C_INTR_MASK_TX_EMPTY_MASK);

		tx_next_index = i2c_config->tx_soft_fifo_current_index + 1;

		/*
		 * Check if the data or read requests
		 * stored in current buffer are not the last ones
		 */
		if (0 < stxbuf[tx_next_index].length)
		{
			i2c_config->tx_soft_fifo_current_index = tx_next_index;
			/* Set Tx FIFO threshold for triggering next TX_EMPTY interrupt */
			i2c_set_next_tx_threshold(chid);

			/* Check if i2c channel is operating as a master */
			if (I2C_MASTER == i2c_config->operation_mode)
			{
				/* Check if i2c channel is operating as a master-transmitter */
				if (I2C_TX_TRASMITTING == i2c_config->state_machine)
				{
					/* Transmit the next data in soft Tx FIFO buffers */
					i2c_master_tx(chid);
				}
				/* If i2c channel is operating as a master-receiver */
				else
				{
					/* Request next data */
					i2c_master_rx(chid);
				}
			}

			/* If i2c channel is operating as a slave-transmitter */
			else
			{
				/* Transmit the next data in soft Tx FIFO buffers */
				i2c_slave_tx(chid);
			}
		}
	}
	/* Check if RX_FULL interrupt occurred */
	else if (i2c_reg->INTR_STAT & I2C_INTR_STAT_RX_FULL_MASK)
	{
		/* Check if i2c channel is operating as a master-receiver */
		if (I2C_MASTER == i2c_config->operation_mode)
		{
			hal_i2c_buffer_t	*srxbuf = i2c_config->rx_soft_fifo;
			uint16_t			rx_current_index;
			uint16_t			i;

			rx_current_index = i2c_config->rx_soft_fifo_current_index;

			/* Read data from hardware Rx FIFO */
			for (i = 0; i < srxbuf[rx_current_index].length; i++)
				srxbuf[rx_current_index].bytes[i] = i2c_reg->DATA_CMD & 0xFF;

			/*
			 * Check if the received data stored in current buffer
			 * are not the last data
			 */
			if (0 < srxbuf[rx_current_index + 1].length)
			{
				/* Set in order to receive the next data */
				i2c_config->rx_soft_fifo_current_index++;
				i2c_reg->RX_TL = srxbuf[rx_current_index + 1].length - 1;
			}
		}
		/* If i2c channel is operating as a slave-receiver */
		else
		{
			/* Update state machine */
			i2c_config->state_machine = I2C_RX_RECEIVING;
			/* Call back function to upper layer */
			if(NULL != i2c_config->irq_handler)
				(*(i2c_config->irq_handler))(chid, I2C_RX_RECEIVING);
		}
	}
	/* Check if RD_REQ interrupt occurred */
	else if (i2c_reg->INTR_STAT & I2C_INTR_STAT_RD_REQ_MASK)
	{
		/* Clear pending interrupt */
		i2c_reg->CLR_RD_REQ;
		/* Update state machine */
		i2c_config->state_machine = I2C_READ_REQUESTED;
		/* Call back function to upper layer */
		if(NULL != i2c_config->irq_handler)
			(*(i2c_config->irq_handler))(chid, I2C_READ_REQUESTED);
	}
	/* Check if error interrupts occurred */
	else if (i2c_reg->INTR_STAT & I2C_INTR_STAT_TX_ABRT_MASK)
	{
		/* Clear pending interrupt */
		i2c_reg->CLR_TX_ABRT;
		/* Update state machine */
		i2c_config->state_machine = I2C_ERROR;
	}
	/* Check if STOP or RESTART interrupt occurred */
	else if ((i2c_reg->INTR_STAT & I2C_INTR_STAT_STOP_DET_MASK)
			|| (i2c_reg->INTR_STAT & I2C_INTR_STAT_RESTART_DET_MASK))
	{
		/* Clear pending interrupt */
		if (i2c_reg->INTR_STAT & I2C_INTR_STAT_STOP_DET_MASK)
			i2c_reg->CLR_STOP_DET;
		else
			i2c_reg->CLR_RESTART_DET;
		/*
		 * Check if i2c channel is operating as a transmitter
		 * and there is no error occurred
		 */
		if (I2C_TX_TRASMITTING == i2c_config->state_machine)
		{
			/* Update state machine */
			i2c_config->state_machine = I2C_TX_COMPLETED;
		}
		/*
		 * If i2c channel is operating as a receiver
		 * and there is no error occurred
		 */
		else if (I2C_RX_RECEIVING == i2c_config->state_machine)
		{
			/* Update state machine */
			i2c_config->state_machine = I2C_RX_COMPLETED;
		}
		/* Call back function to upper layer */
		if (NULL != i2c_config->irq_handler)
			(*(i2c_config->irq_handler))(chid, i2c_config->state_machine);
	}
	/* Other interrupts */
	else
	{
		/* Clear pending interrupt */
		i2c_reg->CLR_INTR;
	}
}

/*
 * i2c_push_to_sfifo
 * Sets Tx FIFO threshold for triggering next TX_EMPTY interrupt
 */
static void i2c_set_next_tx_threshold(hal_i2c_channel_t chid)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;
	uint16_t			tx_next_index;

	/* Get next soft Tx FIFO buffer index for transmission */
	tx_next_index = i2c_config->tx_soft_fifo_current_index + 1;

	/*
	 * Check if the next soft Tx FIFO buffer is not the last buffer
	 * for transmission
	 */
	if (0 < i2c_config->tx_soft_fifo[tx_next_index + 1].length)
	{
		/*
		 * Trigger TX_EMPTY interrupt
		 * when the next soft Tx FIFO buffer completes transmission
		 * (then hardware Tx FIFO reaches to a half size)
		 * in order to push another soft Tx FIFO buffer to hardware Tx FIFO
		 */
		i2c_reg->TX_TL = i2c_config->tx_soft_fifo[tx_next_index].length - 1;
	}
	else
	{
		/* Trigger TX_EMPTY interrupt when hardware Tx FIFO is empty */
		i2c_reg->TX_TL = 0;
	}
}

/*
 * i2c_push_to_sfifo
 * Pushes to source buffer into the destination buffer
 */
static void i2c_push_to_sfifo(hal_i2c_channel_t chid, hal_i2c_buffer_t *sbuf,
														hal_i2c_buffer_t *dbuf)
{
	i2c_config_t	*i2c_config = &(i2c_configs[chid]);
	i2c_reg_t		*i2c_reg = i2c_config->i2c_base;
	uint16_t			hw_fifo_half_depth;
	uint16_t			num_half_hwbufs;
	uint16_t			remain_byte_count;
	uint16_t			i;
	/* Store temporarily for reentrancy */
	uint8_t				*bytes = sbuf->bytes;

	/* Get the depth of a half of hardware FIFO */

	/* Check if destination buffer is soft Tx FIFO */
	if (dbuf == i2c_config->tx_soft_fifo)
	{
		hw_fifo_half_depth = (((i2c_reg->COM_PARAM &
							I2C_COMP_PARAM_TXBUF_DEPTH_MASK) >> 16) + 1) >> 1;
	}
	/* If destination buffer is soft Rx FIFO */
	else
	{
		hw_fifo_half_depth = (((i2c_reg->COM_PARAM &
							I2C_COMP_PARAM_RXBUF_DEPTH_MASK) >> 8) + 1) >> 1;
	}
	/*
	 * Get the number of soft FIFO buffers
	 * needed to store the half hardware FIFO buffers
	 */
	num_half_hwbufs = sbuf->length / hw_fifo_half_depth;
	/*
	 * Push the source buffers into the soft FIFO buffers
	 * so that the hardware FIFO buffer can reach a half of size
	 * if containing each soft FIFO buffer.
	 * This is used to create double buffer
	 * for faster transmission/reception and avoid data miss
	 */
	for (i = 0; i < num_half_hwbufs; i++)
	{
		dbuf[i].bytes = bytes;
		dbuf[i].length = hw_fifo_half_depth;
		bytes += hw_fifo_half_depth;
	}
	/*
	 * Push the remaining (last) data into the next soft FIFO buffer.
	 * This data size shall be less than a half of the hard FIFO depth
	 */
	remain_byte_count = sbuf->length - (i * hw_fifo_half_depth);
	dbuf[i].bytes = bytes;
	dbuf[i].length = remain_byte_count;
	/*
	 * Specify last soft FIFO buffer for transmission/reception
	 * by setting the length of next soft FIFO buffer to 0.
	 * (If variable remain_byte_count is equal to 0, soft FIFO buffer [i - 1]
	 * is the last buffer for transmission/reception.
	 * Otherwise (remain_byte_count > 0), soft FIFO buffer [i] is the last one)
	 */
	if (remain_byte_count > 0)
		dbuf[i + 1].length = 0;
}

/*
 * I2C0_IRQHandler
 * Internal I2C channel 0 interrupt handler
 */
static void I2C0_IRQHandler(void)
{
	i2c_isr_handler(0);
}

/*
 * I2C1_IRQHandler
 * Internal I2C channel 1 interrupt handler
 */
static void I2C1_IRQHandler(void)
{
	i2c_isr_handler(1);
}

/*
 * I2C2_IRQHandler
 * Internal I2C channel 2 interrupt handler
 */
static void I2C2_IRQHandler(void)
{
	i2c_isr_handler(2);
}

/*
 * I2C3_IRQHandler
 * Internal I2C channel 3 interrupt handler
 */
static void I2C3_IRQHandler(void)
{
	i2c_isr_handler(3);
}

/*
 * I2C4_IRQHandler
 * Internal I2C channel 4 interrupt handler
 */
static void I2C4_IRQHandler(void)
{
	i2c_isr_handler(4);
}

/*
 * I2C5_IRQHandler
 * Internal I2C channel 5 interrupt handler
 */
static void I2C5_IRQHandler(void)
{
	i2c_isr_handler(5);
}

/*
 * I2C6_IRQHandler
 * Internal I2C channel 6 interrupt handler
 */
static void I2C6_IRQHandler(void)
{
	i2c_isr_handler(6);
}

/*
 * I2C7_IRQHandler
 * Internal I2C channel 7 interrupt handler
 */
static void I2C7_IRQHandler(void)
{
	i2c_isr_handler(7);
}

/*
 * I2C8_IRQHandler
 * Internal I2C channel 8 interrupt handler
 */
static void I2C8_IRQHandler(void)
{
	i2c_isr_handler(8);
}

/*
 * I2C9_IRQHandler
 * Internal I2C channel 9 interrupt handler
 */
static void I2C9_IRQHandler(void)
{
	i2c_isr_handler(9);
}

/*
 * I2C10_IRQHandler
 * Internal I2C channel 10 interrupt handler
 */
static void I2C10_IRQHandler(void)
{
	i2c_isr_handler(10);
}

/*
 * I2C11_IRQHandler
 * Internal I2C channel 11 interrupt handler
 */
static void I2C11_IRQHandler(void)
{
	i2c_isr_handler(11);
}

/*
 * I2C12_IRQHandler
 * Internal I2C channel 12 interrupt handler
 */
static void I2C12_IRQHandler(void)
{
	i2c_isr_handler(12);
}

/*
 * I2C13_IRQHandler
 * Internal I2C channel 13 interrupt handler
 */
static void I2C13_IRQHandler(void)
{
	i2c_isr_handler(13);
}

/*
 * I2C14_IRQHandler
 * Internal I2C channel 14 interrupt handler
 */
static void I2C14_IRQHandler(void)
{
	i2c_isr_handler(14);
}

/*
 * I2C15_IRQHandler
 * Internal I2C channel 15 interrupt handler
 */
static void I2C15_IRQHandler(void)
{
	i2c_isr_handler(15);
}

/*
 * I2C16_IRQHandler
 * Internal I2C channel 16 interrupt handler
 */
static void I2C16_IRQHandler(void)
{
	i2c_isr_handler(16);
}

/*
 * I2C17_IRQHandler
 * Internal I2C channel 17 interrupt handler
 */
static void I2C17_IRQHandler(void)
{
	i2c_isr_handler(17);
}

/*
 * I2C18_IRQHandler
 * Internal I2C channel 18 interrupt handler
 */
static void I2C18_IRQHandler(void)
{
	i2c_isr_handler(18);
}

/*
 * I2CS0_IRQHandler
 * Internal I2C slave channel 0 interrupt handler
 */
static void I2CS0_IRQHandler(void)
{
	i2c_isr_handler(19);
}

/*
 * I2CS1_IRQHandler
 * Internal I2C slave channel 1 interrupt handler
 */
static void I2CS1_IRQHandler(void)
{
	i2c_isr_handler(20);
}

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE *******/
