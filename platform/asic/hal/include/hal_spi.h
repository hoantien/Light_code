/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_spi.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    01-Feb-2016
 * @brief   This file contains all the functions prototype of the SPI peripheral
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "hal_dma.h"

#ifdef __cplusplus
extern "C"
{
#endif
/* Private define ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* SPI base address */
#define SPI_MASTER				((spi_resgister_t *) SPIM_BASE)
#define SPI_SLAVE0				((spi_resgister_t *) SPIS0_BASE)
#define SPI_SLAVE1				((spi_resgister_t *) SPIS1_BASE)

/* Bit fields in CTRLR0 */
#define SPI_FRF_FRAME_OFFSET	21
#define SPI_FRAME_STD_SPI		(0x00 << SPI_FRF_FRAME_OFFSET)
#define SPI_FRAME_DUAL_SPI		(0x01 << SPI_FRF_FRAME_OFFSET)
#define SPI_FRAME_QUAD_SPI		(0x02 << SPI_FRF_FRAME_OFFSET)

#define SPI_DFS_32_OFFSET		16

#define SPI_CFS_OFFSET			12

#define SPI_SRL					(1 << 11)
#define SPI_SLVOE				(1 << 10)

#define SPI_TMOD_OFFSET			8
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TR				(0x0 << SPI_TMOD_OFFSET)  /* xmit & recv */
#define SPI_TMOD_TO				(0x1 << SPI_TMOD_OFFSET)  /* xmit only */
#define SPI_TMOD_RO				(0x2 << SPI_TMOD_OFFSET)  /* recv only */
#define SPI_TMOD_EPROMREAD		(0x3 << SPI_TMOD_OFFSET)  /* eeprom read mode */

#define SPI_SCOL				(1 << 7)
#define SPI_SCPH				(1 << 6)

#define SPI_FRF_OFFSET			4
#define SPI_FRF_SPI				(0x0 << SPI_FRF_OFFSET)
#define SPI_FRF_SSP				(0x1 << SPI_FRF_OFFSET)
#define SPI_FRF_MICROWIRE		(0x2 << SPI_FRF_OFFSET)
#define SPI_FRF_RESV			(0x3 << SPI_FRF_OFFSET)
#define SPI_DFS_16_OFFSET		0

/* Bit fields in SR (status register), 7 bits */
#define SR_MASK					0x7f		/* cover 7 bits */
#define SR_BUSY					(1 << 0)
#define SR_TF_NOT_FULL			(1 << 1)
#define SR_TF_EMPT				(1 << 2)
#define SR_RF_NOT_EMPT			(1 << 3)
#define SR_RF_FULL				(1 << 4)
#define SR_TX_ERR				(1 << 5)
#define SR_DCOL					(1 << 6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI_OFFSET	0
#define SPI_INT_TXEI		(1 << 0) /* Transmit FIFO empty interrupt mask */

#define SPI_INT_TXOI_OFFSET	1
#define SPI_INT_TXOI		(1 << 1) /* Transmit FIFO overflow interrupt mask */

#define SPI_INT_RXUI_OFFSET	2
#define SPI_INT_RXUI		(1 << 2) /* Receive FIFO underflow interrupt mask */

#define SPI_INT_RXOI_OFFSET	3
#define SPI_INT_RXOI		(1 << 3) /* Receive FIFO overflow interrupt mask  */

#define SPI_INT_RXFI_OFFSET	4
#define SPI_INT_RXFI		(1 << 4) /* Receive FIFO full interrupt mask      */

#define SPI_INT_MSTI_OFFSET	5
#define SPI_INT_MSTI		(1 << 5) /* multi-master contention interrupt mask*/

/* TX RX interrupt level threshold, max can be 256 */
#define SPI_INT_THRESHOLD	32

/* Bit fields in DMA Control Register */
#define TDMAE				(1 << 1)	/* transmit DMA enable */
#define RDMAE				(1 << 0)	/* receive DMA enable */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief spi_resgister_t
 * SPI registers structure definition
 */
typedef struct spi_register_t
{
	__IO uint32_t CTRL0;	/* Offset 0x00 - Control Register 0               */
	__IO uint32_t CTRL1;	/* Offset 0x04 - Control Register 1               */
	__IO uint32_t SSIENR;	/* Offset 0x08 - SSI Enable Register              */
	__IO uint32_t MWCR;		/* Offset 0x0c - Microwire Control Register       */
	__IO uint32_t SER;		/* Offset 0x10 - Slave Enable Register            */
	__IO uint32_t BAUDR;	/* Offset 0x14 - Baud Rate Select                 */
	__IO uint32_t TXFTLR;	/* Offset 0x18 - Transmit FIFO Threshold Level    */
	__IO uint32_t RXFTLR;	/* Offset 0x1c - Receive FIFO Threshold Level     */
	__I  uint32_t TXFLR;	/* Offset 0x20 - Transmit FIFO Level Register     */
	__I  uint32_t RXFLR;	/* Offset 0x24 - Receive FIFO Level Register      */
	__I  uint32_t SR;		/* Offset 0x28 - Status Register                  */
	__IO uint32_t IMR;		/* Offset 0x2c - Interrupt Mask Register          */
	__I  uint32_t ISR;		/* Offset 0x30 - Interrupt Status Register        */
	__I  uint32_t RISR;		/* Offset 0x34 - Raw Interrupt Status Register    */
	__I  uint32_t TXOICR;	/* Offset 0x38 - Tx FIFO Overflow Interrupt Clear */
	__I  uint32_t RXOICR;	/* Offset 0x3c - Rx FIFO Overflow Interrupt Clear */
	__I  uint32_t RXUICR;	/* Offset 0x40 - Rx FIFO Underflow Interrupt Clear*/
	__I  uint32_t MSTICR;	/* Offset 0x44 - Multi-Master Interrupt Clear     */
	__I  uint32_t ICR;		/* Offset 0x48 - Interrupt Clear Register         */
	__IO uint32_t DMACR;	/* Offset 0x4c - DMA Control Register             */
	__IO uint32_t DMATDLR;	/* Offset 0x50 - DMA Transmit Data Level          */
	__IO uint32_t DMARDLR;	/* Offset 0x54 - DMA Receive Data Level           */
	__I  uint32_t IDR;		/* Offset 0x58 - Identification Register          */
	__I  uint32_t VERSION;	/* Offset 0x5c - coreKit version ID register      */
	__IO uint32_t DR[36];	/* Offset 0x60 - 0xec: Data Register              */
	__IO uint32_t RXDLY;	/* Offset 0xf0 - RXD Sample Delay Register        */
	__IO uint32_t SPI_CTRLR0;	/* Offset 0xf4 - SPI control register         */
	__IO uint32_t RSVD_1;	/* Offset 0xf8 - Reserved location for future use */
	__IO uint32_t RSVD_2;	/* Offset 0xfc - Reserved location for future use */
} spi_resgister_t;

/**
 * @brief spi_status_t
 * SPI return type definition
 */
typedef enum spi_status_t
{
	SPI_OK,
	SPI_TIMEOUT,
	SPI_READY,
	SPI_BUSY,
	SPI_BUSY_TX,
	SPI_BUSY_RX,
	SPI_BUSY_TX_RX,
	SPI_TX_BLOCK_COMPLETED,
	SPI_RX_BLOCK_COMPLETED,
	SPI_TX_RX_BLOCK_COMPLETED,
	SPI_CHANNEL_BUSY,
	SPI_INVALID_LENGTH,
	SPI_INVALID_CHANNEL,
	SPI_INVALID_PRIORITY,
	SPI_ERROR,
	SPI_UNKNOWN_ERROR
} spi_status_t;

/**
 * @brief flag_status_t
 * Flag status type definition
 */
typedef unsigned char flag_status_t;

#define IS_STATUS(STATUS)	((STATUS == ENABLE) || (STATUS == DISABLE))

/**
 * @brief fifo_status_t
 * FIFO return status type definition
 */
typedef enum fifo_status
{
	FIFO_EMPTY		= 0,
	FIFO_NOT_EMPTY	= 1,
	FIFO_FULL		= 2
} fifo_status_t;

/**
 * @brief ssi_type_t
 * SSI interfaces type definition
 */
typedef enum ssi_type_t
{
	SSI_MOTO_SPI		= 0x00,	/* Motorola Serial Peripheral Interface (SPI) */
	SSI_TI_SSP			= 0x01,	/* Texas Instruments Serial Protocol (SPI) */
	SSI_NS_MICROWIRE	= 0x02	/* National Semiconductor Microwire */
} ssi_type_t;

#define IS_SSI_MODE(MODE)	((MODE == SSI_MOTO_SPI) || \
							 (MODE == SSI_TI_SSP)	|| \
							 (MODE == SSI_NS_MICROWIRE))

/**
 * @brief spi_type_t
 * SPI operation type definition
 */
typedef enum spi_type_t
{
	SPI_STD  = 0x00,	/* spi standard mode 	*/
	SPI_DUAL = 0x01,	/* spi dual mode		*/
	SPI_QUAD = 0x02		/* spi quad mode		*/
} spi_type_t;

#define IS_SPI_TYPE(TYPE)	((TYPE == SPI_STD)	|| \
							 (TYPE == SPI_DUAL)	|| \
							 (TYPE == SPI_QUAD))

/**
 * @brief spi_channel_t
 * SPI channel type definition
 */
typedef enum ssi_channel_t
{
	SSI_CH0 = 0,	/* Master channel */
	SSI_CH1 = 1,	/* slave channel */
	SSI_CH2 = 2		/* slave channel */
} ssi_channel_t;

#define IS_SSI_CHANNEL(CHANNEL)		((CHANNEL == SSI_CH0)	|| \
									(CHANNEL == SSI_CH1)	|| \
									(CHANNEL == SSI_CH2))

/**
 * @brief spi_mode_t
 * SPI data mode (clock phase and polarity) type definition
 */
typedef enum spi_mode
{
	SPI_MODE_0,	/* CPOL=0, CPHA=0 (MSB-First)*/
	SPI_MODE_1,	/* CPOL=0, CPHA=1 (MSB-First)*/
	SPI_MODE_2,	/* CPOL=1, CPHA=0 (MSB-First)*/
	SPI_MODE_3	/* CPOL=1, CPHA=1 (MSB-First)*/
} spi_mode_t;

#define IS_SPI_PHASE_MODE(MODE)		((MODE == SPI_MODE_0)	|| \
									(MODE == SPI_MODE_1)	|| \
									(MODE == SPI_MODE_2)	|| \
									(MODE == SPI_MODE_3))

/**
 * @brief spi_transfer_mode_t
 * SPI transfer mode type definition
 */
typedef enum spi_transfer_mode
{
	TX_AND_RX	= 0x00,		/* xmit & recv		*/
	TX_ONLY		= 0x01,		/* xmit only		*/
	RX_ONLY		= 0x02,		/* recv only		*/
	EEPROM_READ	= 0x03		/* eeprom read mode	*/
} spi_transfer_mode_t;

#define IS_SPI_TRANSFER_MODE(MODE)	((MODE == TX_AND_RX) || \
									(MODE == TX_ONLY)	 || \
									(MODE == RX_ONLY)	 || \
									(MODE == EEPROM_READ))

/**
 * @brief  spi_data_size_t
 * SPI data size type definition
 */
typedef enum spi_data_size
{
	DATASIZE_8BIT	= 8,
	DATASIZE_16BIT	= 16,
	DATASIZE_32BIT	= 32
} spi_data_size_t;

#define IS_SPI_DATA_SIZE(SIZE)		((SIZE == DATASIZE_8BIT) || \
									(SIZE == DATASIZE_16BIT) || \
									(SIZE == DATASIZE_32BIT))

/**
 * @brief  spi_dma_channel_t
 * SPI channel connect to DMA
 */
typedef enum spi_dma_channel
{
	SPI0_DMA_RX = 0,	/* CH0 - spim_dma_rx */
	SPI0_DMA_TX = 1,	/* CH1 - spim_dma_tx */
	SPI1_DMA_RX = 2,	/* CH2 - spi1_dma_rx */
	SPI1_DMA_TX = 3,	/* CH3 - spi1_dma_tx */
	SPI2_DMA_RX = 4,	/* CH4 - spi2_dma_rx */
	SPI2_DMA_TX = 5		/* CH5 - spi2_dma_tx */
} spi_dma_channel_t;

/**
 * @brief spi_buffer_t
 * Data buffer for transferring structure definition
 */
struct buffer_t
{
	uint16_t len;		/* Data Length		*/
	uint16_t counter;	/* Transfer counter	*/
	uint8_t *buf;		/* Data buffer		*/
};

/**
 * @brief spi_buffer_t
 * SPI data buffer structure definition
 */
typedef struct spi_buffer_t
{
	struct buffer_t rxbuf;
	struct buffer_t txbuf;
} spi_buffer_t;

/**
 * @brief spi_dma_t
 * SPI configure using DMA
 */
typedef struct spi_dma
{
	flag_status_t enable;
	hal_dma_channel_t chid;
	void (*callback_func)(void);
} spi_dma_t;

/**
 * @brief spi_config_t
 * SPI configuration structure definition
 */
typedef struct spi_config_t
{
	ssi_type_t			ssi_mode;	/*!< SSI mode :	 SPI, SSP, Microwire	*/
	ssi_channel_t		channel;	/*!< Select SPI channel					*/
	spi_type_t			spi_type;	/*!< Select SPI type: STD, DUAL, QUAD	*/
	spi_mode_t			spi_mode;	/*!< Select clock phase and polarity	*/
	spi_data_size_t		data_size;	/*!< Select size of a data transfer		*/
	spi_transfer_mode_t	transfer_mode; /*!< Select mode transfer			*/
	uint16_t 			master_data_frame; /*!< Number of data frames		*/
	uint32_t 			master_clk_freq;/*!< Select clock SSI clock frequency*/
	spi_dma_t			dma_tx;		/*!< Configure SPI using DMA for transmit*/
	spi_dma_t			dma_rx;		/*!< Configure SPI using DMA for receive */
} spi_config_t;

/**
 * @brief dma_config_t
 * DMA configuration structure definition
 */
typedef struct dma_config
{
	uint8_t dma_transmit_data_level;
	uint8_t dma_receive_data_level;
	flag_status_t dma_transmit_enable;
	flag_status_t dma_receive_enable;
} dma_config_t;

/**
 * @brief spi_interrupt_handle_t
 * SPI configuration structure definition
 */
typedef struct spi_interrupt_handle_t
{
	void (*multi_master_handler)(void);
	void (*rx_fifo_full_handler)(void);
	void (*rx_fifo_overflow_handler)(void);
	void (*rx_fifo_underflow_handler)(void);
	void (*tx_fifo_overflow_handler)(void);
	void (*tx_fifo_empty_handler)(void);
} spi_interrupt_handle_t;

/**
 * @brief  spi_interrupt_t
 * SPI interrupt configure structure definition
 */
typedef struct spi_interrupt
{
	uint8_t			tx_fifo_threshold; /*!< controls the threshold value for
											the transmit FIFO memory		*/

	uint8_t			rx_fifo_threshold; /*!< controls the threshold value for
											the receive FIFO memory			*/

	flag_status_t	multi_master_contention; /*!< Multi-Master contention
												interrupt mask				*/

	flag_status_t	rx_fifo_full;	/*!< Receive FIFO full interrupt mask
										Set when the receive FIFO is equal to
										or above its threshold value plus 1
										and requires service to prevent
										an overflow							*/

	flag_status_t	rx_fifo_overflow; /*!< Receive FIFO overflow interrupt
										Set when the receive logic attempts to
										place data into the receive FIFO after
										it has been completely filled		*/

	flag_status_t	rx_fifo_underflow;/*!< Receive FIFO underflow interrupt
										Set when an APB access attempts to read
										from the receive FIFO when it is empty*/

	flag_status_t	tx_fifo_overflow; /*!< Transmit FIFO overflow interrupt
										Set when an APB access attempts to
										write into the transmit FIFO after it
										has been completely filled			*/

	flag_status_t	tx_fifo_empty;	/*!< Transmit FIFO empty interrupt mask
										Set when the transmit FIFO is equal
										to or below its threshold value and
										requires service to prevent
										under-run */

	spi_interrupt_handle_t callback_handler; /*!< Interrupt handler functions*/
} spi_interrupt_init_t;

/**
 * @brief spi_hw_config_t
 * SPI hardware configuration structure
 */
typedef struct spi_hw_config
{
	spi_resgister_t *spix;
} spi_hw_config_t;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief hal_spi_disable
 * This function used to disable SSI module
 * @param *spi: point to SPI peripheral you will use
 * @retval none
 */
void hal_spi_disable(spi_config_t *spi);

/**
 * @brief hal_spi_enable
 * This function used to enable SSI module
 * @param *spi: point to SPI peripheral you will use
 * @retval none
 */
void hal_spi_enable(spi_config_t *spi);

/**
 * @brief hal_spi_reset
 * This function used to reset SSI module
 * @param *spi: point to SPI peripheral you will use
 * @retval none
 */
void hal_spi_reset(spi_config_t *spi);

/**
 * @brief hal_spi_init
 * Initialize SPIx peripheral
 * @param *spi: point to SPI peripheral you will use
 * @retval reference to #spi_status_t
 */
spi_status_t hal_spi_init(spi_config_t *spi);

/**
 * @brief hal_spi_interrupt_deinit
 * Deinitialize SPIx peripheral
 * @param *spi: point to SPI peripheral you will use
 * @retval none
 */
void hal_spi_interrupt_deinit(spi_config_t *spi);

/**
 * @brief hal_spi_interrupt_init
 * Initialize SPIx peripheral
 * @param *spi: point to SPI peripheral you will use
 * @param *interrupt: point to interrupt configuration pointer
 * @retval none
 */
void hal_spi_interrupt_init(spi_config_t *spi, spi_interrupt_init_t *interrupt);

/**
 * @brief hal_spi_dma_init
 * Initialize DMA SPIx peripheral
 * @param *spi: point to SPI peripheral you will use
 * @param *dma: configure DMA operation
 * @retval none
 */
void hal_spi_dma_init(spi_config_t *spi, dma_config_t *dma);

/**
 * @brief hal_spi_slave_select
 * this function is used to enable CS pin of slave device
 * @param spi_slave: SPI slave number you will use
 * @param value: ENABLE - enable chip select, DISABLE - disable chip select
 * @retval reference to #spi_status_t
 */
spi_status_t hal_spi_slave_select(uint8_t cs_pin, flag_status_t value);

/**
 * @brief hal_spi_transceiver
 * Transmit and receive a buffer data. Users must make sure that #CS pin on
 * device is presented before calling this function.
 * IMPORTANT :
 * 1 IF YOU USE THIS FUNCTION IN SLAVER MODE, YOU HAVE TO CHECK RETURN
 * STATUS TO MAKE SURE THAT TRANSFER IS COMPLETE AND NOT ERROR
 *
 * 2. IF YOU USE THIS FUNCTION IN DMA MODE, YOU SHOULD USING
 * "hal_dma_is_transfer_completed" FUNCTION TO
 * CHECK TRANSCEIVER COMPLETE IN CALLBACK FUNCTION
 *
 * eg: if(hal_dma_is_transfer_completed(SPI_DMA_RX_CHANNEL) == HAL_DMA_RESULT_YES)
 *		{
 *			// do something
 *		}
 *
 * @param *spi: point to SPI peripheral you will use
 * @param *buff: buffer structure which used for sending and receiving
 * @retval SPI_OK: transmission and receive complete, SPI_ERROR: not complete
 */
spi_status_t hal_spi_transceiver(spi_config_t *spi, spi_buffer_t *buff);

/**
 * @brief hal_spi_receive_buf
 * Receive a buffer data. Users must make sure that #CS pin on
 * device is presented before calling this function.
 * IMPORTANT :
 * 1. IF YOU USE THIS FUNCTION IN SLAVER MODE, YOU HAVE TO CHECK RETURN
 * STATUS TO MAKE SURE THAT TRANSFER IS COMPLETE AND NOT ERROR
 *
 * 2. IF YOU USE THIS FUNCTION IN DMA MODE, YOU SHOULD USING
 * "hal_dma_is_transfer_completed" FUNCTION TO
 * CHECK TRANSCEIVER COMPLETE IN CALLBACK FUNCTION
 *
 * eg: if(hal_dma_is_transfer_completed(SPI_DMA_RX_CHANNEL) == HAL_DMA_RESULT_YES)
 *		{
 *			// do something
 *		}
 *
 * @param *spi: point to SPI peripheral you will use
 * @param *rx_buf: buffer data will be store data from host
 * @param len: number of data of buffer
 * @retval SPI_OK: receive data finish, SPI_ERROR: error
 */
spi_status_t hal_spi_receive_buf(spi_config_t *spi,
									uint8_t *rx_buf,
									uint16_t len);

/**
 * @brief hal_spi_transmit_buf
 * Transmit a buffer data. Users must make sure that #CS pin on
 * device is presented before calling this function.
 * IMPORTANT : IF YOU USE THIS FUNCTION IN SLAVER MODE, YOU HAVE TO CHECK RETURN
 * STATUS TO MAKE SURE THAT TRANSFER IS COMPLETE AND NOT ERROR
 *
 * 2. IF YOU USE THIS FUNCTION IN DMA MODE, YOU SHOULD USING
 * "hal_dma_is_transfer_completed" FUNCTION TO
 * CHECK TRANSCEIVER COMPLETE IN CALLBACK FUNCTION
 *
 * eg: if(hal_dma_is_transfer_completed(SPI_DMA_TX_CHANNEL) == HAL_DMA_RESULT_YES)
 *		{
 *			// do something
 *		}
 *
 * @param *spi: point to SPI peripheral you will use
 * @param *rx_buf: buffer data will be sending
 * @param len: number of data of buffer
 * @retval SPI_OK: receive data finish, SPI_ERROR: error
 */
spi_status_t hal_spi_transmit_buf(spi_config_t *spi,
									uint8_t *rx_buf,
									uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_SPI_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
