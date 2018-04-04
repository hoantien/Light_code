/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_spi.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_spi driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_hal_spi.h"
#include "it_log_swapper.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct hal_spi_test
{
	ssi_channel_t   CC; 	/**! Channel */
	ssi_type_t      PP; 	/**! Protocol */
	spi_type_t      MM; 	/**! Multiplex mode */
	spi_mode_t      SS; 	/**! Data sampling mode */
	spi_data_size_t ZZ; 	/**! Frame size */
	spi_transfer_mode_t TT; /**! Transmit mode */
	uint32_t DDDD; 			/**! Clock frequency */
	uint16_t NN; 			/**! Number of data frame */
	uint16_t LL; 			/**! Number of testing bytes */
	uint16_t XXXX; 			/**! 16 bits Host command */
	uint16_t UU;			/**! Slave ID (Camera) */
} hal_spi_test;

typedef enum
{
    CMD_REQUEST = 0, /**! HW CMD at request mode */
    CMD_RESPOND = 1   /**! HW CMD respond mode */
}spi_hw_cmd_t;

typedef enum
{
	CMD_NEGATIVE = 0,
	CMD_POSITIVE = 1,
}cmd_t;

/* Private define ------------------------------------------------------------*/
#define BUFF_SIZE 32
/** Default configuration */
#define SPI_DEFAULT_CONFIG \
							{ 					\
								SSI_MOTO_SPI, 	\
								SSI_CH0,		\
								SPI_STD,		\
								SPI_MODE_0,		\
								DATASIZE_8BIT,	\
								TX_AND_RX,		\
								0,				\
								0x32,			\
								{DISABLE, HAL_DMA_CH1, NULL},\
								{DISABLE, HAL_DMA_CH1, NULL},\
							}
/** IRQ's default configuration */
#define SPI_DEFAULT_IRQ_CFG  						\
							{ 						\
								SPI_INT_THRESHOLD,	\
								SPI_INT_THRESHOLD,	\
								DISABLE,			\
								DISABLE,			\
								DISABLE,			\
								DISABLE,			\
								DISABLE,			\
								DISABLE,			\
								{			\
									NULL, 	\
									NULL, 	\
									NULL, 	\
									NULL, 	\
									NULL, 	\
									NULL  	\
								}\
							}
/* Private macro -------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Exported global variables --------------------------------------------------*/
LOCAL volatile uint32_t tx_counter  = 0;
LOCAL volatile uint32_t rx_counter  = 0;
LOCAL volatile bool 	dma_tx_done = FALSE;
LOCAL volatile bool 	dma_rx_done = FALSE;
/* Static functions ----------------------------------------------------------*/
/**
 * @brief identify hardware command mode or generate hardware command mode
 * @param[in] cmd  : cmd mode (request or respond)
 * @param[out] NA
 * @return  0   : inactive CMD
 *          1   : activated CMD
 *          -1  : error
 * @Detail identify hardware command mode or generate hardware command mode
 *         this function is only available for test and module is testing at
 *         transmit only mode or receive only mode.
 */
LOCAL int it_hal_spi_hw_cmd(spi_hw_cmd_t cmd, cmd_t dir);

/**
 * @brief To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *    mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *    mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 */
LOCAL int it_hal_spi_001(char** argv, int argc);

/**
 * @brief To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *  mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *  mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 */
LOCAL int it_hal_spi_002(char** argv, int argc);

/**
 * @brief To verify that: Asic could received data from Host in interrupt mode
 *  without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could received data from Host in interrupt mode
 *  without DMA
 */
LOCAL int it_hal_spi_003(char** argv, int argc);

/**
 * @brief To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 */
LOCAL int it_hal_spi_004(char** argv, int argc);

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 */
LOCAL int it_hal_spi_005(char** argv, int argc);

/**
 * @brief Input parameters for testcase
 * @detail Input parameters for testcase
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_spi_006(char** argv, int argc);

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 */
LOCAL int it_hal_spi_007(char** argv, int argc);

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *        in using DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *         in using DMA
 */
LOCAL int it_hal_spi_008(char** argv, int argc);

/**
 * @brief To verify that: Asic could receive data from camera slave in interrupt
 *                        mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could receive data from camera slave in interrupt
 *                        mode with DMA is enabled
 */
LOCAL int it_hal_spi_009(char** argv, int argc);

/**
 * @brief To verify that: Asic could transmit data according to host request in
 *                        interrupt mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmit data according to host request in
 *                        interrupt mode with DMA is enabled
 */
LOCAL int it_hal_spi_010(char** argv, int argc);

/**
 * @brief To verify that: Asic could receives data from host in interrupt mode
 *                        with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could receives data from host in interrupt mode
 *                         with DMA is enabled
 */
LOCAL int it_hal_spi_011(char** argv, int argc);

/**
 * @brief To verify that: Asic could transmits data to camera slave in interrupt
 *                        mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmits data to camera slave in interrupt
 *                         mode with DMA is enabled
 */
LOCAL int it_hal_spi_012(char** argv, int argc);

/**
 * @brief To verify that: Asic that verify performance API SPI
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmits data to camera slave in interrupt
 *                         mode with DMA is enabled
 */
LOCAL int it_hal_spi_013(char** argv, int argc);

/**
 * @brief To verify that: Asic that verify performance API SPI
 *   without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 */
LOCAL int it_hal_spi_014(char** argv, int argc);
/**
 * @brief Input parameters for testcase
 * @detail Input parameters for testcase
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL hal_spi_test config_testcase(char** argv, int argc)
{
	hal_spi_test spi_test;

	/**! Get Protocol via argv[0] */
	spi_test.PP = strtol(argv[0], NULL, 16);

	/**! Get Multiplex mode via argv[1] */
	spi_test.MM = strtol(argv[1], NULL, 16);

	/**! Get Data sampling mode via argv[2] */
	spi_test.SS = strtol(argv[2], NULL, 16);

	/**! Get Frame size via argv[3] */
	spi_test.ZZ = strtol(argv[3], NULL, 16);

	/**! Get Transmit mode via argv[4] */
	spi_test.TT = strtol(argv[4], NULL, 16);

	/**! Get Clock divider via argv[5] */
	spi_test.DDDD = strtol(argv[5], NULL, 16);

	/**! Get Number of testing bytes via argv[6] */
	spi_test.LL = strtol(argv[6], NULL, 10);

	/**! Get 16 bits Host command via argv[7] */
	spi_test.XXXX = strtol(argv[7], NULL, 16);

	/**! get Slave ID */
	if (argc == 9)
		spi_test.UU = strtol(argv[8], NULL, 16);
	else
		spi_test.UU = 0x00;

	return spi_test;
}

/**
 * @brief Input parameters for convert_data_size
 * @detail Input parameters for convert data size
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 	 : spi_data_size_t
 */
LOCAL spi_data_size_t convert_data_size(uint8_t data)
{
	switch(data)
	{
		case 0:
		{
			return DATASIZE_8BIT;
		}
		case 1:
		{
			return DATASIZE_16BIT;
		}
		case 2:
		{
			return DATASIZE_32BIT;
		}
		default:
		{
			console_putstring("Unsupport frame size.\r\n");
			return DATASIZE_8BIT;
		}
	}

}

/**
 * @brief tx_fifo_overflow callback function
 * @detail tx_fifo_overflow callback function
 * @param[in] NA
 * @param[out] tx_counter
 */
LOCAL void it_spi_tx_fifo_overflow_handler(void)
{
	tx_counter++;
}

/**
 * @brief rx_fifo_overflow callback function
 * @detail rx_fifo_overflow callback function
 * @param[in] NA
 * @param[out] rx_counter
 */
LOCAL void it_spi_rx_fifo_overflow_handler(void)
{
	rx_counter++;
}

/**
 * @brief SPI's DMA transmit done callback
 * @param[in] NA
 * @param[out] tx_counter
 * @return NA
 * @details SPI's DMA transmit done callback
 */
LOCAL void it_spi_dma_tx_done(void)
{
	dma_tx_done = TRUE;
}

/**
 * @brief SPI's DMA transmit done callback
 * @param[in] NA
 * @param[out] tx_counter
 * @return NA
 * @details SPI's DMA transmit done callback
 */
LOCAL void it_spi_dma_rx_done(void)
{
	dma_rx_done = TRUE;
}

/* Exported global variables --------------------------------------------------*/
LOCAL it_map_t it_spi_tests_table[] = {
		{"SPI_001", it_hal_spi_001},
		{"SPI_002", it_hal_spi_002},
		{"SPI_003", it_hal_spi_003},
		{"SPI_004", it_hal_spi_004},
		{"SPI_005", it_hal_spi_005},
		{"SPI_006", it_hal_spi_006},
		{"SPI_007", it_hal_spi_007},
		{"SPI_008", it_hal_spi_008},
		{"SPI_009", it_hal_spi_009},
		{"SPI_010", it_hal_spi_010},
		{"SPI_011", it_hal_spi_011},
		{"SPI_012", it_hal_spi_012},
		{"SPI_013", it_hal_spi_013},
		{"SPI_014", it_hal_spi_014},

		{"",  NULL}
};

LOCAL hal_gpio_t    hw_cmd_gpio  =  {GPIO_PORTA, GPIO_PIN_6, GPIO_DIR_IN};
LOCAL volatile bool hw_cmd_enable = FALSE;
/* Exported functions --------------------------------------------------------*/

/**
 * @brief SPI module's testing handler
 * @detail to SPI module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_spi_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_spi_tests_table);
	if (-1 != index)
	{
		return it_spi_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}


/**
 * @brief To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *    mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *    mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 */
LOCAL int it_hal_spi_001(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t  spi              = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint8_t dummy[4] = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH1;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num * (1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	uint8_t* tmp  = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		log_printf("Malloc error\r\n");
		if (data) free(data);
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
		tmp[index]  = 0;
	}

	/**! Setting TX overflow IRQ profile */
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			               it_spi_tx_fifo_overflow_handler;
	tx_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/** Dummy send to receive CMD from host */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, dummy, cmd_frame_num));

	volatile uint32_t time_out = 5000000;
	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--time_out));

	/** Check time out */
	qc_assert(time_out);

	/**! Send confirmation */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/** Dummy read to remove garbage received from CMD */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, dummy, cmd_frame_num));

	/**! Receive data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Transferring data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));

		/** Dummy receive to avoid RX Fifo overflow */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &tmp[index], chunk));

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify IRQ */
	qc_assert(!tx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *  mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that:
 *  - Transfer data according to master request by Host (Upstream) in interrupt
 *  mode without DMA.
 *  - Number of data frame does not effect to SPI module operation when it is
 *  Slave.
 */
LOCAL int it_hal_spi_002(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t          spi      = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH1;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num * (1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	/**! Setting TX overflow IRQ profile */
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			               it_spi_tx_fifo_overflow_handler;
	tx_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

	volatile uint32_t time_out = 5000000;

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/** Check time out */
	qc_assert(time_out);

	/**! Send confirmation */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Verify IRQ */
	qc_assert(!tx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could received data from Host in interrupt mode
 *        without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that: Asic could received data from Host in interrupt mode
 *  without DMA
 */
LOCAL int it_hal_spi_003(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
//	uint8_t dummy[4] = {00, 00, 00, 00};
	uint8_t dummy[4] = {0xA5, 0xA5, 0xA5, 0xA5};
	uint16_t cmd_frame_num = 0;
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH2;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num * (1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	uint8_t* tmp  = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		if (data) free(data);
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Reset data buffer */
	memset(data, 0x00, length);
	memset(tmp , 0x00, length);

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.rx_fifo_overflow_handler = \
			                     it_spi_rx_fifo_overflow_handler;
	rx_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

//	/**! Initialize HW CMD to support multiple tests */
//	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	while(1)
	{
		while (SPI_OK != hal_spi_transmit_buf(&spi, dummy, cmd_frame_num));
		while (SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num));
	}
	/** Dummy send to receive CMD from host */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, dummy, cmd_frame_num));

	volatile uint32_t time_out = 5000000;
	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--time_out));
	/** Check time out */
	qc_assert(time_out);

	/**! Send confirmation */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/** Dummy read to remove garbage received from CMD */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, dummy, cmd_frame_num));

	/**! Receive data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Dummy transmit to avoid TX FIFO under low IRQ */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &tmp[index], chunk));
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Verify IRQ */
	qc_assert(!rx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}
/**
 * @brief To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 */
LOCAL int it_hal_spi_004(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;

	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH2;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num*(1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = 0;
	}

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.rx_fifo_overflow_handler = \
			                     it_spi_rx_fifo_overflow_handler;
	rx_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	volatile uint32_t time_out = 5000000;
	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--time_out));
	/** Check time out */
	qc_assert(time_out);

	/**! Received data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Verify IRQ */
	qc_assert(!rx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *        in interrupt mode without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 */
LOCAL int it_hal_spi_005(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num*(1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data = malloc(length);
	uint8_t* tmp  = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		if (data) free(data);
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
		tmp[index]  = 0;
	}

	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;
	/**! Reset TX FIFO callback counter */
	tx_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Set direction as input */
	hw_cmd_gpio.direction = GPIO_DIR_IN;
	/* Initialize hardware CMD */
	hal_gpio_init(&hw_cmd_gpio);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
//	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));

	while (1)
	{
		/** Transmit command */
		while(SPI_OK != hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));
		while(SPI_OK != hal_spi_receive_buf (&spi, cmd, cmd_frame_num));
	}

	/**! Dummy read to remove garbage RX FIFO data */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Clear CMD buffer */
	memset(cmd, 0x00, 4);

	/**! Delay 5us */
	_delay_us(5);

	/** Transfer dummy to read confirmation from slave */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Received confirmation */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));

		/**! Received to remove garbage bytes in RX FIFO */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &tmp[index], chunk));

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Verify confirmation from slave */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[1] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)));
	}
	else
	{
		qc_assert((cmd[0] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)));
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 */
LOCAL int it_hal_spi_006(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num * (1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data = malloc(length);

	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Initialize HW CMD to support multiple tests */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));

	/**! Transfer command */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *  in interrupt mode without DMA
 */
LOCAL int it_hal_spi_007(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num*(1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data  = malloc(length);
	uint8_t* dummy = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!dummy)
	{
		log_printf("Malloc error\r\n");
		if(data) free(data);
		return -1;
	}

	/**! Prepare transmitted data */
	memset(data, 0x00, length);
	memset(dummy, 0x00, length);

	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Set direction as input */
	hw_cmd_gpio.direction = GPIO_DIR_IN;
	/* Initialize hardware CMD */
	hal_gpio_init(&hw_cmd_gpio);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
//	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));

	/** Transmit command */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Dummy read to remove garbage RX FIFO data */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Clear CMD buffer */
	memset(cmd, 0x00, 4);

	/**! Delay 2us */
	_delay_us(2);

	/** Transfer dummy to read confirmation from slave */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Received confirmation */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Transfer dummy to read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &dummy[index], chunk));
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Verify confirmation from slave */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[1] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)));
	}
	else
	{
		qc_assert((cmd[0] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could send data down stream to camera slave
 *        in using DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could send data down stream to camera slave
 *         in using DMA
 */
LOCAL int it_hal_spi_008(char** argv, int argc)
{
	return SPI_OK;
}

/**
 * @brief To verify that: Asic could receive data from camera slave in interrupt
 *                        mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that: Asic could receive data from camera slave in interrupt
 *                        mode with DMA is enabled
 */
LOCAL int it_hal_spi_009(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	volatile uint32_t timeout = 0;
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	/**! Configure operation */
	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num*(1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data = malloc(length);
	uint8_t* dummy = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!dummy)
	{
		log_printf("Malloc error\r\n");
		if(data) free(data);
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	/**! Clear dummy data */
	memset(dummy, 0x00, length);

	/**! Reset data buffer */
	memset(data, 0x00, length);

	/**! Setup FIFO */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;
	/**! Reset fifo callback counter */
	tx_counter = 0;

	/**! Configure DMA */
	spi.dma_tx.enable = ENABLE;
	spi.dma_tx.chid = HAL_DMA_CH1;
	spi.dma_tx.callback_func = it_spi_dma_tx_done;
	spi.dma_rx.enable = ENABLE;
	spi.dma_rx.chid = HAL_DMA_CH8;
	spi.dma_rx.callback_func = it_spi_dma_rx_done;

	/**! Reset DMA flags */
	dma_tx_done = FALSE;
	dma_rx_done = FALSE;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Initialize HW CMD to support multiple tests */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));

	/** Transmit command */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for dma IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));

	/**! Verify timeout */
	qc_assert(timeout);

	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/**! Dummy read to remove garbage RX FIFO data */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));

	/**! Verify timeout */
	qc_assert(timeout);

	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Clear CMD buffer */
	memset(cmd, 0x00, 4);

	/** Transfer dummy to read confirmation from slave */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Received confirmation */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Transfer dummy to read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &dummy[index], chunk));
		/**! Waiting for dma IRQ done */
		timeout = 1000;
		while ((TRUE != dma_tx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_tx_done = FALSE;
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));
		/**! Waiting for DMA IRQ done */
		timeout = 1000;
		while ((TRUE != dma_rx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_rx_done = FALSE;
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Verify confirmation from slave */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[1] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)));
	}
	else
	{
		qc_assert((cmd[0] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could transmit data according to host request in
 *                        interrupt mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmit data according to host request in
 *                        interrupt mode with DMA is enabled
 */
LOCAL int it_hal_spi_010(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint8_t dummy[4] = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH1;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num * (1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	uint8_t* tmp = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		log_printf("Malloc error\r\n");
		if (data) free(data);
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	/**! Setting TX overflow IRQ profile */
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			               it_spi_tx_fifo_overflow_handler;
	tx_counter = 0;

	/**! Configure DMA */
	spi.dma_tx.enable = ENABLE;
	spi.dma_tx.chid = HAL_DMA_CH1;
	spi.dma_tx.callback_func = it_spi_dma_tx_done;
	spi.dma_rx.enable = ENABLE;
	spi.dma_rx.chid = HAL_DMA_CH8;
	spi.dma_rx.callback_func = it_spi_dma_rx_done;

	/**! Reset DMA flags */
	dma_tx_done = FALSE;
	dma_rx_done = FALSE;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

	volatile uint32_t timeout = 5000000;

	/** Dummy send to receive CMD from host */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, dummy, cmd_frame_num));

	/**! Waiting for dma IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--timeout));
	/** Check time out */
	qc_assert(timeout);

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Send confirmation */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for dma IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/** Dummy read to remove garbage received from CMD */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, dummy, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Receive data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));
		/**! Waiting for dma IRQ done */
		timeout = 1000;
		while ((TRUE != dma_tx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_tx_done = FALSE;

		/** Dummy receive to avoid RX FIFO overflow */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &tmp[index], chunk));
		/**! Waiting for DMA IRQ done */
		timeout = 1000;
		while ((TRUE != dma_rx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_rx_done = FALSE;

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify IRQ */
	qc_assert(!tx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could receives data from host in interrupt mode
 *                        with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could receives data from host in interrupt mode
 *                         with DMA is enabled
 */
LOCAL int it_hal_spi_011(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint8_t dummy[4] = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH1;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num * (1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	uint8_t* tmp = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Reset data buffer */
	memset(data, 0x00, length);
	memset(tmp, 0x00, length);

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.rx_fifo_overflow_handler = \
			                     it_spi_rx_fifo_overflow_handler;
	/**! Reset RX FIFO callback counter */
	rx_counter = 0;

	/**! Configure DMA */
	spi.dma_tx.enable = ENABLE;
	spi.dma_tx.chid = HAL_DMA_CH1;
	spi.dma_tx.callback_func = it_spi_dma_tx_done;
	spi.dma_rx.enable = ENABLE;
	spi.dma_rx.chid = HAL_DMA_CH8;
	spi.dma_rx.callback_func = it_spi_dma_rx_done;

	/**! Reset DMA flags */
	dma_tx_done = FALSE;
	dma_rx_done = FALSE;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Enable SPI module */
	hal_spi_enable(&spi);

	volatile uint32_t timeout = 5000000;

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/** Dummy send to receive CMD from host */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, dummy, cmd_frame_num));

	/**! Waiting for dma IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--timeout));
	/** Check time out */
	qc_assert(timeout);

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Send confirmation */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for dma IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/** Dummy read to remove garbage received from CMD */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, dummy, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Receive data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}

		/**! Send dummy to avoid TX empty IRQ */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &tmp[index], chunk));

		/**! Waiting for dma IRQ done */
		timeout = 1000;
		while ((TRUE != dma_tx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_tx_done = FALSE;

		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));

		/**! Waiting for DMA IRQ done */
		timeout = 1000;
		while ((TRUE != dma_rx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_rx_done = FALSE;

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Verify IRQ */
	qc_assert(!rx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic could transmits data to camera slave in interrupt
 *                        mode with DMA is enabled
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmits data to camera slave in interrupt
 *                         mode with DMA is enabled
 */
LOCAL int it_hal_spi_012(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	volatile uint32_t timeout = 0;
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num*(1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data = malloc(length);
	uint8_t* tmp = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Verify memory allocation */
	if(!tmp)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}
	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;
	/**! Reset TX FIFO callback counter */
	tx_counter = 0;

	/**! Configure DMA */
	spi.dma_tx.enable = ENABLE;
	spi.dma_tx.chid = HAL_DMA_CH1;
	spi.dma_tx.callback_func = it_spi_dma_tx_done;
	spi.dma_rx.enable = ENABLE;
	spi.dma_rx.chid = HAL_DMA_CH8;
	spi.dma_rx.callback_func = it_spi_dma_rx_done;

	/**! Reset DMA flags */
	dma_tx_done = FALSE;
	dma_rx_done = FALSE;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));

	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);

	/**! Initialize HW CMD to support multiple tests */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));

	/** Transmit command */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/**! Dummy read to remove garbage RX FIFO data */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Clear CMD buffer */
	memset(cmd, 0x00, 4);

	/** Transfer dummy to read confirmation from slave */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_tx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_tx_done = FALSE;

	/**! Received confirmation */
	qc_assert(SPI_OK == hal_spi_receive_buf(&spi, cmd, cmd_frame_num));

	/**! Waiting for DMA IRQ done */
	timeout = 1000;
	while ((TRUE != dma_rx_done) && (--timeout));
	/**! Verify timeout */
	qc_assert(timeout);
	/**! Reset DMA flag */
	dma_rx_done = FALSE;

	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		/**! Transmit data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));

		/**! Waiting for DMA IRQ done */
		timeout = 1000;
		while ((TRUE != dma_tx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_tx_done = FALSE;

		/**! Dummy read to avoid RX overflow IRQ */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &tmp[index], chunk));

		/**! Waiting for DMA IRQ done */
		timeout = 1000;
		while ((TRUE != dma_rx_done) && (--timeout));
		/**! Verify timeout */
		qc_assert(timeout);
		/**! Reset DMA flag */
		dma_rx_done = FALSE;

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Verify confirmation from slave */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[1] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)));
	}
	else
	{
		qc_assert((cmd[0] == (ret_config.XXXX & 0xFF)) \
			   && (cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)));
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}
/**
 * @brief To verify that: Asic that verify performance API SPI
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could transmits data to camera slave in interrupt
 *                         mode with DMA is enabled
 */
LOCAL int it_hal_spi_013(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4];
	uint16_t length = 0;
	uint16_t cmd_frame_num = 0;
	uint16_t frames_num = 0;
	uint16_t select = 0;
	hal_gpio_t gpio = {0, 0, 1};
	/**! Verify input parameters */
	if ((10 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);
	select = (uint16_t)strtol(argv[9], NULL, 10);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH0;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	frames_num = ret_config.LL;
	length = frames_num * (1<<ret_config.ZZ);

	/**! Prepare CMD data frame */
	memset(cmd, 0x00, 4);
	cmd_frame_num = (2 < (1<<ret_config.ZZ) ? 1 :(2/(1<<ret_config.ZZ)));

	/**! Transmit data command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		cmd[0] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[1] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}
	else
	{
		cmd[0] = (uint8_t)(ret_config.XXXX & 0xFF);
		cmd[1] = (uint8_t)((ret_config.XXXX >> 8) & 0xFF);
		cmd[2] = 0x00;
		cmd[3] = 0x00;
	}

	/**! Provide buffer */
	uint8_t* data = malloc(length);

	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = index;
	}

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.tx_fifo_overflow_handler = \
			                     it_spi_tx_fifo_overflow_handler;

	/**! Reset all test point to default */
	qc_assert_reset();
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
		/**! Initializing SPI module by configured data */
		qc_assert(SPI_OK == hal_spi_init(&spi));
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Initialize HW CMD to support multiple tests */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_POSITIVE);
	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/** Select number slave */
	qc_assert(SPI_OK == hal_spi_slave_select(ret_config.UU, ENABLE));
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Transfer command */
	qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, cmd, cmd_frame_num));
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Transmit data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}
		if(4 == select)
		{
			hal_gpio_set_high(&gpio);
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_transmit_buf(&spi, &data[index], chunk));
		if(4 == select)
		{
			hal_gpio_set_low(&gpio);
		}
		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/** Check interrupt call back */
	qc_assert(!tx_counter);

	/**! Clear HW CMD */
	it_hal_spi_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}

/**
 * @brief To verify that: Asic that verify performance API SPI
 *   without DMA
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @Detail To verify that: Asic could received data from Host in interrupt mode
 *   without DMA
 */
LOCAL int it_hal_spi_014(char** argv, int argc)
{
	hal_spi_test ret_config;
	spi_config_t         spi       = SPI_DEFAULT_CONFIG;
	spi_interrupt_init_t interrupt = SPI_DEFAULT_IRQ_CFG;
	uint16_t index = 0;
	uint8_t cmd[4]   = {00, 00, 00, 00};
	uint16_t cmd_frame_num = 0;
	uint8_t select = 0;
	hal_gpio_t gpio = {0, 0, 1};
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	ret_config = config_testcase(argv,argc);

	/*! Get select SPI */
	select = (uint8_t)strtol(argv[8], NULL, 10);

	spi.ssi_mode = ret_config.PP;
	spi.channel = SSI_CH1;
	spi.spi_type = ret_config.MM;
	spi.spi_mode = ret_config.SS;
	spi.data_size = convert_data_size(ret_config.ZZ);
	spi.transfer_mode = ret_config.TT;
	spi.master_data_frame = 0;
	spi.master_clk_freq = ret_config.DDDD;

	cmd_frame_num = (2 < (1<<ret_config.ZZ)) ? 1 : (2/(1<<ret_config.ZZ));

	/**! Provide buffer */
	uint16_t frames_num = ret_config.LL;
	uint16_t length = frames_num*(1<<ret_config.ZZ);

	uint8_t* data = malloc(length);
	/**! Verify memory allocation */
	if(!data)
	{
		log_printf("Malloc error\r\n");
		return -1;
	}

	/**! Prepare transmitted data */
	for (index = 0; index < length; index++)
	{
		data[index] = 0;
	}

	/**! Setting IRQ profile */
	interrupt.rx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.tx_fifo_threshold = SPI_INT_THRESHOLD - 1;
	interrupt.callback_handler.rx_fifo_overflow_handler = \
			                     it_spi_rx_fifo_overflow_handler;
	rx_counter = 0;

	/*! Initialize GPIO */
	hal_gpio_init(&gpio);

	/**! Reset all test point to default */
	qc_assert_reset();

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initializing SPI module by configured data */
	qc_assert(SPI_OK == hal_spi_init(&spi));
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initializing interrupt with indication callback function */
	hal_spi_interrupt_init(&spi, &interrupt);
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Enable SPI module */
	hal_spi_enable(&spi);
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	while(SPI_OK != it_hal_spi_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	volatile uint32_t time_out = 5000000;

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Read command */
	while((SPI_OK != hal_spi_receive_buf(&spi, cmd, cmd_frame_num))
			                                      && (--time_out));
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/** Check time out */
	qc_assert(time_out);

	/**! Received data */
	for (index = 0; frames_num > 0;)
	{
		uint16_t chunk = 0;
		if (frames_num > SPI_INT_THRESHOLD)
		{
			chunk = SPI_INT_THRESHOLD;
		}
		else
		{
			chunk = frames_num;
		}

		if(4 == select)
		{
			hal_gpio_set_high(&gpio);
		}
		/**! Read data */
		qc_assert(SPI_OK == hal_spi_receive_buf(&spi, &data[index], chunk));
		if(4 == select)
		{
			hal_gpio_set_low(&gpio);
		}

		/**! Increase size */
		index += chunk * (1<<ret_config.ZZ);
		/**! Reduce frame number */
		frames_num -= chunk;
	}

	/**! Check received command */
	if(DATASIZE_8BIT == spi.data_size)
	{
		qc_assert((cmd[0] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[1] == (ret_config.XXXX & 0xFF)));
	}
	else
	{
		qc_assert((cmd[1] == ((ret_config.XXXX >> 8) & 0xFF)) \
			   && (cmd[0] == (ret_config.XXXX & 0xFF)));
	}

	/**! Verify data */
	for (index = 0; index < length; index++)
	{
		if (data[index] != index)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Verify IRQ */
	qc_assert(!rx_counter);

	/** Free data memory */
	free(data);

	/**! Judgment */
	qc_report();

	return SPI_OK;
}
/**
 * @brief identify hardware command mode or generate hardware command mode
 * @param[in] cmd  : cmd mode (request or respond)
 * @param[out] NA
 * @return  0   : inactive CMD
 *          1   : activated CMD
 *          -1  : error
 * @Detail identify hardware command mode or generate hardware command mode
 *         this function is only available for test and module is testing at
 *         transmit only mode or receive only mode.
 */
LOCAL int it_hal_spi_hw_cmd(spi_hw_cmd_t cmd, cmd_t dir)
{
    if (CMD_REQUEST == cmd) /**! Request */
    {
		#ifdef ASB
        /**! Provide high level output to trigger slave */
    	if ((GPIO_DIR_IN == hw_cmd_gpio.direction) || (!hw_cmd_enable))
    	{
    		/**! Set direction as output */
    		hw_cmd_gpio.direction = GPIO_DIR_OUT;

    		/**! Force set HW as output pin */
            hal_gpio_init(&hw_cmd_gpio);

            /**! Mark HW CMD as enabled */
            hw_cmd_enable = TRUE;
    	}

        /**! Set indicated pin as HIGH to start CMD */
    	if (dir == CMD_POSITIVE)
    		hal_gpio_set_high(&hw_cmd_gpio);
    	else
    		hal_gpio_set_low(&hw_cmd_gpio);
		#elif P2
    	hal_pwm_set_high(PWM0_CH4);
		#endif
        return SPI_OK;
    }
    else                    /**! Respond */
    {
        /**! Verify input */
        if ((hal_gpio_level_t)dir == hal_gpio_read(&hw_cmd_gpio)) return SPI_OK;
    }
    /**! Return error */
    return -1;
}


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
