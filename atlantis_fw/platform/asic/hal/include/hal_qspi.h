/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_qspi.h
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-10-2016
 * @brief	This file contains definitions of the Qspi driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_QSPI_H__
#define __HAL_QSPI_H__

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief qspi_mode_t
 * SPI mode type definition
 */
typedef enum spi_mode_reg
{
	PIO1_GO_MODE	= 0x00000001,	/* PIO mode 1. */
	PIO2_GO_MODE	= 0x00000002,	/* PIO mode 2. */
	PP_GO_MODE		= 0x00000100	/* Enable Page Program mode. */
} spi_mode_reg_t;

#define IS_QSPI_MODE(MODE)	(((MODE) == PIO1_GO_MODE) || \
							((MODE) == PIO2_GO_MODE) || \
							((MODE) == PP_GO_MODE))

/**
 * @brief sout_default_t
 * Default value type definition
 */
typedef enum sout_default
{
	SOUT_DEFAULT_LOW	= 0,
	SOUT_DEFAULT_HIGH	= 1
} sout_default_t;

/**
 * @brief sclk_default_t
 * SCLK default value type definition
 */
typedef enum sclk_default
{
	SCLK_DEFAULT_LOW	= 0,
	SCLK_DEFAULT_HIGH	= 1
} sclk_default_t;

/**
 * @brief qspi_lanes_t
 * PIO lanes type definition
 */
typedef enum qspi_lanes
{
	NO_LANE		= 0,	/* Not using lane */
	ONE_LANE	= 1,	/* Using 1 lane */
	TWO_LANES	= 2,	/* Using 2 lanes */
	FOUR_LANES	= 4		/* Using 4 lanes */
} qspi_lanes_t;

#define IS_QSPI_LANES(X)	(((X) == NO_LANE) || \
							((X) == ONE_LANE) || \
							((X) == TWO_LANES) || \
							((X) == FOUR_LANES))

/**
 * @brief pio_output_en_t
 * PIO OUTPUT EN type definition
 */
typedef enum pio_output_en
{
	NO_OUTPUT		= 0,	/* Not using output */
	ONE_OUTPUT		= 1,	/* S_oen0=1. */
	TWO_OUTPUTS		= 3,	/* S_oen1=1, S_oen0=1. */
	FOUR_OUTPUTS	= 15	/* S_oen3=1, S_oen2=1, S_oen1=1, S_oen0=1. */
} pio_output_en_t;

#define IS_QSPI_PIO_OUTPUT(X)	(((X) == NO_OUTPUT) || \
								((X) == ONE_OUTPUT) || \
								((X) == TWO_OUTPUTS) || \
								((X) == FOUR_OUTPUTS))

/**
 * @brief qspi_status_t
 * Qspi status type definition
 */
typedef enum qspi_status
{
	QSPI_ERROR	= 0,
	QSPI_BUSY,
	QSPI_TIMEOUT,
	QSPI_SUCCESS
} qspi_status_t;

/**
 * @brief qspi_mode_t
 * Qspi mode type
 */
typedef enum qspi_mode
{
	SINGLE_MODE	= 0,
	QSPI_MODE
} qspi_mode_t;

/**
 * @brief qspi_register_t
 * QSPI register type
 */
typedef struct qspi_register
{
	/* Basic register */
	__IO uint32_t SPI_MODE;			/* Offset 0x00 - SPI Mode				  */
	__IO uint32_t SPI_INT_EN;		/* Offset 0x04 - SPI interrupt enable	  */
	__O  uint32_t SPI_INT_CLR;		/* Offset 0x08 - SPI interrupt clear	  */
	__I  uint32_t SPI_INT_STATUS;	/* Offset 0x0C - SPI interrupt status	  */

	/* PIO MODE 1 register */
	__IO uint32_t PIO1_GO;			/* Offset 0x10 - PIO mode1 enable 		  */
	__IO uint32_t PIO1_CTRL;		/* Offset 0x14 - PIO mode1 control		  */
	__I  uint32_t RESERVED1[2];		/* Offset 0x18 - 0x1C : reserved		  */
	__IO uint32_t PIO1_STEP0_CONF;	/* Offset 0x20 - PIO mode 1 step 0 config */
	__IO uint32_t PIO1_STEP1_CONF;	/* Offset 0x24 - PIO mode 1 step 1 config */
	__IO uint32_t PIO1_STEP2_CONF;	/* Offset 0x28 - PIO mode 1 step 2 config */
	__IO uint32_t PIO1_STEP3_CONF;	/* Offset 0x2C - PIO mode 1 step 3 config */

	/* PIO MODE 2 register */
	__IO uint32_t PIO2_GO;			/* Offset 0x30 - PIO mode1 enable 		  */
	__IO uint32_t PIO2_CTRL;		/* Offset 0x34 - PIO mode1 control		  */
	__I  uint32_t RESERVED2[2];		/* Offset 0x38 - 0x3C : reserved		  */
	__IO uint32_t PIO2_STEP0_CONF;	/* Offset 0x40 - PIO mode 2 step 0 config */
	__IO uint32_t PIO2_STEP1_CONF;	/* Offset 0x44 - PIO mode 2 step 1 config */
	__IO uint32_t PIO2_STEP2_CONF;	/* Offset 0x48 - PIO mode 2 step 2 config */
	__IO uint32_t PIO2_STEP3_CONF;	/* Offset 0x4C - PIO mode 2 step 3 config */

	/* XIP MODE register */
	__IO uint32_t RESERVED3;		/* Offset 0x50 - Reserved				  */
	__IO uint32_t XIP_CTRL;			/* Offset 0x54 - XIP mode control		  */
	__I  uint32_t RESERVED4[2];		/* Offset 0x58 - 0x5C : reserved		  */
	__IO uint32_t XIP_CMD_CODE;		/* Offset 0x60 - XIP mode command code	  */
	__IO uint32_t XIP_CMD_CONF;		/* Offset 0x64 - XIP mode command config  */
	__IO uint32_t RESERVED5;		/* Offset 0x68 - reserved				  */
	__IO uint32_t XIP_ADDR_CONF;	/* Offset 0x6C - XIP mode address config  */
	__IO uint32_t XIP_DUMMY_CODE;	/* Offset 0x70 - XIP Mode dummy code	  */
	__IO uint32_t XIP_DUMMY_CONF;	/* Offset 0x74 - XIP Mode dummy config	  */
	__IO uint32_t XIP_DATA_CONF;	/* Offset 0x78 - XIP Mode RX data config  */
	__IO uint32_t RESERVED6;		/* Offset 0x7C - reserved				  */

	/* PAGE PROGRAM register */
	__IO uint32_t PP_GO;			/* Offset 0x80 - Page program mode enable */
	__IO uint32_t PP_CTRL;			/* Offset 0x84 - Page program mode control*/
	__IO uint32_t RESERVED7[2];		/* Offset 0x88 - 0x8C: reserved			  */
	__IO uint32_t PP_CMD_CODE;		/* Offset 0x90 - Page program mode command*/
	__IO uint32_t PP_CMD_CONF;		/* Offset 0x94 - Page program mode command
																configure     */
	__IO uint32_t PP_ADDR_CODE;		/* Offset 0x98 - Page program mode address
																code		  */
	__IO uint32_t PP_ADDR_CONF;		/* Offset 0x9C - Page program mode address
																configure	  */
	__IO uint32_t PP_DUMMY_CODE;	/* Offset 0xA0 - Page program mode dummy
																code		  */
	__IO uint32_t PP_DUMMY_CONF;	/* Offset 0xA4 - Page program mode dummy
																configure	  */
	__IO uint32_t PP_DATA_CONF;		/* Offset 0xA8 - Page program mode TX data
																configure	  */
	__IO uint32_t RESERVED8[21];	/* Offset 0xAC - reserved				  */

	union
	{	/* Offset 0x100 - 0x17C: transmit buffer  */
		__IO uint32_t TX_BUFF_REG[32];
		__IO uint8_t TX_BUFF_REG8[32 * 4];
	};

	union
	{	/* Offset 0x180 - 0x1FC: receive buffer	  */
		__IO uint32_t RX_BUFF_REG[32];
		__IO uint8_t RX_BUFF_REG8[32 * 4];
	};
} qspi_register_t;

/**
 * @brief qspi_ctrl_t
 * Qspi control register type
 */
typedef struct qspi_ctrl
{
	/* The divisor of serial clock. Value 0 is not allowed. */
	uint8_t			div_sclk;
	/* The value of chip select. */
	uint8_t			chip_select;
	/* Default value when SOUT1 is idle. */
	sout_default_t	sout1_default:1;
	/* Default value when SOUT2 is idle. */
	sout_default_t	sout2_default:1;
	/* Default value when SOUT3 is idle. */
	sout_default_t	sout3_default:1;
	/* The SCLK default value when it’s idle. For Serial mode1, the value
	should be 0. And 1 for mode3. */
	sclk_default_t	sclk_default:1;
	/* The number of cycle after transfer finished. The recommend value is 1. */
	uint32_t		post_cycle:2;
	/* The number of cycle before start transfer. The recommend value is 2. */
	uint32_t		pre_cycle:2;
	/* The number of idle cycle. The recommend value is 2. */
	uint32_t		idle_cycle:2;
} qspi_ctrl_t;

/**
 * @brief pio_step_conf_t
 * PIO_STEP_CONF register type
 */
typedef struct pio_step_conf
{
	/* The total number of SCLK cycle. */
	uint32_t		cycle:10;
	/* Reserved. */
	uint32_t		Reserved1:6;
	/* PIO lanes. */
	qspi_lanes_t	pio_lanes:3;
	/* Reserved. */
	uint32_t		Reserved2:1;
	/* Control the SPI output port: S_oen0~S_oen3. */
	pio_output_en_t	pio_output_en:4;
} pio_step_conf_t;

/**
 * @brief pp_conf_t
 * Qspi page program configuration register type
 */
typedef struct pp_conf
{
	/* The total number of SCLK cycle. */
	uint32_t		pp_cmd_cycle:10;
	/* Reserved. */
	uint32_t		Reserved:6;
	/* Using lanes. */
	qspi_lanes_t	pp_lanes:3;
} pp_conf_t;

/**
 * @brief qspi_ctrl_mode_t
 */
typedef struct qspi_ctrl_mode
{
	qspi_ctrl_t		pio1_ctrl;
	qspi_ctrl_t		pio2_ctrl;
	qspi_ctrl_t		pp_ctrl;
} qspi_ctrl_mode_t;

/**
 * @brief pio_mode_t
 * Qspi pio mode
 */
typedef struct pio_mode
{
	pio_step_conf_t		pio_step0_conf;
	pio_step_conf_t		pio_step1_conf;
	pio_step_conf_t		pio_step2_conf;
	pio_step_conf_t		pio_step3_conf;
} pio_mode_t;

/**
 * @brief page_program_mode_t
 * Qspi page program mode
 */
typedef struct page_program_mode
{
	uint32_t		pp_cmd_code; /* The command code of Page Program mode. */
	pp_conf_t		pp_cmd_conf;
	uint32_t		pp_addr_code; /* The Address of Page Program mode. */
	pp_conf_t		pp_addr_conf;
	uint32_t		pp_dummy_code; /* The Dummy cycle code of \
										Page Program mode. */
	pp_conf_t		pp_dummy_conf;
	pp_conf_t		pp_data_conf;
} page_program_mode_t;

/**
 * @brief qspi_transfer_cmd_t
 * Qspi transfer command structure
 */
typedef struct qspi_transfer_cmd
{
	spi_mode_reg_t		spi_mode_reg;
	pio_mode_t			pio_mode1;
	pio_mode_t			pio_mode2;
	page_program_mode_t	pp_mode;
	__IO uint32_t		*tx_buff_reg;
	__IO uint8_t		*tx_buff_reg8;
	__IO uint32_t		*rx_buff_reg;
	__IO uint8_t		*rx_buff_reg8;
} qspi_transfer_cmd_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/*
 * @brief qspi_init
 * Init qspi hardware
 * @param *qspi_ctrl_mode : pointer structure initialize qspi control register.
 * @return	void
 */
void qspi_init(qspi_ctrl_mode_t *qspi_ctrl_mode);

/*
 * @brief flash_sector_erase
 * Qspi transfer function
 * @param *qspi_transfer_cmd : pointer structure use setting command
 *								to transfer.
 * @return qspi status
 */
qspi_status_t transfer_cmd(qspi_transfer_cmd_t *qspi_transfer_cmd);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_QSPI_H__ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd. ********END OF FILE*******/
