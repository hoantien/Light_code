/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_axi2mipi.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-01-2016
 * @brief	This file contains functions used to initialize
 *			module AXI to MIPI
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_AXI2MIPI_H__
#define __HAL_AXI2MIPI_H__

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define -----------------------------------------------------------*/
/* AXI MIPI BRIDGE ADDR */
#define AXI_MIPI_TX_BRIDGE_0	((axi2mipi_regs_t *) AXI2MIPI0_BASE)
#define AXI_MIPI_TX_BRIDGE_1	((axi2mipi_regs_t *) AXI2MIPI1_BASE)

/* AXI2MIPI registers offset */
#define BRIDGE_CTRL_REG			0x000	/*Bridge Control */

#define DMA_SRC_A_REG			0x004	/* DMA Read Source Address */
#define DMA_TRANS_A_REG			0x008	/* DMA Read Transfer Size  */

#define DMA_SRC_B_REG			0x00C	/* DMA Read Source Address */
#define DMA_TRANS_B_REG			0x010	/* DMA Read Transfer Size  */

#define DMA_SRC_C_REG			0x014	/* DMA Read Source Address */
#define DMA_TRANS_C_REG			0x018	/* DMA Read Transfer Size  */

#define DMA_SRC_D_REG			0x01C	/* DMA Read Source Address */
#define DMA_TRANS_D_REG			0x020	/* DMA Read Transfer Size  */

#define INTR_MSK_REG			0x024	/* Interrupt Mask  */
#define INTR_STS_REG			0x028	/* Interrupt Status  */
#define INTR_CLR_REG			0x02C	/* Interrupt Clear  */

#define AXI_ERR_ADDR_REG		0x030	/* AXI Slave Error Address */
#define VC_SELECT_REG			0x034	/* Virtual Channel Selection */

#define IMG_CTRL_A_REG			0x038	/* Image Control  */
#define IMG_CTRL_B_REG			0x03C	/* Image Control  */
#define IMG_CTRL_C_REG			0x040	/* Image Control  */
#define IMG_CTRL_D_REG			0x044	/* Image Control  */

#define FRAME_CTRL_A_REG		0x048	/* Frame Control  */
#define FRAME_CTRL_B_REG		0x04C	/* Frame Control  */
#define FRAME_CTRL_C_REG		0x050	/* Frame Control  */
#define FRAME_CTRL_D_REG		0x054	/* Frame Control  */

#define SHORT_PACKED_CTRL_REG	0x80	/* Short packet control register */
#define USER_FRAME_NUM_REG_0	0x84	/* User Frame Number Register 0 */
#define USER_FRAME_NUM_REG_1	0x88	/* User Frame Number Register 1 */

#define HOR_CTRL_A_REG			0x100	/* Horizontal Control */
#define HOR_BLANK_A_REG			0x104

#define HOR_CTRL_B_REG			0x110	/* Horizontal Control */
#define HOR_BLANK_B_REG			0x114

#define HOR_CTRL_C_REG			0x120	/* Horizontal Control */
#define HOR_BLANK_C_REG			0x124

#define HOR_CTRL_D_REG			0x130	/* Horizontal Control */
#define HOR_BLANK_D_REG			0x134

#define TIMING_GEN_CTRL_REG		0x150	/* Timing Generator Control*/

/* Exported macro ------------------------------------------------------------*/
#define GET_AXI2MIPI_BASE_ADDR(insidx)	(AXI2MIPI0_BASE + insidx * 0x1000)

/* Read / Write MIPI register macro */
#define READ_AXI2MIPI_REG(insidx, regoffset) \
	(*(volatile uint32_t *)(GET_AXI2MIPI_BASE_ADDR(insidx) + regoffset))

#define WRITE_AXI2MIPI_REG(insidx, regoffset, regval) \
	((*(volatile uint32_t *)(GET_AXI2MIPI_BASE_ADDR(insidx) + regoffset))= (uint32_t)regval)

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief interrupt_flag
 * AXI to MIPI interrupt flags type definition
 */
typedef enum
{
	FE_INT_FIFO_A = BIT0,
	FE_INT_FIFO_B = BIT1,
	FE_INT_FIFO_C = BIT2,
	FE_INT_FIFO_D = BIT3,

	FS_INT_FIFO_A = BIT4,
	FS_INT_FIFO_B = BIT5,
	FS_INT_FIFO_C = BIT6,
	FS_INT_FIFO_D = BIT7,

	DMA_LINE_INT_FIFO_A = BIT8,
	DMA_LINE_INT_FIFO_B = BIT9,
	DMA_LINE_INT_FIFO_C = BIT10,
	DMA_LINE_INT_FIFO_D = BIT11,

	DMA_FRAME_INT_FIFO_A = BIT12,
	DMA_FRAME_INT_FIFO_B = BIT13,
	DMA_FRAME_INT_FIFO_C = BIT14,
	DMA_FRAME_INT_FIFO_D = BIT15,

	EMPTY_INT_FIFO_A = BIT16,
	EMPTY_INT_FIFO_B = BIT17,
	EMPTY_INT_FIFO_C = BIT18,
	EMPTY_INT_FIFO_D = BIT19,

	FULL_INT_FIFO_A = BIT20,
	FULL_INT_FIFO_B = BIT21,
	FULL_INT_FIFO_C = BIT22,
	FULL_INT_FIFO_D = BIT23,

	AXI_ERR_INT_FIFO_A = BIT24,
	AXI_ERR_INT_FIFO_B = BIT25,
	AXI_ERR_INT_FIFO_C = BIT26,
	AXI_ERR_INT_FIFO_D = BIT27
} axi2mipi_isr_flag_t;

/**
 * @brief axi2mipi_stream_mode_t
 * Stream mode status type definition
 */
typedef enum axi2mipi_stream_mode
{
	AXI2MIPI_SNAPSHOT_MODE	= 0,
	AXI2MIPI_ALU_MODE		= 1,
	AXI2MIPI_PREVIEW_MODE	= 2,
	AXI2MIPI_DISABLE_MODE	= 3
} axi2mipi_stream_mode_t;

#define IS_MIPI_STREAM_MODE(MODE)		(((MODE) == AXI2MIPI_SNAPSHOT_MODE)	|| \
										((MODE) == AXI2MIPI_ALU_MODE)		|| \
										((MODE) == AXI2MIPI_PREVIEW_MODE)	|| \
										((MODE) == AXI2MIPI_DISABLE_MODE))

/**
 * @brief flag_status_t
 * Flag status type definition
 */
typedef uint8_t flag_status_t;

#define IS_MIPI_STATUS(STATUS)	((STATUS == ENABLE) || (STATUS == DISABLE))

/**
 * @brief axi2mipi_channel_t
 * MIPI TX channel type definition
 */
typedef enum axi2mipi_channel
{
	AXI2MIPI_BRIDGE_0 = 0,
	AXI2MIPI_BRIDGE_1 = 1
} axi2mipi_channel_t;

#define IS_MIPI_CHANNEL(CHANNEL)	((CHANNEL == AXI2MIPI_BRIDGE_0) || \
									(CHANNEL == AXI2MIPI_BRIDGE_1))
#define NUM_AXI2MIPI_CHANNEL		2

/**
 * @brief mipi_data_type
 * MIPI data type
 */
typedef enum mipi_data_type
{
	AXI2MIPI_RAW8		= 0x2A,
	AXI2MIPI_RAW10		= 0x2B,
	AXI2MIPI_RAW12		= 0x2C,
	AXI2MIPI_RAW14		= 0x2D,
	AXI2MIPI_LIGHT_RAW	= 0x30
} axi2mipi_data_type_t;

#define IS_MIPI_DATA_TYPE(TYPE)		(((TYPE) == AXI2MIPI_RAW8)	|| \
									((TYPE) == AXI2MIPI_RAW10)	|| \
									((TYPE) == AXI2MIPI_RAW12)	|| \
									((TYPE) == AXI2MIPI_RAW14)	|| \
									((TYPE) == AXI2MIPI_LIGHT_RAW))

/**
 * @brief mipi_packed_t
 * Data from Rx is packed or unpacked format
 */
typedef enum mipi_packed
{
	AXI2MIPI_PACKED_DATA		= 0,
	AXI2MIPI_UNPACKED_DATA		= 1
} axi2mipi_packed_t;

#define IS_MIPI_PACKED(TYPE)		(((TYPE) == AXI2MIPI_PACKED_DATA) || \
									((TYPE) == AXI2MIPI_UNPACKED_DATA))
/*
 * @brief directly_pass_t
 * Data From RX Is Directly Pass
 */
typedef enum
{
	NONE_BYPASS = 0,
	BYPASS_FROM_RX1,
	BYPASS_FROM_RX2,
	BYPASS_FROM_RX3,
	BYPASS_FROM_RX4,
	BYPASS_FROM_RX5,
	BYPASS_FROM_RX6
} directly_pass_t;

#define IS_DIRECTLY_PASS(FROM)	(((FROM) == NONE_BYPASS)		|| \
								((FROM) == BYPASS_FROM_RX1)		|| \
								((FROM) == BYPASS_FROM_RX2)		|| \
								((FROM) == BYPASS_FROM_RX3)		|| \
								((FROM) == BYPASS_FROM_RX4)		|| \
								((FROM) == BYPASS_FROM_RX5)		|| \
								((FROM) == BYPASS_FROM_RX6))

/**
 * @brief axi2mipi_ff_t
 * This data type used for FIFO select
 */
typedef enum axi2mipi_ff
{
	AXI2MIPI_FIFO_A = BIT0,
	AXI2MIPI_FIFO_B = BIT1,
	AXI2MIPI_FIFO_C = BIT2,
	AXI2MIPI_FIFO_D = BIT3,
	AXI2MIPI_FIFO_ALL = 0x0f
} axi2mipi_ff_t;

#define IS_AXI2MIPI_FIFO(FIFO)		(((FIFO) & 0xFFFFFFF0) == 0)

/**
 * @brief axi2mipi_vc_for_ff_t
 * This data type used for selects virtual channel for FIFO
 */
typedef enum axi2mipi_vc_for_ff
{
	VC0_FOR_FIFO = 0,
	VC1_FOR_FIFO,
	VC2_FOR_FIFO,
	VC3_FOR_FIFO
} axi2mipi_vc_for_ff_t;

#define IS_VC_FOR_FIFO(VC)			(((VC) == VC0_FOR_FIFO)		|| \
									((VC) == VC1_FOR_FIFO)		|| \
									((VC) == VC2_FOR_FIFO)		|| \
									((VC) == VC3_FOR_FIFO))

/**
 * @brief axi2mipi_img_size_t
 * This data type used for configures image size
 */
typedef struct axi2mipi_img_size
{
	uint16_t width;
	uint16_t height;
} axi2mipi_img_size_t;

typedef enum
{
	IS_SLAVE_PORT	= 0,
	IS_MASTER_PORT	= 1
} axi_is_ms_or_slv_t;
/**
 * @brief axi2mipi_img_type_t
 * This data type used for configure image type format
 */
typedef struct axi2mipi_img_type
{
	axi2mipi_data_type_t tx_dt;	/* This field is type of data MIPI TX will send to VC */
	axi2mipi_data_type_t rx_dt;	/* This field should be same with data type of MIPI RX */
	axi2mipi_packed_t pack;
	directly_pass_t ff_bypass;
} axi2mipi_img_type_t;


typedef struct axi2mipi_wrap
{
	uint16_t wrap_num;			/* The line number to wrap the base address. */
	flag_status_t wrap_en;		/* Enable the DMA write wrapping function */
} axi2mipi_wrap_t;

/**
 * @brief axi2mipi_property_t
 * This data type used for configures stream function
 */
typedef struct axi2mipi_property
{
	/* Used to choose FIFOs (A, B, C, D) will configure,
	 * can use OR operation to selects multiple FIFO */
	axi2mipi_ff_t fifo_x;

	uint32_t src_addr;
	axi2mipi_stream_mode_t stream_mode;
	axi2mipi_img_type_t img_type;
	axi2mipi_img_size_t img_size;
	axi2mipi_vc_for_ff_t vc_for_ff;
	axi2mipi_wrap_t wrap;
} axi2mipi_property_t;

/**
 * @brief axi2mipi_hvsync_t
 * This data type used for configures horizontal and vertical
 */
typedef struct axi2mipi_hvsync
{
	/* Used to choose FIFOs (A, B, C, D) will configure,
	 * can use OR operation to selects multiple FIFO */
	axi2mipi_ff_t fifo_x;

	/* Horizontal control */
	uint16_t hde_width;/* FIFO Line Active (bytes/line) */
	uint16_t hs_width; /* FIFO Horizontal Sync(idi_clks/line) */

	uint16_t hfp_width; /* FIFO Horizontal Back Porch */
	uint16_t hbp_width; /* FIFO Horizontal Front Porch */
} axi2mipi_hsync_t;

/**
 * @brief axi2mipi_cont_frame_t
 * This data type used for configures continuous frame
 */
typedef enum axi2mipi_cont_frame
{
	NONE_CONTINUOUS = 0,
	FRAME_CONTINUOUS = 1,
} axi2mipi_cont_frame_t;

#define IS_FRAME_CONT(CONT)		(((CONT) == NONE_CONTINUOUS)		|| \
								((CONT) == FRAME_CONTINUOUS))
/**
 * @brief axi2mipi_callback
 * This data type used for call back function
 */
typedef void (*axi2mipi_callback)(axi2mipi_channel_t channel,
		axi2mipi_isr_flag_t interrupt_source, void *user_data);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief hal_axi2mipi_set_stream_property
 * This function used to configures stream image
 * @param channel: the channel index of AXI2MIPI
 * @param opt: the AXI to MIPI option
 * @retval none
 */
void hal_axi2mipi_set_stream_property(axi2mipi_channel_t channel, axi2mipi_property_t *opt);

/**
 * @brief hal_axi2mipi_hsync_cfg
 * This function used to configure horizontal and vertical sync
 * @param channel: the channel index of AXI2MIPI
 * @param hvsync: the horizontal and vertical options
 * @retval none
 */
void hal_axi2mipi_hsync_cfg(axi2mipi_channel_t channel, axi2mipi_hsync_t *hvsync);
/**
 * @brief hal_axi2mipi_init
 * This function used to initialize DPHY MIPI TX
 * @param channel: the channel index of AXI2MIPI
 * @retval none
 */
void hal_axi2mipi_init(axi2mipi_channel_t channel);

/**
 * @brief hal_axi2mipi_frame_valid
 * When this bit is set, it says the programming of register is valid
 * @param channel: the channel index of AXI2MIPI
 * @param fifo_x: FIFO (A, B, C, D), use OR operation to select multiple FIFO
 * @param valid_en: When enable, it says the programming of register is valid
 * @retval none
 */
void hal_axi2mipi_programming_valid(axi2mipi_channel_t channel,
									axi2mipi_ff_t fifo_x,
									flag_status_t valid_en);

/**
 * @brief hal_axi2mipi_start
 * This function used to start AXI to MIPI module
 * @param channel: the channel index of AXI2MIPI
 * @param fifo_x: the FIFO (A, B, C, D) will start option
 * @retval none
 */
void hal_axi2mipi_start(axi2mipi_channel_t channel, axi2mipi_ff_t fifo_x);

/**
 * @brief hal_axi2mipi_stop
 * This function used to stop AXI to MIPI module
 * @param channel: the channel index of AXI2MIPI
 * @param fifo_x: FIFO (A, B, C, D) will stop option
 * @retval none
 */
void hal_axi2mipi_stop(axi2mipi_channel_t channel, axi2mipi_ff_t fifo_x);

/**
 * @brief hal_axi2mipi_set_frame_mode
 * This function used to configure DMA frame continuous mode
 * @param channel: the channel index of AXI2MIPI
 * @param fifo_x: FIFO (A, B, C, D), use OR operation to select multiple FIFO
 * @param cont_mode: frame continuous or none continuous mode
 * @retval none
 */
void hal_axi2mipi_set_frame_mode(axi2mipi_channel_t channel,
								axi2mipi_ff_t fifo_x,
								axi2mipi_cont_frame_t cont_mode);

/**
 * @brief hal_axi2mipi_irq_mask
 * This function used to configure interrupt
 * @param channel: the channel index of AXI2MIPI
 * @param irq_sel: the interrupts source option
 * @param enable: enable or disable interrupt
 * @retval none
 */
void hal_axi2mipi_irq_mask(axi2mipi_channel_t channel,
							axi2mipi_isr_flag_t irq_sel,
							uint8_t enable);


/**
 * @brief hal_axi2mipi_irq_enable
 * This function used to enable interrupt for AXI to MIPI module
 * @param channel: the channel index of AXI2MIPI
 * @param irq_callback: the call back function
 * @param user_data: this pointer used to transfer data to interrupt handler
 * @retval none
 */
void hal_axi2mipi_irq_enable(axi2mipi_channel_t channel,
							axi2mipi_callback irq_callback,
							void *user_data);

/**
 * @brief hal_axi2mipi_irq_disable
 * This function used to disable interrupt for AXI to MIPI module
 * @param channel: the channel index of AXI2MIPI
 * @retval none
 */
void hal_axi2mipi_irq_disable(axi2mipi_channel_t channel);
void setup_tx_all(uint16_t width, uint16_t height);
unsigned int hal_axi2mipi_get_width_byte(uint16_t width, axi2mipi_data_type_t data_type,
								axi2mipi_stream_mode_t mode);
#ifdef __cplusplus
}
#endif
#endif /* __HAL_AXI2MIPI_H__ */
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
