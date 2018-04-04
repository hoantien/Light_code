/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_mipi2axi.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-01-2016
 * @brief   This file contains APIs for controlling MIPI2AXI (MIPI RX bridge)
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_MIPI2AXI_H_
#define __HAL_MIPI2AXI_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
#define MIPI2AXI_ASSERT(x)	assert_param(x == MIPI2AXI_OK)
/* MIPI2AXI register offset */
#define READY_REG			0x04
#define DPM_REG				0x10
#define VC_REMAP_REG		0x14
#define DT_REG_VC01			0x18
#define DT_REG_VC23			0x1C
#define ALU_SRC_REG			0x24
#define MIPI_TX_BASE_REG	0x2C
#define ALU_SIZE_REG		0x28
#define ALU_OP_REG			0x20
#define INT1_MASK_REG		0x30
#define SIGNAL_INT1_CLR_REG	0x34
#define INT1_ST_REG			0x38
#define INT2_MASK_REG		0x40
#define ERROR_INT2_CLR_REG	0x44
#define INT2_ST_REG			0x48
#define RX_FIFO_DMA_REG		0x54
#define VC_BYPASS_REG		0x60

#define VC0_FRAME_SEL		0x100
#define VC0_DEST_REG		0x104
#define VC0_SIZE_REG		0x108
#define VC0_WRAP_REG		0x10C

#define VC1_FRAME_SEL		0x110
#define VC1_DEST_REG		0x114
#define VC1_SIZE_REG		0x118
#define VC1_WRAP_REG		0x11C

#define VC2_FRAME_SEL		0x120
#define VC2_DEST_REG		0x124
#define VC2_SIZE_REG		0x128
#define VC2_WRAP_REG		0x12C

#define VC3_FRAME_SEL		0x130
#define VC3_DEST_REG		0x134
#define VC3_SIZE_REG		0x138
#define VC3_WRAP_REG		0x13C

/* Maximum MIPI2AXI channel */
#define MIPI2AXI_MAX_INS					6
/* Maximum virtual channel supported by this bridge */
#define MIPI2AXI_MAX_VC_NUM					4

/* Exported macro ------------------------------------------------------------*/
#define GET_INS_BASE_ADDR(insidx)	(MIPI2AXI0_BASE + insidx * 0x1000)
/* Read / Write MIPI register macro */
#define READ_MIPI2AXI_REG(insidx, regoffset) \
	(*(volatile uint32_t *)(GET_INS_BASE_ADDR(insidx) + regoffset))

#define WRITE_MIPI2AXI_REG(insidx, regoffset, regval) \
	(*(volatile uint32_t *)(GET_INS_BASE_ADDR(insidx) + regoffset) = (uint32_t)regval)
/* Exported typedef ----------------------------------------------------------*/

/**
 * @brief mipi2axi_status_t
 * Return code enumerations
 */
typedef enum mipi2axi_status
{
	MIPI2AXI_OK,
	MIPI2AXI_INVALID_ARG,
	MIPI2AXI_UNINITIALIZE,
	MIPI2AXI_ERR
} mipi2axi_status_t;
/**
 * @brief mipi2axi_irq_raw_num_t
 * Interrupt number enumeration
 */
typedef enum mipi2axi_irq_raw_num
{
	MIPI2AXI_SIGNAL_INTR = 0x38,
	MIPI2AXI_ERROR_INTR = 0x48
} mipi2axi_irq_raw_num_t;

#define IS_IRQ_RAW_NUM(NUM)		(((NUM) == MIPI2AXI_SIGNAL_INTR) || \
								((NUM) == MIPI2AXI_ERROR_INTR))

/**
 * @brief mipi2axi_irq_mask_num_t
 * Interrupt MASK
 */
typedef enum mipi2axi_irq_mask_num
{
	MIPI2AXI_SIGNAL_MASK_INTR = 0x30,
	MIPI2AXI_ERROR_MASK_INTR = 0x40
} mipi2axi_irq_mask_num_t;

#define IS_IRQ_MASK_NUM(NUM)	(((NUM) == MIPI2AXI_SIGNAL_MASK_INTR) || \
								((NUM) == MIPI2AXI_ERROR_MASK_INTR))

/**
 * @brief mipi2axi_raw_int1_detail_t
 * Raw interrupt 1 detail
 */
typedef enum mipi2axi_raw_int1_detail
{
	R1_VC3_FRAME_END		=	(1 << 31),
	R1_VC2_FRAME_END		=	(1 << 30),
	R1_VC1_FRAME_END		=	(1 << 29),
	R1_VC0_FRAME_END		=	(1 << 28),
	R1_VC3_FRAME_START		=	(1 << 27),
	R1_VC2_FRAME_START		=	(1 << 26),
	R1_VC1_FRAME_START		=	(1 << 25),
	R1_VC0_FRAME_START		=	(1 << 24),
	R1_R_DMA_FINISH			=	(1 << 20),
	R1_VC3_LINE_END			=	(1 << 15),
	R1_VC2_LINE_END			=	(1 << 14),
	R1_VC1_LINE_END			=	(1 << 13),
	R1_VC0_LINE_END			=	(1 << 12),
	R1_VC3_LINE_START		=	(1 << 11),
	R1_VC2_LINE_START		=	(1 << 10),
	R1_VC1_LINE_START		=	(1 << 9),
	R1_VC0_LINE_START		=	(1 << 8),
	R1_VC3_W_DMA_FINISH		=	(1 << 3),
	R1_VC2_W_DMA_FINISH		=	(1 << 2),
	R1_VC1_W_DMA_FINISH		=	(1 << 1),
	R1_VC0_W_DMA_FINISH		=	(1 << 0)
} mipi2axi_raw_int1_detail_t;
/**
 * @brief mipi2axi_raw_int2_detail_t
 * Raw interrupt 2 detail
 */
typedef enum mipi2axi_raw_int2_detail
{
	R2_DMA_W_ABORT			= (1 << 29),
	R2_DMA_W_FAIL			= (1 << 28),
	R2_RFIFO_UF_INT			= (1 << 24),
	R2_WFIFO_OV_INT			= (1 << 20),
	R2_PIXEL_MORE_INT3		= (1 << 19),
	R2_PIXEL_MORE_INT2		= (1 << 18),
	R2_PIXEL_MORE_INT1		= (1 << 17),
	R2_PIXEL_MORE_INT0		= (1 << 16),
	R2_LINE_MORE_INT3		= (1 << 15),
	R2_LINE_MORE_INT2		= (1 << 14),
	R2_LINE_MORE_INT1		= (1 << 13),
	R2_LINE_MORE_INT0		= (1 << 12),
	R2_LINE_LESS_INT3		= (1 << 11),
	R2_LINE_LESS_INT2		= (1 << 10),
	R2_LINE_LESS_INT1		= (1 << 9),
	R2_LINE_LESS_INT0		= (1 << 8),
	R2_PIXEL_LESS_INT3		= (1 << 7),
	R2_PIXEL_LESS_INT2		= (1 << 6),
	R2_PIXEL_LESS_INT1		= (1 << 5),
	R2_PIXEL_LESS_INT0		= (1 << 4)
} mipi2axi_raw_int2_detail_t;

typedef void (*mipi2axi_callback)(uint8_t iidx, uint32_t signal_irq_detail,
									uint32_t error_irq_detail, void *user_data);

typedef uint8_t	flag_status_t;
#define IS_MIPI2AXI_STATUS(STATUS)		(((STATUS) == ENABLE) || \
										((STATUS) == DISABLE))

typedef enum mipi2axi_channel
{
	MIPI2AXI0 = 0,
	MIPI2AXI1,
	MIPI2AXI2,
	MIPI2AXI3,
	MIPI2AXI4,
	MIPI2AXI5
} mipi2axi_channel_t;
#define IS_MIPI2AXI_CHANNEL(CHANNEL)	(((CHANNEL) == MIPI2AXI0)	|| \
										((CHANNEL) == MIPI2AXI1)	|| \
										((CHANNEL) == MIPI2AXI2)	|| \
										((CHANNEL) == MIPI2AXI3)	|| \
										((CHANNEL) == MIPI2AXI4)	|| \
										((CHANNEL) == MIPI2AXI5))
#define NUM_MIPI2AXI_CHANNEL			6


/* Data packing mode options for virtual channel */
typedef enum mipi2axi_vc_data_pack_mode
{
	MIPI2AXI_SNAPSHOT_MODE = 0,
	MIPI2AXI_ALU_MODE = 1,
	MIPI2AXI_PREVIEW_MODE = 2,
	MIPI2AXI_DISABLE_MODE = 3
} mipi2axi_stream_mode_t;

#define IS_VC_PACK_MODE(MODE)		(((MODE) == MIPI2AXI_SNAPSHOT_MODE) || \
									((MODE) == MIPI2AXI_ALU_MODE)		|| \
									((MODE) == MIPI2AXI_PREVIEW_MODE)	|| \
									((MODE) == MIPI2AXI_DISABLE_MODE))

/* Using OR operation to choose multiple VC */
typedef enum
{
	NO_REMAP = 0,
	VC0_REMAP = BIT0,
	VC1_REMAP = BIT1,
	VC2_REMAP = BIT3,
	VC3_REMAP = BIT4,
	VC_REMAP_ALL = 0x0f
} vc_mapping_t;

#define IS_VC_REMAP(VC_REMAP)		(((VC_REMAP) & 0xf0) == 0)

/* Type using for configure MIPI data type */
typedef enum mipi2axi_vc_ch
{
	MIPI2AXI_VC0_CH = BIT0,
	MIPI2AXI_VC1_CH = BIT1,
	MIPI2AXI_VC2_CH = BIT2,
	MIPI2AXI_VC3_CH = BIT3,
	MIPI2AXI_VC_ALL = 0x0f
} mipi2axi_vc_ch_t;

#define IS_VC_CHANNEL(CHANNEL)		(((CHANNEL) & 0xFFFFFFF0) == 0)

typedef enum mipi2axi_data_type
{
	MIPI2AXI_RAW8		= 0x2A,
	MIPI2AXI_RAW10		= 0x2B,
	MIPI2AXI_RAW12		= 0x2C,
	MIPI2AXI_RAW14		= 0x2D
} mipi2axi_data_type_t;

#define IS_RAW_DATA_TYPE(TYPE)		(((TYPE) == MIPI2AXI_RAW8)	|| \
									((TYPE) == MIPI2AXI_RAW10)	|| \
									((TYPE) == MIPI2AXI_RAW12)	|| \
									((TYPE) == MIPI2AXI_RAW14))

/* Data type using for configure ALU */
typedef enum mipi2axi_alu_op
{
	ALU_OP0				= 0,
	ALU_OP1_PLUS_OP0	= 1,
	ALU_OP1_DIV_OP0		= 2,
	ALU_OP0_DIV_OP1		= 3,
	ALU_NOT_OP0			= 4,
	ALU_NOT_OP1			= 5,
	ALU_OP1				= 6,
	ALU_OP0_OTHER		= 7
} mipi2axi_alu_op_t;

#define IS_ALU_OP(MODE)			(((MODE) == ALU_OP0)			|| \
								((MODE) == ALU_OP1_PLUS_OP0)	|| \
								((MODE) == ALU_OP1_DIV_OP0)		|| \
								((MODE) == ALU_OP0_DIV_OP1)		|| \
								((MODE) == ALU_NOT_OP0)			|| \
								((MODE) == ALU_NOT_OP1)			|| \
								((MODE) == ALU_OP1)				|| \
								((MODE) == ALU_OP0_OTHER))


/* line signal control */
typedef enum mipi2axi_line_signal
{
	IS_LINE = 1,	/* No line start/end short packet */
	NO_LINE = 0	/* There is line start/end short packet */
} mipi2axi_line_signal_t;

typedef struct
{
	 mipi2axi_line_signal_t line_start_signal;
	 mipi2axi_line_signal_t line_end_signal;
} mipi2axi_line_signal_ctr_t;

/* bypass mode */
typedef enum mipi2axi_bypass_mode
{
	DISABLE_BYPASS	= 0,
	IDI				= 1,
	PACK			= 2,
	UNPACK			= 3
} mipi2axi_bypass_mode_t;

#define IS_MIPI2AXI_BYPASS(MODE)		(((MODE) == DISABLE_BYPASS)		|| \
										((MODE) == IDI)					|| \
										((MODE) == PACK)				|| \
										((MODE) == UNPACK))

typedef struct mipi2axi_bypass
{
	/* Used to select virtual (VC0, VC1, VC2, VC3) will be configure,
	 * can use OR operation to selects multiple VC */
	mipi2axi_vc_ch_t vc_option;

	mipi2axi_bypass_mode_t mode;
} mipi2axi_bypass_t;

typedef struct mipi2axi_frame
{
	/* Used to select virtual (VC0, VC1, VC2, VC3) will be configure,
	 * can use OR operation to selects multiple VC */
	mipi2axi_vc_ch_t vc_option;

	uint8_t frame_num;
	flag_status_t frame_ena;
} mipi2axi_frame_t;

typedef struct
{
	uint32_t alu_src_addr;
	uint32_t dst_addr;
} mipi2axi_dst_address_t;

typedef struct
{
	uint16_t width;
	uint16_t height;
} mipi2axi_img_size_t;

typedef struct
{
	mipi2axi_data_type_t data_type;
	mipi2axi_vc_ch_t vc_will_captured;
} mipi2axi_data_t;

typedef struct
{
	/* Used to select virtual (VC0, VC1, VC2, VC3) will be configure,
	 * can use OR operation to selects multiple VC */
	mipi2axi_vc_ch_t				vc_option;

	mipi2axi_stream_mode_t			stream_mode;
	mipi2axi_dst_address_t			addr;
	mipi2axi_data_t					img_type;
	mipi2axi_img_size_t				img_size;
} mipi2axi_property_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief hal_mipi2axi_set_stream_property
 * This function used to configure stream parameters
 * @param channel: the channel index of MIPI2AXI
 * @param opt: the stream operation option
 * @retval none
 */
void hal_mipi2axi_set_stream_property(mipi2axi_channel_t channel,
									mipi2axi_property_t *opt);

/**
 * @brief hal_mipi2axi_bypass_cfg
 * This function used to configure MIPI2AXI bypass
 * @param channel: the channel index of MIPI2AXI
 * @param vc_option: the virtual channel will configure
 * @param mode: the modes of bypass
 * @retval none
 */
void hal_mipi2axi_bypass_cfg(mipi2axi_channel_t channel,
							mipi2axi_vc_ch_t vc_option,
							mipi2axi_bypass_mode_t mode);

/**
 * @brief hal_mipi2axi_alu_cfg
 * This function used to configure MIPI2AXI ALU
 * @param alu_op: the options of ALU mode
 * @param alu_ena: enable or disable ALU mode
 * @retval none
 */
void hal_mipi2axi_alu_cfg(mipi2axi_channel_t channel,
							mipi2axi_alu_op_t alu_op,
							flag_status_t alu_ena);

/**
 * @brief hal_mipi2axi_frame_control_cfg
 * This function used to configure MIPI2AXI frame
 * @param channel: the channel index of MIPI2AXI
 * @param vc_option: Used to select virtual (VC0, VC1, VC2, VC3) will be configure,
 *  can use OR operation to selects multiple VC
 * @param frame_num: This field is programmed by SW to select which frame is selected
 * @param frame_ena: Frame rate control enable/disable
 * @retval none
 */
void hal_mipi2axi_frame_control_cfg(mipi2axi_channel_t channel,
									mipi2axi_vc_ch_t vc_option,
									uint8_t frame_num,
									flag_status_t frame_ena);

/**
 * @brief hal_mipi2axi_wrap_cfg
 * This function used to configure MIPI2AXI wrap around
 * @param channel: the channel index of MIPI2AXI
 * @param vc_option: Used to select virtual (VC0, VC1, VC2, VC3) will be configure,
 *  can use OR operation to selects multiple VC
 * @param wrap_num: The line number to wrap the base address
 * @param wrap_en: Enable/Disable the DMA write wrapping function
 * @retval none
 */
void hal_mipi2axi_wrap_cfg(mipi2axi_channel_t channel,
							mipi2axi_vc_ch_t vc_option,
							uint16_t wrap_num,
							flag_status_t wrap_en);

/**
 * @brief hal_mipi2axi_vc_mapping
 * This function used to configure MIPI2AXI virtual channel remap
 * @param channel: the channel index of MIPI2AXI
 * @param vc_remap: the virtual channel remap option
 * @retval none
 */
void hal_mipi2axi_vc_mapping(mipi2axi_channel_t channel, vc_mapping_t vc_remap);

/**
 * @brief hal_mipi2axi_set_tx_base_addr
 * This function used to configure MIPI2AXI transmit base address
 * @param channel: the channel index of MIPI2AXI
 * @param addr: the virtual channel transmit base address option
 * @retval none
 */
void hal_mipi2axi_set_tx_base_addr(mipi2axi_channel_t channel, uint32_t addr);

/**
 * @brief hal_mipi2axi_dma_cfg
 * This function used to configure MIPI2AXI DMA
 * @param channel: the channel index of MIPI2AXI
 * @param write_abort_counter: The counter for DMA engine to abort DMA write transfer,
 * used if part of a line is lost, the unit is clock cycles.
 * @param write_fifo_threshold: write FIFO threshold. One for 16x128 bits
 * @retval none
 */
void hal_mipi2axi_dma_cfg(mipi2axi_channel_t channel,
							uint8_t write_abort_counter,
							uint8_t write_fifo_threshold);

/**
 * @brief hal_mipi2axi_start
 * Start the MIPI2AXI channel
 * @param channel: the channel index of MIPI2AXI
 * @retval none
 */
void hal_mipi2axi_start(mipi2axi_channel_t channel);

/**
 * @brief hal_mipi2axi_stop
 * Stop the MIPI2AXI channel
 * @param channel: the channel index of MIPI2AXI
 * @retval none
 */
void hal_mipi2axi_stop(mipi2axi_channel_t channel);

/**
 * @brief hal_mipi2axi_irq_mask
 * Initialize interrupt for MIPI2AXI channel
 * @param channel: the channel index of MIPI2AXI
 * @param irq_detail: the interrupt options,using OR operation to select multiple interrupt.
 * @param irq_num: select error interrupt or signal interrupt
 * @param enable: enable or disable interrupt
 * @retval none
 */
void hal_mipi2axi_irq_mask(mipi2axi_channel_t channel,
							mipi2axi_irq_mask_num_t irq_num,
							uint32_t irq_detail,
							uint8_t enable);

/**
 * @brief hal_mipi2axi_enable_irq
 * Enable interrupt for MIPI2AXI channel
 * @param channel: the channel index of MIPI2AXI
 * @param cb: the callback will be called when interrupt occurred
 * @param user_data: this pointer used to transfer data to interrupt handler
 * @retval none
 */
void hal_mipi2axi_irq_enable(mipi2axi_channel_t channel,
							mipi2axi_callback callback,
							void *user_data);

/**
 * @brief hal_mipi2axi_enable_irq
 * Enable interrupt for MIPI2AXI channel
 * @param channel: the channel index of MIPI2AXI
 * @retval none
 */
void hal_mipi2axi_irq_disable(mipi2axi_channel_t channel);

/**
 * @brief hal_mipi2axi_init
 * Initialize DPHY
 * @param channel: the channel index of MIPI2AXI
 * @retval none
 */
void hal_mipi2axi_init(mipi2axi_channel_t channel);

#ifdef __cplusplus
}
#endif
#endif /*__HAL_MIPI2AXI_H_ */

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.***** END OF FILE *********/
