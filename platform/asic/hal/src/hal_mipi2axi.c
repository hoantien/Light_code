/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_mipi2axi.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    July-01-2016
 * @brief   This file contains APIs for controlling MIPI2AXI (MIPI RX bridge)
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "assert.h"
#include "hal_mipi2axi.h"
#include "board_config.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "os.h"
#include "task.h"

/* Private defines -----------------------------------------------------------*/
/* MIPI 2 CSI registers*/
#define	PHY_TEST_CTRL1_REG		0x54
#define	PHY_TEST_CTRL0_REG		0x50
#define	PHY_SHUTDOWNZ_REG		0x40
#define	CSI2_RESETN_REG			0x08
#define	N_LANES_REG				0x04
#define	PHY_RESET_REG			0x44

/* RX FIFO default value */
#define SHIFT_TO_BIT0			0
#define SHIFT_TO_BIT1			1
#define SHIFT_TO_BIT2			2
#define SHIFT_TO_BIT3			3
#define SHIFT_TO_BIT4			4
#define SHIFT_TO_BIT5			5
#define SHIFT_TO_BIT6			6
#define SHIFT_TO_BIT7			7
#define SHIFT_TO_BIT8			8
#define SHIFT_TO_BIT9			9
#define SHIFT_TO_BIT10			10
#define SHIFT_TO_BIT11			11
#define SHIFT_TO_BIT12			12
#define SHIFT_TO_BIT13			13
#define SHIFT_TO_BIT14			14
#define SHIFT_TO_BIT15			15
#define SHIFT_TO_BIT16			16
#define SHIFT_TO_BIT17			17
#define SHIFT_TO_BIT18			18
#define SHIFT_TO_BIT19			19
#define SHIFT_TO_BIT20			20
#define SHIFT_TO_BIT21			21
#define SHIFT_TO_BIT22			22
#define SHIFT_TO_BIT23			23
#define SHIFT_TO_BIT24			24
#define SHIFT_TO_BIT25			25
#define SHIFT_TO_BIT26			26
#define SHIFT_TO_BIT27			27
#define SHIFT_TO_BIT28			28
#define SHIFT_TO_BIT29			29
#define SHIFT_TO_BIT30			30
#define SHIFT_TO_BIT31			31

#define	EIGHT_BITS_PER_PIXEL				8
#define	TEN_BITS_PER_PIXEL					10
#define	TWELVE_BITS_PER_PIXEL				12
#define	FOURTEEN_BITS_PER_PIXEL				14

#define VC_0		0
#define VC_1		1
#define VC_2		2
#define VC_3		3

#define	H_HALF_DWORD_MASK			0x0000ffff
#define	L_HALF_DWORD_MASK			0xffff0000

#define RX_FIFO_DMA_DEFAULT		0x00104
//#define RX_FIFO_DMA_DEFAULT		0x800104

#define ISR_DISABLE_ALL		(uint32_t)(0xFFFFFFFF)

#define MIPI_CHANNEL_INSTANCE_OFFSET    ((uint32_t)(CSLHS1_BASE - CSLHS0_BASE))

/* Set value for MIPI RX Controller */
#define MIPI_RX_REG(ch, addr) \
    (*((volatile uint32_t *)(CSLHS0_BASE + (ch)*MIPI_CHANNEL_INSTANCE_OFFSET + (addr))))

/* Set value for MIPI RX Controller */
#define WRITE_MIPI_RX_REG(ch, addr, val) \
	(MIPI_RX_REG(ch, addr) = (uint32_t)val)

/* Macro to align number x in multiple of 16 */
#define GET_MULTIPLES_16(x) (((x - 1) | 15) + 1)

/* Private typedefs ----------------------------------------------------------*/
typedef struct mipi2axi_ins_info
{
	mipi2axi_callback callback;
	void *user_data;
} mipi2axi_ins_info_t;
/* Private variables ---------------------------------------------------------*/
static mipi2axi_ins_info_t instances[MIPI2AXI_MAX_INS] =
{
	{
		.callback = NULL,
		.user_data = NULL
	},
	{
		.callback = NULL,
		.user_data = NULL
	},
	{
		.callback = NULL,
		.user_data = NULL
	},
	{
		.callback = NULL,
		.user_data = NULL
	},
	{
		.callback = NULL,
		.user_data = NULL
	},
	{
		.callback = NULL,
		.user_data = NULL
	},
};

/* Private functions ---------------------------------------------------------*/
static int convert_in_halfword(uint16_t width, mipi2axi_data_type_t data_type,
								mipi2axi_stream_mode_t mode)
{
	int num_of_halfword;
	int bits_per_pixel;
	switch(data_type)
	{
		case MIPI2AXI_RAW10:
		{
			bits_per_pixel = TEN_BITS_PER_PIXEL;
			break;
		}
		case MIPI2AXI_RAW12:
		{
			bits_per_pixel = TWELVE_BITS_PER_PIXEL;
			break;
		}
		case MIPI2AXI_RAW14:
		{
			bits_per_pixel = FOURTEEN_BITS_PER_PIXEL;
			break;
		}
		case MIPI2AXI_RAW8:
		default:
		{
			bits_per_pixel = EIGHT_BITS_PER_PIXEL;
			break;
		}
	}
	if(mode == MIPI2AXI_SNAPSHOT_MODE)
	{
		/* 10bits per pixel would be packed in 16 bytes */
		num_of_halfword = (width * bits_per_pixel) / 8;
		/* If it is not exactly multiple of 16 then do +1 to it */
		if((num_of_halfword % 16) != 0)
			num_of_halfword = (num_of_halfword / 16) + 1;
		else
			num_of_halfword = (num_of_halfword / 16);
	}
	else
	{
		/* num_of_halfword should be num_of_pixels*2/16
		 * because 1 pixel occupies 2 bytes */
		num_of_halfword = (width * 2) / 16;
	}
	return num_of_halfword;
}
static void mipi_rx_dphy_init(uint8_t ins_idx)
{
    /* PHY_SHUTDOWNZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_SHUTDOWNZ_REG, 1);
    /* PHY_RESETZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_RESET_REG, 1);
    /* CSI2_RESETN */
    WRITE_MIPI_RX_REG(ins_idx, CSI2_RESETN_REG, 1);
	/* PHY_TEST_CTRL1 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL1_REG, 0);
	/* PHY_TEST_CTRL0 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0);
    /* PHY_SHUTDOWNZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_SHUTDOWNZ_REG, 0);
    /* PHY_RESETZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_RESET_REG, 0);
	/* PHY_TEST_CTRL0 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0x1);
	/* PHY_TEST_CTRL0 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0x2);
	/* PHY_TEST_CTRL1 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL1_REG, 0x00010044);
	/* PHY_TEST_CTRL0 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0x0);
    /* PHY_TEST_CTRL1 */
    WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL1_REG, 0x00000044);
    /* PHY_TEST_CTRL1 */
    WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL1_REG, 0x00000016);
    /* PHY_TEST_CTRL0 */
    WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0x2);
	/* PHY_TEST_CTRL0 */
	WRITE_MIPI_RX_REG(ins_idx, PHY_TEST_CTRL0_REG, 0x0);
    /* Data lanes = 4 */
    WRITE_MIPI_RX_REG(ins_idx, N_LANES_REG, 0x03);
    /* PHY_SHUTDOWNZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_SHUTDOWNZ_REG, 1);
    /* PHY_RESETZ */
    WRITE_MIPI_RX_REG(ins_idx, PHY_RESET_REG, 1);
}

static inline void mipi2axi_irq_hdl(mipi2axi_channel_t channel)
{
	uint32_t int_detail, error_detail;

	/* Check data interrupt register */
	int_detail = READ_MIPI2AXI_REG(channel, MIPI2AXI_SIGNAL_INTR);
	/* Check error interrupt register */
	error_detail = READ_MIPI2AXI_REG(channel, MIPI2AXI_ERROR_INTR);
	if(error_detail)
	{
	    unsigned int tick = xTaskGetTickCountFromISR();
		printf(
				"\r\nRX[%d] Error: 0x%x [ts: %d] \r\n",
				(unsigned int)channel,
				(unsigned int)error_detail,
				tick);

	}
	if(instances[channel].callback != 0)
	{
		instances[channel].callback(
									channel,
									int_detail,
									error_detail,
									instances[channel].user_data);
	}

	/* Clear interrupt flags */
	WRITE_MIPI2AXI_REG(channel, SIGNAL_INT1_CLR_REG, int_detail);
	WRITE_MIPI2AXI_REG(channel, ERROR_INT2_CLR_REG, error_detail);
}

static void mipi2axi_irq_hdl_ins0(void)
{
    mipi2axi_irq_hdl(0);
}

static void mipi2axi_irq_hdl_ins1(void)
{
    mipi2axi_irq_hdl(1);
}

static void mipi2axi_irq_hdl_ins2(void)
{
    mipi2axi_irq_hdl(2);
}

static void mipi2axi_irq_hdl_ins3(void)
{
    mipi2axi_irq_hdl(3);
}

static void mipi2axi_irq_hdl_ins4(void)
{
    mipi2axi_irq_hdl(4);
}

static void mipi2axi_irq_hdl_ins5(void)
{
    mipi2axi_irq_hdl(5);
}

static void rx_controller_intr(mipi2axi_channel_t channel)
{
	uint8_t has_error = 0;
	unsigned int intval = MIPI_RX_REG(channel, 0xC);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_MAIN: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0xe0);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_PHY_FATAL: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0xf0);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_PKT_FATAL: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0x100);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_FRAME_FATAL: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0x110);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_PHY: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0x120);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_PKT: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0x130);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_LINE: 0x%08x \r\n", (int)channel, intval);

	intval = MIPI_RX_REG(channel, 0x140);
	has_error |= intval;
	if(intval)
		printf("\r\nMIPI RX[%d] error INT_ST_IPI: 0x%08x \r\n", (int)channel, intval);

#if 0
	if(has_error)
	{
		hal_mipi2axi_stop(channel);
		/*while(1);*/
	}
#endif
}

static void rx_controller_intr_ins0(void)
{
    rx_controller_intr(0);
}

static void rx_controller_intr_ins1(void)
{
    rx_controller_intr(1);
}

static void rx_controller_intr_ins2(void)
{
    rx_controller_intr(2);
}

static void rx_controller_intr_ins3(void)
{
    rx_controller_intr(3);
}

static void rx_controller_intr_ins4(void)
{
    rx_controller_intr(4);
}

static void rx_controller_intr_ins5(void)
{
    rx_controller_intr(5);
}

/* Exported functions --------------------------------------------------------*/
void hal_mipi2axi_init(mipi2axi_channel_t channel)
{
	/* Check parameters */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));

	/* Initialize DPHY of MIPI RX Controller */
	mipi_rx_dphy_init(channel);

	/* Configure default DMA */
	WRITE_MIPI2AXI_REG(channel, RX_FIFO_DMA_REG, RX_FIFO_DMA_DEFAULT);

	/* Disable all interrupt */
	WRITE_MIPI2AXI_REG(channel, INT1_MASK_REG, ISR_DISABLE_ALL);
	WRITE_MIPI2AXI_REG(channel, INT2_MASK_REG, ISR_DISABLE_ALL);
}

void hal_mipi2axi_set_stream_property(mipi2axi_channel_t channel,
										mipi2axi_property_t *opt)
{
	/* Check parameters ------------------------------------------------------*/
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_VC_CHANNEL(opt->vc_option));
	assert_param(IS_VC_CHANNEL(opt->img_type.vc_will_captured));
	assert_param(IS_VC_PACK_MODE(opt->stream_mode));
	assert_param(IS_RAW_DATA_TYPE(opt->img_type.data_type));

	uint32_t tmp = 0, reg_val = 0, offset = 0;
	uint32_t vc_option = opt->vc_option;
	uint32_t current_vc = 0;

	/*
	 * Configure data packing mode (stream mode)
	 */
	tmp = opt->stream_mode;
	reg_val = READ_MIPI2AXI_REG(channel, DPM_REG);
	if(opt->vc_option & MIPI2AXI_VC0_CH)
	{
		reg_val &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT0));
		reg_val |= (tmp << SHIFT_TO_BIT0);
	}
	if(opt->vc_option & MIPI2AXI_VC1_CH)
	{
		reg_val &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT8));
		reg_val |= (tmp << SHIFT_TO_BIT8);
	}
	if(opt->vc_option & MIPI2AXI_VC2_CH)
	{
		reg_val &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT16));
		reg_val |= (tmp << SHIFT_TO_BIT16);
	}
	if(opt->vc_option & MIPI2AXI_VC3_CH)
	{
			reg_val &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT24));
			reg_val |= (tmp << SHIFT_TO_BIT24);
	}
	WRITE_MIPI2AXI_REG(channel, DPM_REG, reg_val);

	/*
	 * Configures data type
	 */

	uint8_t vc_will_capture = 0;
	switch(opt->img_type.vc_will_captured)
	{
		case MIPI2AXI_VC0_CH:
			vc_will_capture = VC_0;
			break;
		case MIPI2AXI_VC1_CH:
			vc_will_capture = VC_1;
			break;
		case MIPI2AXI_VC2_CH:
			vc_will_capture = VC_2;
			break;
		case MIPI2AXI_VC3_CH:
			vc_will_capture = VC_3;
			break;
		default:
			vc_will_capture = VC_0;
			break;
	}
	tmp = (uint32_t)((vc_will_capture << SHIFT_TO_BIT8) +
					(opt->img_type.data_type << SHIFT_TO_BIT0));
	if(opt->vc_option & MIPI2AXI_VC0_CH)
	{
		reg_val = READ_MIPI2AXI_REG(channel, DT_REG_VC01);
		reg_val &= (uint32_t)L_HALF_DWORD_MASK; /* Clear 16 bits */
		reg_val |= (tmp << SHIFT_TO_BIT0);
		WRITE_MIPI2AXI_REG(channel, DT_REG_VC01, reg_val);
	}
	if(opt->vc_option & MIPI2AXI_VC1_CH)
	{
		reg_val = READ_MIPI2AXI_REG(channel, DT_REG_VC01);
		reg_val &= (uint32_t)H_HALF_DWORD_MASK; /* Clear 16 bits */
		reg_val |= (tmp << SHIFT_TO_BIT16);
		WRITE_MIPI2AXI_REG(channel, DT_REG_VC01, reg_val);
	}
	if(opt->vc_option & MIPI2AXI_VC2_CH)
	{
		reg_val = READ_MIPI2AXI_REG(channel, DT_REG_VC23);
		reg_val &= (uint32_t)L_HALF_DWORD_MASK; /* Clear 16 bits */
		reg_val |= (tmp << SHIFT_TO_BIT0);
		WRITE_MIPI2AXI_REG(channel, DT_REG_VC23, reg_val);
	}
	if(opt->vc_option & MIPI2AXI_VC3_CH)
	{
		reg_val = READ_MIPI2AXI_REG(channel, DT_REG_VC23);
		reg_val &= (uint32_t)H_HALF_DWORD_MASK; /* Clear 16 bits */
		reg_val |= (tmp << SHIFT_TO_BIT16);
		WRITE_MIPI2AXI_REG(channel, DT_REG_VC23, reg_val);
	}

	/*
	 * Calculate image pixel
	 */
	tmp = convert_in_halfword(opt->img_size.width, opt->img_type.data_type,
								opt->stream_mode);
	tmp |= (uint32_t)((opt->img_size.height << SHIFT_TO_BIT16) & L_HALF_DWORD_MASK);

	current_vc = 0;
	vc_option = opt->vc_option;
	while(vc_option)
	{
		if(vc_option & 1)
		{
			/*
			 * Configure destination address
			 */
			offset = current_vc * (VC1_DEST_REG - VC0_DEST_REG);
			WRITE_MIPI2AXI_REG(channel, VC0_DEST_REG + offset, opt->addr.dst_addr);

			/*
			 * Configure pixel (image size)
			 */
			offset = current_vc * (VC1_SIZE_REG - VC0_SIZE_REG);
			WRITE_MIPI2AXI_REG(channel, VC0_SIZE_REG + offset, tmp);
		}

		vc_option >>= 1;
		current_vc++;
	}

	/*
	 * Configures source address
	 */
	WRITE_MIPI2AXI_REG(channel, ALU_SRC_REG, opt->addr.alu_src_addr);

	/*
	 * Configure ALU size
	 */
	WRITE_MIPI2AXI_REG(channel, ALU_SIZE_REG, tmp);
}




void hal_mipi2axi_bypass_cfg(mipi2axi_channel_t channel,
							mipi2axi_vc_ch_t vc_option,
							mipi2axi_bypass_mode_t mode)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_VC_CHANNEL(vc_option));
	assert_param(IS_MIPI2AXI_BYPASS(mode));

	uint8_t current_vc = 0;
	uint32_t tmp = 0;

	/* Choose virtual channel (just one channel will bypass) */
	if(vc_option & MIPI2AXI_VC0_CH)
		current_vc = 0;
	else if(vc_option & MIPI2AXI_VC1_CH)
		current_vc = 1;
	else if(vc_option & MIPI2AXI_VC2_CH)
		current_vc = 2;
	else if(vc_option & MIPI2AXI_VC3_CH)
		current_vc = 3;

	tmp = (current_vc << SHIFT_TO_BIT4) + mode;
	WRITE_MIPI2AXI_REG(channel, VC_BYPASS_REG, tmp);
}



void hal_mipi2axi_alu_cfg(mipi2axi_channel_t channel,
							mipi2axi_alu_op_t alu_op,
							flag_status_t alu_ena)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_MIPI2AXI_STATUS(alu_ena));
	assert_param(IS_ALU_OP(alu_op));

	uint32_t tmp = 0;
	tmp = (alu_op << SHIFT_TO_BIT16) + (alu_ena << SHIFT_TO_BIT0);
	WRITE_MIPI2AXI_REG(channel, ALU_OP_REG, tmp);
}


void hal_mipi2axi_frame_control_cfg(mipi2axi_channel_t channel,
									mipi2axi_vc_ch_t vc_option,
									uint8_t frame_num,
									flag_status_t frame_ena)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_VC_CHANNEL(vc_option));
	assert_param(IS_MIPI2AXI_STATUS(frame_ena));

	uint16_t reg_offset = 0;
	uint32_t tmp = 0;
	uint32_t current_vc = 0;

	/* Scan all virtual channel selected */
	while(vc_option)
	{
		if(vc_option & 1)
		{
			/* Select register offset of virtual channel */
			reg_offset = current_vc * 0x10;
			tmp = (uint32_t)((frame_num << SHIFT_TO_BIT8) + (frame_ena << SHIFT_TO_BIT0));
			WRITE_MIPI2AXI_REG(channel, VC0_FRAME_SEL + reg_offset, tmp);
		}
		vc_option >>= 1;
		current_vc++;
	}
}

void hal_mipi2axi_wrap_cfg(mipi2axi_channel_t channel,
							mipi2axi_vc_ch_t vc_option,
							uint16_t wrap_num,
							flag_status_t wrap_en)

{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_VC_CHANNEL(vc_option));
	assert_param(IS_MIPI2AXI_STATUS(wrap_en));

	uint16_t reg_offset = 0;
	uint32_t tmp = 0;
	uint32_t current_vc = 0;

	/* Scan all virtual channel selected */
	while(vc_option)
	{
		if(vc_option & 1)
		{
			/* Select register offset of virtual channel */
			reg_offset = current_vc * (VC1_WRAP_REG - VC0_WRAP_REG);
			tmp = (uint32_t)((wrap_num << SHIFT_TO_BIT16) + (wrap_en << SHIFT_TO_BIT0));
			WRITE_MIPI2AXI_REG(channel, VC0_WRAP_REG + reg_offset, tmp);
		}
		vc_option >>= 1;
		current_vc++;
	}
}

void hal_mipi2axi_vc_mapping(mipi2axi_channel_t channel, vc_mapping_t vc_remap)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_VC_REMAP(vc_remap));

	WRITE_MIPI2AXI_REG(channel, VC_REMAP_REG, vc_remap);
}

void hal_mipi2axi_set_tx_base_addr(mipi2axi_channel_t channel, uint32_t addr)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));

	WRITE_MIPI2AXI_REG(channel, MIPI_TX_BASE_REG, addr);
}

void hal_mipi2axi_dma_cfg(mipi2axi_channel_t channel,
							uint8_t write_abort_counter,
							uint8_t write_fifo_threshold)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));

	uint32_t tmp = 0;

	tmp = (uint32_t)((write_abort_counter << SHIFT_TO_BIT16)
					+ (write_fifo_threshold << SHIFT_TO_BIT8));

	WRITE_MIPI2AXI_REG(channel, RX_FIFO_DMA_REG, tmp);
}

void hal_mipi2axi_stop(mipi2axi_channel_t channel)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	WRITE_MIPI2AXI_REG(channel, READY_REG, DISABLE);
}

void hal_mipi2axi_start(mipi2axi_channel_t channel)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	WRITE_MIPI2AXI_REG(channel, READY_REG, ENABLE);
}

void hal_mipi2axi_irq_mask(mipi2axi_channel_t channel,
							mipi2axi_irq_mask_num_t irq_num,
							uint32_t irq_detail,
							uint8_t enable)
{
	/* Check parameter */
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	assert_param(IS_IRQ_MASK_NUM(irq_num));
	assert_param(IS_MIPI2AXI_STATUS(enable));

	uint32_t interrupt_reg_val = READ_MIPI2AXI_REG(channel, irq_num);

	if(enable)
	{
		/* Enable interrupt by clear bit to 0 */
		irq_detail = ISR_DISABLE_ALL & (~(uint32_t)irq_detail);
		interrupt_reg_val &= irq_detail;
	}
	else
	{
		/* Disable interrupt by set bit to 1 */
		interrupt_reg_val |= irq_detail;
	}
	WRITE_MIPI2AXI_REG(channel, irq_num, interrupt_reg_val);
}

void hal_mipi2axi_irq_enable(mipi2axi_channel_t channel,
							mipi2axi_callback callback,
							void *user_data)
{
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	switch(channel)
	{
		case 0:
		{
			vic_register_irq(MIPI0_RX_IRQn, mipi2axi_irq_hdl_ins0);
		    vic_register_irq(CSIH0_IRQn, rx_controller_intr_ins0);
			break;
		}
		case 1:
		{
			vic_register_irq(MIPI1_RX_IRQn, mipi2axi_irq_hdl_ins1);
            vic_register_irq(CSIH1_IRQn, rx_controller_intr_ins1);
			break;
		}
		case 2:
		{
			vic_register_irq(MIPI2_RX_IRQn, mipi2axi_irq_hdl_ins2);
            vic_register_irq(CSIH2_IRQn, rx_controller_intr_ins2);
			break;
		}
		case 3:
		{
			vic_register_irq(MIPI3_RX_IRQn, mipi2axi_irq_hdl_ins3);
            vic_register_irq(CSIH3_IRQn, rx_controller_intr_ins3);
			break;
		}
		case 4:
		{
			vic_register_irq(MIPI4_RX_IRQn, mipi2axi_irq_hdl_ins4);
            vic_register_irq(CSIH4_IRQn, rx_controller_intr_ins4);
			break;
		}
		case 5:
		{
			vic_register_irq(MIPI5_RX_IRQn, mipi2axi_irq_hdl_ins5);
            vic_register_irq(CSIH5_IRQn, rx_controller_intr_ins5);
			break;
		}
		default:
		{
			break;
		}
	}

	WRITE_MIPI_RX_REG(channel, 0xe4, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0xf4, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0x104, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0x114, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0x124, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0x134, 0xFFFFFFFF);

	WRITE_MIPI_RX_REG(channel,0x144, 0xFFFFFFFF);

	instances[channel].callback = callback;
	instances[channel].user_data = user_data;
}

void hal_mipi2axi_irq_disable(mipi2axi_channel_t channel)
{
	assert_param(IS_MIPI2AXI_CHANNEL(channel));
	vic_unregister_irq(MIPI0_RX_IRQn + channel);
	instances[channel].callback = NULL;
	instances[channel].user_data = NULL;
}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.***** END OF FILE *********/
