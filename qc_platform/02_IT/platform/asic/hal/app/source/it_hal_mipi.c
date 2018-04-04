/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_mipi2axi.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    July 04, 2016
 * @brief   This file contains expand of the HAL MIPI to AXI
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "it_hal_mipi.h"
/* Private define ------------------------------------------------------------*/
#define ATCM				0x03300000
#define BTCM				0x03400000
#define DDR_ADDR 			0x40000000
#define ISR_DISABLE_ALL		0x0fffffff

#define PRINT_AXI_2_MIPI	1
#define PRINT_MIPI_2_AXI	2

#define ENABLE				1
#define DISABLE				0


/* Exported variables --------------------------------------------------------*/
LOCAL tx_callback_index mipi_tx_callback_index;
LOCAL rx_callback_index mipi_rx_callback_index;
IMPORT cam_module_t cam_module_list_t[CAM_CH_MAX_NUM];
/* Static functions ----------------------------------------------------------*/
#ifdef DEBUG
LOCAL void print_mipi_rx_debug(uint8_t channel);
LOCAL void print_mipi_tx_debug(uint8_t channel);
LOCAL void dum_mem_mipi_rx(uint8_t channel);
LOCAL void dum_mem_mipi_tx(uint8_t channel);
#endif

LOCAL void it_mipi_tx_callback(axi2mipi_channel_t , axi2mipi_isr_flag_t,void *);
LOCAL void it_mipi_rx_callback(uint8_t,uint32_t,uint32_t,void *);
LOCAL void reset_isr_index(mipi_type);

LOCAL int mipi_tx_vc1(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc2(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc3(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc12(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc13(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc23(uint16_t ,flag_status_t ,uint32_t );
LOCAL int mipi_tx_vc123(uint16_t ,flag_status_t ,uint32_t );

/**
 * @brief to verify MIPI TX on VC1 FIFO_B
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details - To verify pattern of raw file after snapshot in VC1 FIFO_B
 */
LOCAL int it_hal_mipi_001(char** argv, int argc);
/**
 * @brief to verify interrupt APIs of MIPI TX
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details - To verify frame start and frame and interrupt when snapshot in VC1
 */
LOCAL int it_hal_mipi_002(char** argv, int argc);
/**
 * @brief to verify APIs of MIPI RX with interrupt
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details - To verify frame start and frame and interrupt when stream on
 * 				camera A1 in snapshot mode
 */
LOCAL int it_hal_mipi_003(char** argv, int argc);

LOCAL int it_hal_mipi_004(char** argv, int argc);

LOCAL void set_data_for_tx(void);

/**
 *  @Brief: GPIO module testing map
 */
LOCAL it_map_t it_mipi_tests_table[] = {
		{"MIPI_001", it_hal_mipi_001},
		{"MIPI_002", it_hal_mipi_002},
		{"MIPI_003", it_hal_mipi_003},
		{"MIPI_004", it_hal_mipi_004},
		{"",  NULL}
};

/* Exported functions --------------------------------------------------------*/
int it_hal_mipi_handler(char **argv, int argc)
{
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_mipi_tests_table);
	if (-1 != index)
	{
		return it_mipi_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

LOCAL void set_data_for_tx(void)
{
	/* TCMB: 128KB */
	unsigned int patt = 0x55665566;
	unsigned int addr = BTCM;
	unsigned int i = 0;
	for(i = 0; i < (64*1024); i += 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
	patt = 0x33443344;
	for(i = (64*1024); i < (128*1024); i+= 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
	/* TCMA: 64KB */
	patt = 0x4655434B;
	addr = ATCM;
	for(i = 0; i < (64*1024); i += 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
	patt = 0x12345678;
	for(i = (64*1024); i < (128*1024); i+= 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
}

LOCAL void it_mipi_tx_callback(axi2mipi_channel_t channel,\
		axi2mipi_isr_flag_t interrupt_source, void *user_data)
{
	if ((interrupt_source&FE_INT_FIFO_A)||(interrupt_source&FE_INT_FIFO_B)||
			(interrupt_source&FE_INT_FIFO_C)||(interrupt_source&FE_INT_FIFO_D))
		mipi_tx_callback_index.frame_end++;
	if ((interrupt_source&FS_INT_FIFO_A)||(interrupt_source&FS_INT_FIFO_B)||
			(interrupt_source&FS_INT_FIFO_C)||(interrupt_source&FS_INT_FIFO_D))
		mipi_tx_callback_index.frame_start++;
	if ((interrupt_source&DMA_LINE_INT_FIFO_A)||(interrupt_source&DMA_LINE_INT_FIFO_B)||
			(interrupt_source&DMA_LINE_INT_FIFO_C)||(interrupt_source&DMA_LINE_INT_FIFO_D))
		mipi_tx_callback_index.dma_line++;
	if ((interrupt_source&DMA_FRAME_INT_FIFO_A)||(interrupt_source&DMA_FRAME_INT_FIFO_B)||
			(interrupt_source&DMA_FRAME_INT_FIFO_C)||(interrupt_source&DMA_FRAME_INT_FIFO_D))
		mipi_tx_callback_index.dma_frame++;
	if ((interrupt_source&EMPTY_INT_FIFO_A)||(interrupt_source&EMPTY_INT_FIFO_B)||
			(interrupt_source&EMPTY_INT_FIFO_C)||(interrupt_source&EMPTY_INT_FIFO_D))
		mipi_tx_callback_index.fifo_empty++;
	if ((interrupt_source&FULL_INT_FIFO_A)||(interrupt_source&FULL_INT_FIFO_B)||
			(interrupt_source&FULL_INT_FIFO_C)||(interrupt_source&FULL_INT_FIFO_D))
		mipi_tx_callback_index.fifo_full++;
	if ((interrupt_source&AXI_ERR_INT_FIFO_A)||(interrupt_source&AXI_ERR_INT_FIFO_B)||
			(interrupt_source&AXI_ERR_INT_FIFO_C)||(interrupt_source&AXI_ERR_INT_FIFO_D))
		mipi_tx_callback_index.axi_error++;
}

LOCAL void it_mipi_rx_callback(uint8_t iidx, uint32_t signal_irq_detail,
		uint32_t error_irq_detail, void *user_data)
{
	if (signal_irq_detail&0xF000000)	mipi_rx_callback_index.frame_start++;
	if (signal_irq_detail&0xF0000000)	mipi_rx_callback_index.frame_end++;
	if (signal_irq_detail&0x100000)	mipi_rx_callback_index.dma_finish++;
	if (signal_irq_detail&0xF000)	mipi_rx_callback_index.line_end++;
	if (signal_irq_detail&0xF00)	mipi_rx_callback_index.line_start++;
	if (signal_irq_detail&0xF)
	{
		mipi_rx_callback_index.write_dma_finish++;
		mipi_rx_callback_index.write_dma_finish_WRAP++;
	}
}

LOCAL void make_dummy_for_tx(uint32_t mem_addr)
{
	/* TCMB: 128KB */
	unsigned int patt = 0x55665566;
	unsigned int addr = mem_addr;
	unsigned int i = 0;
	for(i = 0; i < (64*1024); i += 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
	patt = 0x33443344;
	for(i = (64*1024); i < (128*1024); i+= 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
	patt = 0x99889988;
	for(i = (128*1024); i < (192*1024); i += 4)
	{
		volatile unsigned int dst = addr + i;
		memcpy((uint8_t *)(dst), &patt, 4);
	}
}

#ifdef DEBUG
LOCAL void print_reg_val(uint8_t rx_or_tx, uint32_t addr)
{
	uint32_t val = 0;
	if(rx_or_tx == PRINT_MIPI_2_AXI)
	{
		val = READ_MIPI2AXI_REG(addr);
		log_printf("MIPI 2 AXI REG 0x%X\t\tVAL: 0x%X\n\r", (int)addr, (int)val);
	}
	else
	{
		val = READ_AXI2MIPI_REG(addr);
		log_printf("AXI 2 MIPI REG 0x%X\t\tVAL: 0x%X\n\r", (int)addr, (int)val);
	}

}

LOCAL void print_mipi_rx_debug(uint8_t channel)
{
	uint32_t offset = channel * 0x1000;

	log_printf(" \n\rMIPI TO AXI CHANNEL[%d] SETTING\n\r", channel);
	print_reg_val(PRINT_MIPI_2_AXI, 0x04 + offset);
	print_reg_val(PRINT_MIPI_2_AXI, 0x2C + offset);
	print_reg_val(PRINT_MIPI_2_AXI, 0x54 + offset);
	print_reg_val(PRINT_MIPI_2_AXI, 0x10 + offset);
	//log_printf("Address\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x104 + offset);
	//log_printf("Image pixel size\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x108 + offset);
	//log_printf("Frame VC\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x100 + offset);
	//log_printf("Wrap\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x10C + offset);
	//log_printf("Bypass\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x60 + offset);
	//log_printf("Data type\n\r");
	print_reg_val(PRINT_MIPI_2_AXI, 0x18 + offset);
}

LOCAL void print_mipi_tx_debug(uint8_t channel)
{
	uint32_t offset = channel * 0x1000;
	log_printf("\n\rAXI TO MIPI CH [%d] REG SETTING\n\r", channel);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x0);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x4);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x8);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0xC);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x10);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x34);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x38);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x3C);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x48);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x4C);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x50);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x54);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x80);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x100);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x104);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x110);
	print_reg_val(PRINT_AXI_2_MIPI, offset + 0x114);
	log_printf("\n\r");
}

LOCAL void dum_mem_mipi_rx(uint8_t channel)
{
	uint32_t start_addr = 0, end_addr = 0;
	uint32_t offset = channel * 0x1000;
	uint32_t i, val;

	start_addr = 0x04;
	end_addr = 0x13C;

	log_printf(" \n\r");
	log_printf("MIPI 2 AXI Channel: %02d\n\r", (int)channel);
	for(i = start_addr; i <= end_addr; i += 4)
	{
		if(i == 0x08 || i == 0x0C || i == 0x3C || i == 0x4c || i == 0x50
				|| i == 0x5c || (i >= 0x64 && i <= 0xFC))
		{
			continue;
		}
		val = READ_MIPI2AXI_REG(i+offset);
		log_printf("MIPI 2 AXI REG: 0x%X\t\tVAL: 0x%X\n\r",
				(int)i, (int)val);
	}
}

LOCAL void dum_mem_mipi_tx(uint8_t channel)
{
	uint32_t start_addr = 0, end_addr = 0;
	uint32_t offset = channel * 0x1000;
	uint32_t i, val;

	start_addr = 0x00;
	end_addr = 0x150;

	log_printf(" \n\r");
	log_printf("AXI 2 MIPI Channel: %02d\n\r", (int)channel);
	for(i = start_addr; i <= end_addr; i += 4)
	{
		if((i >= 0x58 && i < 0x80) || (i > 0x88 && i < 0x100) ||
			(i > 0x104 && i < 0x110)|| (i > 0x114 && i < 0x120) ||
			(i > 0x124 && i < 0x130) || (i > 0x104 && i < 0x110) || (i > 0x134 && i < 0x10))
		{
			continue;
		}
		val = READ_AXI2MIPI_REG(i+offset);
		log_printf("AXI 2 MIPI REG: 0x%X\t\tVAL: 0x%X\n\r",
				(int)i, (int)val);
	}
}
#endif
LOCAL void reset_isr_index(mipi_type tp)
{
	if (mipi_tx == tp)
		memset(&mipi_tx_callback_index,0,sizeof(tx_callback_index));
	else if(mipi_rx ==tp)
		memset(&mipi_rx_callback_index,0,sizeof(rx_callback_index));
	else
	{
		memset(&mipi_tx_callback_index,0,sizeof(tx_callback_index));
		memset(&mipi_rx_callback_index,0,sizeof(rx_callback_index));
	}
}

LOCAL int mipi_tx_vc1(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= AXI2MIPI_FIFO_B;
	axi2mipi_property_t opt ;
	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}


	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC1_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();

	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);
	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}

	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");

	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc2(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= AXI2MIPI_FIFO_B;
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}

	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC2_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);
	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);
	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}

	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");

	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc3(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= AXI2MIPI_FIFO_B;
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}
	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC3_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);
	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);
	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}

	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");

	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc12(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= (AXI2MIPI_FIFO_B|AXI2MIPI_FIFO_C);
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}
	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC1_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack		= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	if (DDR_ADDR == src_addr) opt.src_addr = DDR_ADDR +(64*1024);
	else opt.src_addr = BTCM;
	opt.fifo_x				= AXI2MIPI_FIFO_C;
	opt.vc_for_ff			= VC2_FOR_FIFO;
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);
	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}

	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");

	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc13(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= (AXI2MIPI_FIFO_B|AXI2MIPI_FIFO_C);
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}

	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC1_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	if (DDR_ADDR == src_addr) opt.src_addr = DDR_ADDR +(64*1024);
	else opt.src_addr = BTCM;
	opt.fifo_x				= AXI2MIPI_FIFO_C;
	opt.vc_for_ff			= VC3_FOR_FIFO;
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);
	qc_assert_reset();

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}
	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");
	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc23(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= (AXI2MIPI_FIFO_B|AXI2MIPI_FIFO_C);
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}
	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC2_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	if (DDR_ADDR == src_addr) opt.src_addr = DDR_ADDR +(64*1024);
	else opt.src_addr = BTCM;
	opt.fifo_x				= AXI2MIPI_FIFO_C;
	opt.vc_for_ff			= VC3_FOR_FIFO;
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}
	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");
	qc_assert(TRUE);
	qc_report();
	return 0;
}

LOCAL int mipi_tx_vc123(uint16_t wrap_num,
		flag_status_t wrap_en,uint32_t src_addr)
{
	uint16_t width = 256, height = 100;
	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		=
			(AXI2MIPI_FIFO_B|AXI2MIPI_FIFO_C|AXI2MIPI_FIFO_D);
	axi2mipi_property_t opt ;

	if (DDR_ADDR == src_addr)
	{
		opt.src_addr			= DDR_ADDR;
		make_dummy_for_tx(DDR_ADDR);
	}
	else
	{
		opt.src_addr			= ATCM;
		set_data_for_tx();
	}

	opt.fifo_x				= AXI2MIPI_FIFO_B;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.vc_for_ff			= VC1_FOR_FIFO;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack	= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;

	qc_assert_reset();
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	if (DDR_ADDR == src_addr) opt.src_addr = DDR_ADDR +(64*1024);
	else opt.src_addr = ATCM + (64*1024);
	opt.fifo_x				= AXI2MIPI_FIFO_C;
	opt.vc_for_ff			= VC2_FOR_FIFO;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	if (DDR_ADDR == src_addr) opt.src_addr = DDR_ADDR +(128*1024);
	else opt.src_addr = ATCM + (128*1024);
	opt.fifo_x				= AXI2MIPI_FIFO_D;
	opt.vc_for_ff			= VC3_FOR_FIFO;
	opt.wrap.wrap_num		= wrap_num;
	opt.wrap.wrap_en		= wrap_en;
	hal_axi2mipi_set_stream_property(channel_tx,&opt);

	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 5 second\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(5000);
	}
	log_printf("Please check pattern in RAW file in /data/misc/camera \n\r");
	qc_assert(TRUE);
	qc_report();
	return 0;
}

/* Testing functions **********************************************************/
/**
 * argv[0] = WRAP_DISABLE or WRAP_ENABLE
 * argv[1] = Wrap number (if argv[0] is WRAP_DISABLE, wrap number is any
 * argv[2] = VC1,VC2,VC3,VC12,VC13,VC23,VC123
 * argv[3] = DDR or any
 * */
LOCAL int it_hal_mipi_001(char **argv, int argc)
{
	uint16_t wrap_num;
	uint32_t src_addr;
	flag_status_t wrap_en;
	if (4 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}
	if (0 == strcmp(argv[0], "WRAP_DISABLE"))
	{
		wrap_en = DISABLE;
		wrap_num = 100;
	}
	else if (0 == strcmp(argv[0], "WRAP_ENABLE"))
	{
		wrap_en = ENABLE;
		wrap_num = (flag_status_t)strtol(argv[1],NULL,10);
	}
	else {
		log_printf("Error: Invalid argument 1.\r\n");
		return -1;
	}

	if (0 == strcmp(argv[3], "DDR"))
	{
		src_addr=DDR_ADDR;
		hal_ddr_init();
	}
	else src_addr=0;

	if (0 == strcmp(argv[2], "VC1")) mipi_tx_vc1(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC2")) mipi_tx_vc2(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC3")) mipi_tx_vc3(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC12")) mipi_tx_vc12(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC13")) mipi_tx_vc13(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC23")) mipi_tx_vc23(wrap_num,wrap_en,src_addr);
	else if (0 == strcmp(argv[2], "VC123")) mipi_tx_vc123(wrap_num,wrap_en,src_addr);
	else
	{
		log_printf("Invalid test case for MIPI TX testing");
		return -1;
	}
	return 0;
}
/**
 * argv[0] = 0,1,2 or 3 correspond with FIFO A,B,C and D
 * argv[1] = 0,1,2 or 3 correspond with VC 0,1,2 and 3
 * argv[2] = 0,1,2,...,27 correspond with interrupt mask bit
 * */
LOCAL int it_hal_mipi_002(char **argv, int argc)
{
	uint16_t width = 1920, height = 1080;

	if (3 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}
	else
	{
		if((3<strtol(argv[0],NULL,10))||(0>strtol(argv[0],NULL,10)))
		{
			log_printf("Error: Invalid argument 0.\r\n");
			return -1;
		}
		if((3<strtol(argv[1],NULL,10))||(0>strtol(argv[1],NULL,10)))
		{
			log_printf("Error: Invalid argument 1.\r\n");
			return -1;
		}
		if((28<strtol(argv[2],NULL,10))||(0>strtol(argv[2],NULL,10)))
		{
			log_printf("Error: Invalid argument 2.\r\n");
			return -1;
		}
	}

	hal_ddr_init();
	make_dummy_for_tx(DDR_ADDR);

	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x;
	axi2mipi_isr_flag_t mipi_tx_isr;
	axi2mipi_property_t opt;

	fifo_x = (axi2mipi_ff_t)(1<<strtol(argv[0],NULL,10));
	opt.vc_for_ff = (axi2mipi_vc_for_ff_t)strtol(argv[1],NULL,10);

	opt.fifo_x				= fifo_x;
	opt.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;
	opt.src_addr			= DDR_ADDR;
	opt.img_size.width		= width;
	opt.img_size.height		= height;
	opt.img_type.tx_dt		= AXI2MIPI_LIGHT_RAW;
	opt.img_type.rx_dt		= AXI2MIPI_RAW10;
	opt.img_type.pack		= AXI2MIPI_PACKED_DATA;
	opt.img_type.ff_bypass	= NONE_BYPASS;
	opt.wrap.wrap_en		= DISABLE;

	if (0==strcmp(argv[2],"NA"))
		mipi_tx_isr=0;
	else
		mipi_tx_isr = (axi2mipi_isr_flag_t)(1<<strtol(argv[2],NULL,10));
	reset_isr_index(mipi_tx);
	set_data_for_tx();
	qc_assert_reset();

	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx,&opt);
	hal_axi2mipi_irq_mask(channel_tx, mipi_tx_isr,ENABLE);
	hal_axi2mipi_irq_enable(channel_tx,it_mipi_tx_callback,NULL);
	hal_axi2mipi_programming_valid(channel_tx,fifo_x,1);
	hal_axi2mipi_set_frame_mode(channel_tx,fifo_x,NONE_CONTINUOUS);

	for (int i = 0; i < 5 ; i++)
	{
		log_printf("Capture after 500 ms\n\r");
		hal_axi2mipi_start(channel_tx,fifo_x);
		_delay_ms(500);
	}
#ifdef DEBUG
	dum_mem_mipi_tx(channel_tx);
	print_mipi_tx_debug(channel_tx);
#endif

	if(mipi_tx_isr == 0)
	{
		qc_assert(mipi_tx_callback_index.frame_end==0);
		qc_assert(mipi_tx_callback_index.frame_start==0);
		qc_assert(mipi_tx_callback_index.dma_line==0);
		qc_assert(mipi_tx_callback_index.dma_frame==0);
		qc_assert(mipi_tx_callback_index.fifo_empty==0);
		qc_assert(mipi_tx_callback_index.fifo_full==0);
		qc_assert(mipi_tx_callback_index.axi_error==0);
	}
	else if (mipi_tx_isr & 0x0f)
		qc_assert(mipi_tx_callback_index.frame_end==5);
	else if (mipi_tx_isr & 0xf0)
		qc_assert(mipi_tx_callback_index.frame_start==5);
	else if (mipi_tx_isr & 0xf00)
		qc_assert(mipi_tx_callback_index.dma_line==154);
	else if (mipi_tx_isr & 0xf000)
		qc_assert(mipi_tx_callback_index.dma_frame==5);
	else if (mipi_tx_isr & 0xf0000)
		qc_assert(mipi_tx_callback_index.fifo_empty==5);
	else if (mipi_tx_isr & 0xf00000)
		qc_assert(mipi_tx_callback_index.fifo_full==5);
	else if (mipi_tx_isr & 0xf000000)
		qc_assert(mipi_tx_callback_index.axi_error==5);
	qc_report();
	return 0;
}
/**
 * argv[0] = 0,1,2,3 or 4 correspond with channel_rx MIPI2AXI0 to MIPI2AXI5
 * argv[1] = 0,1,2 or 3 correspond with VC 0,1,2 and 3
 * argv[2] = 0,1,2,...,31 correspond with interrupt mask bit
 * argv[3] correspond with number of syncin pulse
 * */
LOCAL int it_hal_mipi_003(char **argv, int argc)
{
	uint8_t numpulse = 0;
	uint16_t width = 1920, height = 1080;

	if (4 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}
	else
	{
		if((5<strtol(argv[0],NULL,10))||(0>strtol(argv[0],NULL,10)))
		{
			log_printf("Error: Invalid argument 0.\r\n");
			return -1;
		}
		if((3<strtol(argv[1],NULL,10))||(0>strtol(argv[1],NULL,10)))
		{
			log_printf("Error: Invalid argument 1.\r\n");
			return -1;
		}
		if((31<strtol(argv[2],NULL,10))||(0>strtol(argv[2],NULL,10)))
		{
			log_printf("Error: Invalid argument 2.\r\n");
			return -1;
		}
		numpulse = (uint8_t)strtol(argv[3],NULL,10);
		if(0>numpulse)
		{
			log_printf("Error: Invalid argument 3.\r\n");
			return -1;
		}
	}

	/* Open camera A1 */
	if(-1==cam_open_t(&cam_module_list_t[CAM_CH_A1]))
	{
		log_printf("Failed to open camera\n\r");
		return -1;
	}

	hal_ddr_init();
	mipi2axi_channel_t channel_rx = strtol(argv[0],NULL,10);
	mipi2axi_property_t opt_rx;
	opt_rx.vc_option = (1<<strtol(argv[1],NULL,10));
	opt_rx.addr.dst_addr = DDR_ADDR;
	opt_rx.img_type.vc_will_captured = (1<<strtol(argv[1],NULL,10));
	opt_rx.img_type.data_type = MIPI2AXI_RAW10;
	opt_rx.img_size.height = height;
	opt_rx.img_size.width = width;
	opt_rx.stream_mode = MIPI2AXI_SNAPSHOT_MODE;
	hal_mipi2axi_init(channel_rx);
	hal_mipi2axi_set_stream_property(channel_rx, &opt_rx);
	hal_mipi2axi_bypass_cfg(channel_rx, opt_rx.vc_option, NONE_BYPASS);
	hal_mipi2axi_wrap_cfg(channel_rx, opt_rx.vc_option, height, DISABLE);

	/* Configure interrupt */
	mipi2axi_callback callback_hdl = it_mipi_rx_callback;
	uint32_t irq_detail;
	reset_isr_index(mipi_rx);
	if (0==strcmp(argv[1],"NA"))
		irq_detail = 0;
	else
		irq_detail = (1<<(uint32_t)(strtol(argv[2],NULL,10)));

	hal_mipi2axi_irq_mask(channel_rx,
			MIPI2AXI_SIGNAL_MASK_INTR, irq_detail, ENABLE);
	hal_mipi2axi_irq_enable(channel_rx, callback_hdl, NULL);
	hal_mipi2axi_start(channel_rx);

	/* Configure syncio */
	init_syncio(SG_INF_DIS,numpulse);
	cam_syncio_stream_on(SG_CHANNEL_MAX);
	_delay_ms(1000);

	qc_assert_reset();
	if(irq_detail == 0)
	{
		qc_assert(mipi_rx_callback_index.frame_end==0);
		qc_assert(mipi_rx_callback_index.frame_start==0);
		qc_assert(mipi_rx_callback_index.dma_finish==0);
		qc_assert(mipi_rx_callback_index.line_end==0);
		qc_assert(mipi_rx_callback_index.line_start==0);
		qc_assert(mipi_rx_callback_index.write_dma_finish==0);
		qc_assert(mipi_rx_callback_index.write_dma_finish_WRAP==0);
	}
	else if (irq_detail & 0xF0000000)
		qc_assert(mipi_rx_callback_index.frame_end==numpulse);
	else if (irq_detail & 0xF000000)
		qc_assert(mipi_rx_callback_index.frame_start==numpulse);
	else if (irq_detail & 0x100000)
		qc_assert(mipi_rx_callback_index.dma_finish==numpulse);
	else if (irq_detail & 0xF000)
		qc_assert(mipi_rx_callback_index.line_end==numpulse);
	else if (irq_detail & 0xF00)
		qc_assert(mipi_rx_callback_index.line_start==numpulse);
	else if (irq_detail & 0xF)
		qc_assert(mipi_rx_callback_index.write_dma_finish==numpulse);

#ifdef DEBUG
	log_printf("Interrupt count = %d\n\r",mipi_rx_callback_index.frame_start );
	print_mipi_rx_debug(channel_rx);
	log_printf("\n\r");
	dum_mem_mipi_rx(channel_rx);
#endif

	qc_report();
	return 0;
}

LOCAL int it_hal_mipi_004(char **argv, int argc)
{
	uint16_t width = 1920, height = 1080;
	int size;
	mipi2axi_channel_t channel_rx	= MIPI2AXI0 ;
	mipi2axi_vc_ch_t vc_option		= MIPI2AXI_VC0_CH;
	mipi2axi_property_t opt_rx;

	axi2mipi_channel_t	channel_tx	= AXI2MIPI_BRIDGE_0;
	axi2mipi_ff_t		fifo_x		= AXI2MIPI_FIFO_B;
	axi2mipi_property_t opt_tx;
	axi2mipi_isr_flag_t mipi_tx_isr;

	axi2mipi_cont_frame_t	tx_frame_mode;

	if (2 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}
	else
	{
		/*Choose stream mode*/
		if(0==strcmp(argv[0],"S"))
		{
			opt_rx.stream_mode = MIPI2AXI_SNAPSHOT_MODE;
			opt_tx.stream_mode = AXI2MIPI_SNAPSHOT_MODE;
			opt_tx.vc_for_ff  = VC1_FOR_FIFO;
			tx_frame_mode = NONE_CONTINUOUS;
			mipi_tx_isr	= FE_INT_FIFO_B;
		}
		else if(0==strcmp(argv[0],"P"))
		{
			opt_rx.stream_mode = MIPI2AXI_PREVIEW_MODE;
			opt_tx.stream_mode = AXI2MIPI_PREVIEW_MODE;
			opt_tx.vc_for_ff  = VC0_FOR_FIFO;
			tx_frame_mode = FRAME_CONTINUOUS;
			mipi_tx_isr	= FE_INT_FIFO_A;
		}
		else
		{
			log_printf("Error: Invalid argument 0.\r\n");
			return -1;
		}
		/* Choose memory: DDR or ATCM or BTCM */
		if(0==strcmp(argv[1],"DDR"))
		{
			hal_ddr_init();
			opt_rx.addr.dst_addr	= DDR_ADDR;
			opt_tx.src_addr			= DDR_ADDR;
		}
		else if(0==strcmp(argv[1],"ATCM"))
		{
			opt_rx.addr.dst_addr	= ATCM;
			opt_tx.src_addr			= ATCM;
		}
		else if(0==strcmp(argv[1],"BTCM"))
		{
			opt_rx.addr.dst_addr	= BTCM;
			opt_tx.src_addr			= BTCM;
		}
		else
		{
			log_printf("Error: Invalid argument 1.\r\n");
			return -1;
		}
	}

	/* Open camera A1 */
	if(-1==cam_open_t(&cam_module_list_t[CAM_CH_A1]))
	{
		log_printf("Failed to open camera\n\r");
		return -1;
	}
	/* Configure MIPI RX */
	opt_rx.vc_option 				= vc_option;
	opt_rx.addr.alu_src_addr 		= 0; // don't care
	opt_rx.img_type.vc_will_captured= vc_option;
	opt_rx.img_type.data_type 		= MIPI2AXI_RAW10;
	opt_rx.img_size.height 			= height;
	opt_rx.img_size.width 			= width;
	hal_mipi2axi_init(channel_rx);
	hal_mipi2axi_set_stream_property(channel_rx, &opt_rx);
	hal_mipi2axi_wrap_cfg(channel_rx, vc_option, height, ENABLE);
	hal_mipi2axi_start(channel_rx);

	/* Configure MIPI TX */
	reset_isr_index(mipi_tx);
	opt_tx.fifo_x 					= fifo_x;
	opt_tx.img_size.height 			= height;
	opt_tx.img_size.width 			= width;
	opt_tx.img_type.tx_dt			= AXI2MIPI_LIGHT_RAW;
	opt_tx.img_type.rx_dt			= AXI2MIPI_RAW10;
	opt_tx.img_type.pack			= AXI2MIPI_PACKED_DATA;
	opt_tx.img_type.ff_bypass 		= NONE_BYPASS;
	opt_tx.wrap.wrap_en				= ENABLE;
	opt_tx.wrap.wrap_num			= height;
	hal_axi2mipi_init(channel_tx);
	hal_axi2mipi_set_stream_property(channel_tx, &opt_tx);
	hal_axi2mipi_irq_mask(channel_tx, mipi_tx_isr,ENABLE);
	hal_axi2mipi_irq_enable(channel_tx,it_mipi_tx_callback,NULL);
	hal_axi2mipi_programming_valid(channel_tx, fifo_x, ENABLE);
	hal_axi2mipi_set_frame_mode(channel_tx, fifo_x, tx_frame_mode);

	init_syncio(SG_INF_EN, 2);
	cam_syncio_stream_on(SG_CHANNEL_MAX);
	if(opt_tx.stream_mode == AXI2MIPI_SNAPSHOT_MODE)
	{
		for (int i = 0; i < 5 ; i++)
		{
			log_printf("Capture after 1s\n\r");
			hal_axi2mipi_start(channel_tx,opt_tx.fifo_x);
			_delay_ms(1000);
		}
	}
	else while(console_getstring(&size));

//	log_printf("%d \n\r",mipi_tx_callback_index.frame_end);
	qc_assert(mipi_tx_callback_index.frame_end==5);
	qc_report();
	return 0;
}
