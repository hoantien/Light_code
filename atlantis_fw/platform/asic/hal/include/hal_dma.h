/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_dma.h
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-30-2016
 * @brief	This file contains definitions of the DMA controller
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief hal_dma_channel_t
 *
 * DMA channels
 */
typedef enum hal_dma_channel
{
	HAL_DMA_CH1 = 0,
	HAL_DMA_CH2,
	HAL_DMA_CH3,
	HAL_DMA_CH4,
	HAL_DMA_CH5,
	HAL_DMA_CH6,
	HAL_DMA_CH7,
	HAL_DMA_CH8,
	HAL_DMA_CH_MAX
} hal_dma_channel_t;

/**
 * @brief hal_dma_status_t
 *
 * DMA status
 */
typedef enum hal_dma_status
{
	HAL_DMA_OK = 0,
	HAL_DMA_CH_BUSY,
	HAL_DMA_NULL_PTR,
	HAL_DMA_CH_INVALID,
	HAL_DMA_RESULT_YES,
	HAL_DMA_RESULT_NO
} hal_dma_status_t;

/**
 * @brief dma_axi_master_t
 *
 * DMA AXI master
 */
typedef enum dma_axi_master
{
	AXI_MASTER_1,
	AXI_MASTER_2
} dma_axi_master_t;

/**
 * @brief dma_transfer_type_t
 *
 * DMA transfer type
 */
typedef enum dma_transfer_type
{
	MEM_TO_MEM = 0x000,
	MEM_TO_PER = 0x001,
	PER_TO_MEM = 0x002,
	PER_TO_PER = 0x003,
} dma_transfer_type_t;

/**
 * @brief dma_data_width_t
 *
 * DMA data width
 */
typedef enum dma_data_width
{
	DATA_WIDTH_8	= 0x000,
	DATA_WIDTH_16	= 0x001,
	DATA_WIDTH_32	= 0x002,
	DATA_WIDTH_64	= 0x003,
	DATA_WIDTH_128	= 0x004,
	DATA_WIDTH_256	= 0x005,
	DATA_WIDTH_512	= 0x006
} dma_data_width_t;

/**
 * @brief dma_burst_size_t
 *
 * DMA burst width
 */
typedef enum dma_burst_size
{
	BURST_SIZE_1	= 0x000,
	BURST_SIZE_4	= 0x001,
	BURST_SIZE_8	= 0x002,
	BURST_SIZE_16	= 0x003,
	BURST_SIZE_32	= 0x004,
	BURST_SIZE_64	= 0x005,
	BURST_SIZE_128	= 0x006,
	BURST_SIZE_256	= 0x007,
	BURST_SIZE_512	= 0x008,
	BURST_SIZE_1024	= 0x009
} dma_burst_size_t;

/**
 * @brief dma_multiblock_type_t
 *
 * DMA multiblock type
 */
typedef enum dma_multiblock_type
{
	CONTIGUOUS	= 0,
	RELOAD		= 1,
	SHADOW		= 2,
	LINKED_LIST	= 3
} dma_multiblock_type_t;

/**
 * @brief dma_short_lli_t
 *
 * DMA lli type
 */
typedef struct dma_short_lli
{
	uint32_t src_addr;
	uint32_t dst_addr;
	uint32_t length;
} dma_short_lli_t;

/**
 * @brief dma_init_type_t
 *
 * DMA initialize structure
 */
typedef struct dma_init_type
{
	hal_dma_channel_t	chid;
	uint32_t			src_addr;
	uint32_t			dst_addr;
	uint32_t			length;
	dma_burst_size_t	src_bsize;
	dma_burst_size_t	dst_bsize;
	dma_data_width_t	src_wdata;
	dma_data_width_t	dst_wdata;
	dma_transfer_type_t	trans_type;
	dma_axi_master_t	src_axi;
	dma_axi_master_t	dst_axi;
	uint8_t				periph_type;
	dma_short_lli_t		*lli;
	uint32_t			lli_cnt;
	void (*clb_func)(void);
} dma_init_type_t;

/* Exported function  --------------------------------------------------------*/
/*
 * @brief hal_dma_init
 * Initialize DMA module
 * @param init	point to dma_init_type_t structure
 * @return		reference to hal_dma_status_t
 */
hal_dma_status_t hal_dma_init(dma_init_type_t *init);

/*
 * @brief hal_dma_deinit
 * De-initialize DMA module
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @return		reference to hal_dma_status_t
 */
hal_dma_status_t hal_dma_deinit(hal_dma_channel_t chid);

/*
 * @brief hal_dma_enable
 * Enbale the DMA channel module
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @return		reference to hal_dma_status_t
 */
hal_dma_status_t hal_dma_channel_enable(hal_dma_channel_t chid);

/*
 * @brief hal_dma_disable
 * Disable the DMA channel module
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @return		reference to hal_dma_status_t
 */
hal_dma_status_t hal_dma_channel_disable(hal_dma_channel_t chid);

/*
 * @brief hal_dma_set_channel_priority
 * Set priority of the DMA channel module
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @param priority	priority of the DMA channel.
 * @return		reference to hal_dma_status_t
 */
hal_dma_status_t hal_dma_set_channel_priority(hal_dma_channel_t chid,
															uint8_t priority);

/*
 * @brief hal_dma_get_channel_priority
 * Get priority of the DMA channel module
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @return		the priority of the DMA channel
 */
uint8_t hal_dma_get_channel_priority(hal_dma_channel_t chid);

/*
 * @brief hal_dma_is_transfer_completed
 * Check the DMA channel is transfer completed
 * @param chid	this parameter is referred to hal_dma_channel_t
 * @return		refer to hal_dma_status_t
 */
hal_dma_status_t hal_dma_is_transfer_completed(hal_dma_channel_t chid);

/*
 * @brief hal_dma_reset
 * Reset the DMA controller
 * @return none
 */
void hal_dma_reset(void);

/*
 * @brief hal_dma_enable
 * Enable global interrupt of the DMA controller.
 * @param irq_priority the priority of the DMA interrupt line.
 * @return none
 */
void hal_dma_enable_global_interrupt(uint8_t irq_priority);

#ifdef __cplusplus
}
#endif
#endif	/* __DMA_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
