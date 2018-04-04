/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    usecase_hires.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-07-2016
 * @brief   This file contains functions for snapshot camera
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "light_system.h"
#include "std_type.h"
#include "assert.h"
#include "ar1335.h"
#include "hal_axi2mipi.h"
#include "hal_mipi2axi.h"
#include "usecase_hires.h"
#include "hal_pwm.h"
#include "lcc_system.h"
#include "cortex_r4.h"
#include "usecase.h"
#include "log.h"
#include "hal_cache.h"
#include "light_header.h"

/* Private define ------------------------------------------------------------*/
#define POPULATE_LIGHT_HEADER_TIMEOUT	20000
#define TX_TRANSFER_TASK_TIMEOUT		20000

#define BITS_PER_PIXEL_RAW10	10
#define BITS_PER_PIXEL_RAW12	12
#define BITS_PER_PIXEL_RAW14	14
#define BITS_PER_PIXEL_RAW8		8

#define CAM_PORT_MASK				0x0F
#define CAM_MASK_ALL				(uint32_t)(0x1FFFF)
#define DDR_START_ADDR				DDR_BASE

#define	THUMBNAIL_VCID_MASK			0xf0
#define IMAGE_VCID_MASK				0x0f
/* This define indicate the memory area used to store image */

#define MIPI_TX_ERR_MASK			(uint32_t)(0x0f << 24)
#define SLOGF_ID					SLOG_ID_LCC_CMD_BASE_0000

/* This indicate the number of line that TX should transfer beside raw data */
/* Usually, this depends on number of bytes of light header + 32 bytes */
#define GET_MULTIPLES_32(x) (((x - 1) | 31) + 1)
#define TX_PADDING_LINE					2
#define LIGHT_HEADER_MAX_SIZE			GET_MULTIPLES_32(1024*1024)

#define CAM_A1							0xA1
#define CAM_A2							0xA2
#define CAM_A3							0xA3
#define CAM_A4							0xA4
#define CAM_A5							0xA5
#define CAM_B1							0xB1
#define CAM_B2							0xB2
#define CAM_B3							0xB3
#define CAM_B4							0xB4
#define CAM_B5							0xB5
#define CAM_C1							0xC1
#define CAM_C2							0xC2
#define CAM_C3							0xC3
#define CAM_C4							0xC4
#define CAM_C5							0xC5
#define CAM_C6							0xC6

#define SNAPSHOT_AXI2MIPI_FIFO					AXI2MIPI_FIFO_C
#define SNAPSHOT_AXI2MIPI_DMA_SRC_REG			DMA_SRC_C_REG
#define SNAPSHOT_AXI2MIPI_FRAME_CTRL_REG		FRAME_CTRL_C_REG
#define SNAPSHOT_AXI2MIPI_FE_INT_FIFO			FE_INT_FIFO_C
#define SNAPSHOT_AXI2MIPI_FS_INT_FIFO			FS_INT_FIFO_C

#define THUMBNAIL_AXI2MIPI_FIFO					AXI2MIPI_FIFO_B
#define THUMBNAIL_AXI2MIPI_DMA_SRC_REG			DMA_SRC_B_REG
#define THUMBNAIL_AXI2MIPI_FRAME_CTRL_REG		FRAME_CTRL_B_REG
#define THUMBNAIL_AXI2MIPI_FE_INT_FIFO			FE_INT_FIFO_B
#define THUMBNAIL_AXI2MIPI_FS_INT_FIFO			FS_INT_FIFO_B
extern SemaphoreHandle_t snapshot_sem;
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	CONFIG_FOR_THUMBNAIL = 0x00,
	CONFIG_FOR_SNAPSHOT = 0x01
} select_vcid ;

/* Private variables ---------------------------------------------------------*/
/* This variable used to store a image size of a picture */
uint32_t img_bytes;
volatile uint8_t tx_transfer_done;
volatile uint8_t lh_populate_done;
mipi_frame_t mipi_rx[MAX_NUM_CAPT_CAMERA];

/* This variable used to store number of camera in m_bitmask */
__IO static uint8_t cam_total;

/* This array used to store the CAM index in memory area
 * EX: CAM 2, CAM3, CAM 5, numerical order in memory is 1, 2, 3
 */
uint8_t cam_mem_idx[MAX_NUM_CAPT_CAMERA];

/* This variable used to store index of camera thumbnail in cam_mem_idx array */
static uint8_t cam_thumb_mem_idx;
static uint8_t has_cam_thumb;
/* This variable used to indicate memory index */
static uint8_t thumbnail_mem_idx;

/* These variables indicate that if any CAM on board available,
 * the corresponding bits will set to 1 */
__IO static uint8_t cam_asic_port_mask;

/* Variables used to check frame will snapshot */
__IO static uint8_t num_of_frame = 0;

/* Variable used to indicate the instance of MIPI TX (BRIDGE 0 or 1)*/
uint8_t axi2mipi_tx_bridge;
capt_priv_data_t priv_data;
/* Variables used to check MIPI TX ISR for thumbnail camera */
mipi_tx_frame_t thumbnail_tx;
/* Variables used to check MIPI TX ISR for snapshot cameras */
mipi_tx_frame_t snapshot_tx;

uint8_t light_header_cnt = 0;

/* Variables used to check MIPI RX ISR for all cameras */

static uint8_t mipi_tx_fr_cnt, mipi_tx_done_flag;
uint8_t synio_count;

static uint8_t mem_idx_rx[MAX_NUM_CAPT_CAMERA];
static uint8_t tx_ddr_mem_idx;

static uint8_t set_next_addr_flag[MAX_NUM_CAPT_CAMERA];

void (*callback_hdl)(void);

#define GET_RX_DDR_ADDR(cam_idx, mem_idx) \
	(uint32_t)(DDR_START_ADDR + (mem_idx) * (cam_total * img_bytes) + (cam_idx) * img_bytes)

#define GET_TX_DDR_ADDR(mem_idx)	\
	(uint32_t)(DDR_START_ADDR + ((mem_idx) * cam_total * img_bytes))


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t min_frame_done(mipi_frame_t *p, uint8_t size)
{
	uint16_t min = 0;
	uint8_t i = 0;

	min = 0x100;

	for(i = 0; i < size; i++)
	{
		if(p[i].active)
		{
			min = (min < p[i].fdone_cnt) ? min : p[i].fdone_cnt;
		}
	}

	if(min == 0x100)
		min = 0;

	return min;
}


/* @brief prepare_capture_info
 * This function is used to prepare the information for capture data
 * @param mbitmask: capture module bitmask
 * @param hdr_info_addr: the location that hdr_info should be written
 * @param light_hdr_addr: the location that light_header should be written
 * @return: 1 - for success and 0 for fail
 */
int prepare_capture_info(unsigned int mbitmask, uint8_t hdr_info_idx)
{
	unsigned int light_hdr_addr = 0;
	uint64_t header_offset = 0, record_size = 0;
	const char* magic = "LELR";
	uint32_t padding = 0;
	if(hdr_info_idx != 0)
	{
		padding = MAX_PROTOBUF_HEADER_SIZE;
	}
	SLOGF(SLOG_INFO, "HDR INFO %d and PADDING %d", hdr_info_idx, padding);
	uint32_t start_address = (GET_RX_DDR_ADDR(0, hdr_info_idx) + (hdr_info_idx * padding));
	volatile lightheader_info_t *hdr_info_addr =
		(volatile lightheader_info_t *) (start_address);
	SLOGF(SLOG_INFO, "HDR INFO ADDRESS: 0x%08x", (unsigned int)start_address);
	header_offset = cam_total * img_bytes + sizeof(lightheader_info_t);
	record_size = cam_total * img_bytes + sizeof(lightheader_info_t)
												+ MAX_PROTOBUF_HEADER_SIZE;
	hdr_info_addr->header_offset = header_offset;
	hdr_info_addr->record_size = record_size;
	hdr_info_addr->magic_id[0] = magic[0];
	hdr_info_addr->magic_id[1] = magic[1];
	hdr_info_addr->magic_id[2] = magic[2];
	hdr_info_addr->magic_id[3] = magic[3];
	hdr_info_addr->header_size = 0;
	hdr_info_addr->reversed = 0;

    light_hdr_addr = (uint32_t)(start_address +
        cam_total*img_bytes + sizeof(lightheader_info_t));
    uint32_t light_hdr_len = 0;
    Ltpb__LightHeader *light_hdr = alloc_init_light_header();
    lightheader_error_t ret = get_light_header(mbitmask, light_hdr);
    if(ret == LIGHT_HEADER_ERROR_NONE)
	{
		uint8_t *out = (uint8_t *)(light_hdr_addr);
		light_hdr_len = ltpb__light_header__pack(light_hdr, out);
		SLOGF(SLOG_INFO, "LightHeader size %d at 0x%08x", (int)light_hdr_len, light_hdr_addr);
	}
	else
	{
		SLOGF(SLOG_ERROR, "LightHeader populating failed");
	}
    hdr_info_addr->header_size = light_hdr_len;
	free_mem_light_header(light_hdr);
	/* Flush cache of headers */
	flush_cache(start_address, sizeof(lightheader_info_t));
	flush_cache(light_hdr_addr, light_hdr_len);
	return 1;
}

unsigned int get_real_tx_ddr_address(uint8_t memidx)
{
	if(memidx == 0)
		return GET_TX_DDR_ADDR(memidx);
	else
	{
		return GET_TX_DDR_ADDR(memidx) + LIGHT_HEADER_MAX_SIZE;
	}

}

unsigned int get_real_rx_ddr_address(uint8_t camidx, uint8_t regionidx)
{
	if(regionidx == 0)
	{
		unsigned int address = GET_RX_DDR_ADDR(camidx, regionidx) + sizeof(lightheader_info_t);
		return address;
	}
	else
	{
		unsigned int address = GET_RX_DDR_ADDR(camidx, regionidx) + LIGHT_HEADER_MAX_SIZE + sizeof(lightheader_info_t);
		// printf("\r\nNEXT RX DDR address: %x\r\n", address);
		return address;
	}
}


void task_populate_header(void* param)
{
	uint64_t time_out = POPULATE_LIGHT_HEADER_TIMEOUT;
	uint8_t all_rx_end = 1;
	uint8_t last_cnt = 1;
	uint8_t light_header_cnt = 0;
	while(1)
	{
		EventBits_t ev = xEventGroupWaitBits(priv_data.hires_evhdl,
			EVENT_RX_FRAME_START | EVENT_SNAPSHOT_COMPLETE,
				pdTRUE, pdFALSE, time_out);
		if(ev)
		{
			if(ev & EVENT_RX_FRAME_START)
			{
				if(all_rx_end)
				{
					prepare_capture_info(priv_data.mbitmask, light_header_cnt);
					/* Set the event that Light Header is ready */
					if(priv_data.hires_evhdl)
						xEventGroupSetBits(priv_data.hires_evhdl, EVENT_LIGHT_HEADER_READY);
					priv_data.lh_ev_cnt ++;
					light_header_cnt ++;
					if(light_header_cnt >= MAX_AREA_MEM)
						light_header_cnt = 0;
					all_rx_end = 0;
					if(priv_data.lh_ev_cnt >= num_of_frame)
					{
						ev |= EVENT_SNAPSHOT_COMPLETE;
					}
				}
				else
				{
					int i = 0;
					for(i = 0; i < MAX_NUM_CAPT_CAMERA; i++)
					{
						if(mipi_rx[i].active)
						{
							if(mipi_rx[i].fs_cnt > last_cnt)
							{
								all_rx_end = 1;
								last_cnt = mipi_rx[i].fs_cnt;
								xEventGroupSetBits(priv_data.hires_evhdl, EVENT_RX_FRAME_START);
								break;
							}
						}
					}
					if(i == MAX_NUM_CAPT_CAMERA)
						SLOGF(SLOG_DEBUG, "Already populate Light Header for this RAW");
				}

			}
			if(ev & EVENT_SNAPSHOT_COMPLETE)
			{
				lh_populate_done = 1;
				SLOGF(SLOG_DEBUG, "Deleting %s", __FUNCTION__);
				vTaskDelete(NULL);
			}
		}
		else
		{
			SLOGF(SLOG_DEBUG, "Timeout waiting for snapshot events");
			SLOGF(SLOG_DEBUG, "Deleting %s", __FUNCTION__);
			lh_populate_done = 1;
			vTaskDelete(NULL);
		}
	}
}
void clear_global_variable(void)
{
	memset(mipi_rx, 0x00, sizeof(mipi_rx));

	/* Clear all global */
	mipi_tx_fr_cnt = 0;
	mipi_tx_done_flag = 0;
	synio_count = 0;
	cam_total = 0;
	cam_asic_port_mask = 0;
	axi2mipi_tx_bridge = 0;
	cam_thumb_mem_idx = 0;
	has_cam_thumb = 0;

	tx_ddr_mem_idx = 0;
	thumbnail_mem_idx = 0;

	thumbnail_tx.fe_flag = 0, thumbnail_tx.fs_flag = 0;
	thumbnail_tx.fe_mask = 0, thumbnail_tx.fs_mask = 0;
	snapshot_tx.fe_flag = 0, snapshot_tx.fs_flag = 0;
	snapshot_tx.fe_mask = 0, snapshot_tx.fs_mask = 0;
	memset(mem_idx_rx, 0x00, sizeof(mem_idx_rx));
	memset(set_next_addr_flag, 0x00, sizeof(set_next_addr_flag));
	memset(cam_mem_idx, 0x00, sizeof(cam_mem_idx));
	if(priv_data.hires_evhdl)
	{
		vEventGroupDelete(priv_data.hires_evhdl);
		priv_data.hires_evhdl = NULL;
	}
	memset(&priv_data, 0x0, sizeof(priv_data));
	priv_data.hires_evhdl = xEventGroupCreate();
	priv_data.lh_ev_cnt = 0;

	tx_transfer_done = 0;
	lh_populate_done = 0;

	light_header_cnt = 0;
}

capt_grp_t get_capture_group(unsigned int mbitmask)
{
	int capt_grp = 0;
	/* mbitmask should have A or/and B camera */
	if(mbitmask & CAPT_GRP_AB)
	{
		if(mbitmask & CAPT_GRP_A)
			capt_grp |=  CAPT_GRP_A;

		if(mbitmask & CAPT_GRP_B)
			capt_grp |=  CAPT_GRP_B;

		if(mbitmask & CAPT_GRP_C)
		{
			/*log_error("Capture 3 group is not supported \r\n");
			return 0;*/
			capt_grp |=  CAPT_GRP_C;
		}
	}
	/* mbitmask should have B or/and C camera */
	else if(mbitmask & CAPT_GRP_BC)
	{
		if(mbitmask & CAPT_GRP_B)
			capt_grp |=  CAPT_GRP_B;

		if(mbitmask & CAPT_GRP_C)
			capt_grp |=  CAPT_GRP_C;

		if(mbitmask & CAPT_GRP_A)
		{
			capt_grp |=  CAPT_GRP_C;
			//log_error("Capture 3 group is not supported \r\n");
			//return 0;
		}
	}
	/* if other case, select group of the first CAM */
	else
	{
		cam_typedef_t *pcam;
		uint8_t cam_idx = 0;
		mbitmask &= light_system->m_filter;
		while(mbitmask)
		{
			/* Check camera bit mask */
			if(mbitmask & 1)
			{
				/* Get camera pointer corresponding with board */
				pcam = idx_to_object(cam_idx);
				if(pcam != NULL)
				{
					capt_grp = pcam->info.grp;
					break;
				}
			}
			/* Increase camera index */
			cam_idx++;
			mbitmask >>= 1;
		}
	}
	return capt_grp;
}

static uint32_t get_image_size(uint16_t width, uint16_t height,
								uint8_t image_type)
{
	if(width == 0 || height == 0 || image_type == 0)
		return 0;

	uint32_t image_size = 0;
	uint8_t bits_per_pixel = 0;

	switch(image_type)
	{
		case AXI2MIPI_RAW10:
		{
			bits_per_pixel = BITS_PER_PIXEL_RAW10;
			break;
		}
		case AXI2MIPI_RAW12:
		{
			bits_per_pixel = BITS_PER_PIXEL_RAW12;
			break;
		}
		case AXI2MIPI_RAW14:
		{
			bits_per_pixel = BITS_PER_PIXEL_RAW14;
			break;
		}
		case AXI2MIPI_RAW8:
		default:
		{
			bits_per_pixel = BITS_PER_PIXEL_RAW8;
			break;
		}
	}

	/* 10bits per pixel would be packed in 16 bytes */
	image_size = (width * bits_per_pixel) / 8;
	/* If it is not exactly multiple of 16 then do +1 to it */
	if((image_size % 16) != 0)
		image_size = (image_size / 16) + 1;
	else
		image_size = (image_size / 16);
	image_size = image_size * 16;
	/* Calculate the image size of one frame */
	image_size = height * image_size;

	if(image_size % 4 != 0)
	{
		image_size += 4;
	}

//	return image_size;
	//FIXME: Hard code for now to test
	return 16 * 1024 * 1024;
}

/*
 * Get values:
 * cam_total: this variable indicate that on the ASIC has how many CAMs
 * cam_asic_port_mask: this variable
 * cam_mem_idx,
 * mipi_rx
 */
static void get_global_values(uint32_t m_bitmask)
{
	uint8_t cam_count = 0;
	uint8_t cam_idx;
	uint8_t cam_port_mipi2axi; /* indicate that port index on ASIC board */
	cam_typedef_t *pcam;

	/* Check global bit mask, if global is 1 will set all camera bits mask */
	cam_idx = 0;
	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		/* Check camera bit mask */
		if(m_bitmask & 1)
		{
			/* Get camera pointer corresponding with board */
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				/* Get CAM port index number on EVB board */
				cam_port_mipi2axi = pcam->info.ch - 1;
				/* Get numerical order of CAM in memory
				 * EX: CAM 2, CAM3, CAM 5, numerical order in memory is 1, 2, 3
				 */
				cam_mem_idx[cam_port_mipi2axi] = cam_count;
				mipi_rx[cam_port_mipi2axi].active = 1;
				/* Get camera port mask on ASIC EVB board
				 * EX: CAM2, CAM3, CAM5, mask is 0x22 = 00010 1100
				 */
				cam_asic_port_mask |= (1 << cam_port_mipi2axi);
				/* Get total number of cameras on board */
				cam_count++;
			}
		}
		/* Increase camera index */
		cam_idx++;
		m_bitmask >>= 1;
	}

	cam_total = cam_count;
}

cam_typedef_t *get_cam_thumbnail(uint32_t m_bitmask)
{
	cam_typedef_t *pcam;
	uint32_t cam_group;
	uint16_t cam_thumbnail_id;
	uint8_t cam_idx = 0;

	cam_group = get_capture_group(m_bitmask);
	switch(cam_group)
	{
		case CAPT_GRP_A:
		case CAPT_GRP_AB:
			cam_thumbnail_id = CAM_A1;
			break;
		case CAPT_GRP_B:
		case CAPT_GRP_BC:
			cam_thumbnail_id = CAM_B4;
			break;
		case CAPT_GRP_C:
			cam_thumbnail_id = CAM_C5;
			break;
		default:
			cam_thumbnail_id = CAM_A1;
			break;
	}

	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		/* Check camera bit mask */
		if(m_bitmask & 1)
		{
			/* Get camera pointer corresponding with board */
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				/* Get index of camera thumbnail into cam_mem_idx array */
				if(cam_thumbnail_id == pcam->info.module)
				{
					return pcam;
				}
			}
		}
		/* Increase camera index */
		cam_idx++;
		m_bitmask >>= 1;
	}

	return NULL;
}

/***
 * |+++++++++++++++++++++++++++|
 * |       HEADER INFO         |
 * |+++++++++++++++++++++++++++|
 * |                           |
 * |       CAM1 RAW            |
 * |       CAM2 RAW            |
 * |       ........            |
 * |                           |
 * |+++++++++++++++++++++++++++|
 * |        LIGHT HEADER       |
 * |+++++++++++++++++++++++++++|
 *
 */
static void mipi_rx_snapshot_isr(uint8_t mipi2axi_idx, uint32_t detail,
										uint32_t error, void *user_data)
{
	uint32_t ddr_address = 0;
	//EventGroupHandle_t evhdl = user_data;
	/*unsigned int tick = xTaskGetTickCountFromISR();*/
	/*if(error)
	 {
	 printf("\r\nRX Error: 0x%X\n\r", (unsigned int)error);
	 }*/

	/* Check frame start */
	if(detail & R1_VC0_FRAME_START)
	{
		mipi_rx[mipi2axi_idx].fs_cnt++;
		mipi_rx[mipi2axi_idx].fs_flag = 1;
#if 0
		/* Generate the Light Header at the first frame start event of any RX */
		if(priv_data.hires_evhdl)
		{
			BaseType_t woken = pdFALSE;
			if(xEventGroupSetBitsFromISR(priv_data.hires_evhdl,
								EVENT_RX_FRAME_START, &woken) != pdFAIL)
				portYIELD_FROM_ISR(woken);
		}
#endif
		if(set_next_addr_flag[mipi2axi_idx] == 0)
		{
			if(mipi_rx[mipi2axi_idx].fdone_cnt < (num_of_frame - 1))
			{
				set_next_addr_flag[mipi2axi_idx] = 1;
				mem_idx_rx[mipi2axi_idx]++;
				ddr_address = get_real_rx_ddr_address(
											cam_mem_idx[mipi2axi_idx],
											mem_idx_rx[mipi2axi_idx]);

				mem_idx_rx[mipi2axi_idx] =
									(mem_idx_rx[mipi2axi_idx] >= MAX_AREA_MEM) ?
									0 : mem_idx_rx[mipi2axi_idx];
				WRITE_MIPI2AXI_REG(mipi2axi_idx, VC0_DEST_REG, ddr_address);
				WRITE_MIPI2AXI_REG(mipi2axi_idx, READY_REG, ENABLE);
			}
		}

	}
	/* Check frame end */
	if(detail & R1_VC0_FRAME_END)
	{
		mipi_rx[mipi2axi_idx].fe_cnt++;
		mipi_rx[mipi2axi_idx].fe_flag = 1;

	}

	/* Check finish Rx */
	if(mipi_rx[mipi2axi_idx].fs_flag & mipi_rx[mipi2axi_idx].fe_flag)
	{
		mipi_rx[mipi2axi_idx].fdone_cnt++;
		mipi_rx[mipi2axi_idx].fe_flag = 0;
		mipi_rx[mipi2axi_idx].fs_flag = 0;

		if(mipi_rx[mipi2axi_idx].fdone_cnt < num_of_frame)
		{
			set_next_addr_flag[mipi2axi_idx] = 0;
		}
		else
		{

		}
	}
}

void mipi_tx_snapshot_isr(axi2mipi_channel_t channel,
								axi2mipi_isr_flag_t irq_detail, void *usrdata)
{
	uint32_t ddr_address;
	uint32_t thumbnail_ddr_addr = 0;
	if((channel == axi2mipi_tx_bridge) && (irq_detail & MIPI_TX_ERR_MASK) == 0)
	{
		/*
		 * Check frame start and frame end for thumbnail
		 */
		if(has_cam_thumb)
		{
			if(irq_detail & thumbnail_tx.fs_mask)
			{
				thumbnail_tx.fs_flag = 1;
			}

			if(irq_detail & thumbnail_tx.fe_mask)
			{
				thumbnail_tx.fe_flag = 1;
			}

			if(thumbnail_tx.fs_flag & thumbnail_tx.fe_flag)
			{
				/* Reset mem_idx if mem_idx >= MAX_AREA_MEM */
				thumbnail_mem_idx++;
				thumbnail_mem_idx =
						(thumbnail_mem_idx >= MAX_AREA_MEM) ?
								0 : thumbnail_mem_idx;

				thumbnail_tx.fe_flag = 0;
				thumbnail_tx.fs_flag = 0;

				thumbnail_ddr_addr = GET_RX_DDR_ADDR(
						cam_mem_idx[cam_thumb_mem_idx],
						thumbnail_mem_idx) + sizeof(lightheader_info_t);
				/*printf("\r\nThumbnail DDR ADDR: 0x08%X\n\r", (int)thumbnail_ddr_addr);*/
				WRITE_AXI2MIPI_REG(
						axi2mipi_tx_bridge,
						THUMBNAIL_AXI2MIPI_DMA_SRC_REG,
						thumbnail_ddr_addr);
				/* Validate programming */
				WRITE_AXI2MIPI_REG(
						axi2mipi_tx_bridge,
						THUMBNAIL_AXI2MIPI_FRAME_CTRL_REG,
						ENABLE);
				// printf("\r\nThumbnail sent\r\n");
			}
		}

		/*
		 * Check frame start and frame end for image interrupt
		 */
		if(irq_detail & snapshot_tx.fs_mask)
		{
			snapshot_tx.fs_flag = 1;
		}

		if(irq_detail & snapshot_tx.fe_mask)
		{
			snapshot_tx.fe_flag = 1;
			BaseType_t woken = pdFALSE;
			if(priv_data.hires_evhdl)
			{
				if(pdFAIL != xEventGroupSetBitsFromISR(priv_data.hires_evhdl,
											EVENT_TX_FRAME_COMPLETE, &woken))
					portYIELD_FROM_ISR(woken);
			}
		}

		if(snapshot_tx.fs_flag & snapshot_tx.fe_flag)
		{
			snapshot_tx.fs_flag = 0;
			snapshot_tx.fe_flag = 0;

			/* Reset mem_idx if mem_idx >= MAX_AREA_MEM */
			tx_ddr_mem_idx++;
			tx_ddr_mem_idx =
					(tx_ddr_mem_idx >= MAX_AREA_MEM) ? 0 : tx_ddr_mem_idx;

			/* Switch next address */
			ddr_address = get_real_tx_ddr_address(tx_ddr_mem_idx);
#ifdef DEBUG_MIPI
			printf("\r\nTX next DDR address: 0x%08x\r\n", (int)ddr_address);
#endif
			/* Stop MIPI TX */
			WRITE_AXI2MIPI_REG(axi2mipi_tx_bridge, BRIDGE_CTRL_REG, DISABLE);
			/* Change destination memory address */
			WRITE_AXI2MIPI_REG(
					axi2mipi_tx_bridge,
					SNAPSHOT_AXI2MIPI_DMA_SRC_REG,
					ddr_address);
			/* Validate programming */
			WRITE_AXI2MIPI_REG(
					axi2mipi_tx_bridge,
					SNAPSHOT_AXI2MIPI_FRAME_CTRL_REG,
					ENABLE);

			/* Increase the number of frame transfer finished */
			mipi_tx_fr_cnt++;
			mipi_tx_done_flag = 1;
			if(mipi_tx_fr_cnt >= num_of_frame)
			{
				/* Clear counter */
				snapshot_tx.fs_flag = 0;
				snapshot_tx.fe_flag = 0;
				tx_ddr_mem_idx = 0;

				/* Stop MIPI TX */
				WRITE_AXI2MIPI_REG(
						axi2mipi_tx_bridge,
						BRIDGE_CTRL_REG,
						DISABLE);
				if(priv_data.hires_evhdl)
				{
					BaseType_t woken = pdFALSE;
					if(pdFAIL != xEventGroupSetBitsFromISR(priv_data.hires_evhdl,
											EVENT_SNAPSHOT_COMPLETE, &woken))
						portYIELD_FROM_ISR(woken);
				}
				/* Start call back function */
				if(callback_hdl != NULL)
				{
					callback_hdl();
				}
			}
		}
	}
}

static void setup_mipi_rx_snapshot(uint32_t m_bitmask,
									uint16_t width, uint16_t height,
									uint8_t image_type, EventGroupHandle_t evhdl)
{
	uint32_t ddr_address = 0;
	uint8_t mem_idx = 0;
	uint8_t cam_idx = 0;
	uint8_t port_idx = 0;
	cam_typedef_t *pcam;

	cam_typedef_t *cam_thumbnail = get_cam_thumbnail(m_bitmask);
	if(cam_thumbnail != NULL)
	{
		has_cam_thumb = 1;
		SLOGF(SLOG_INFO, "CAM THUMBNAIL: %X", cam_thumbnail->info.module);
	}
	else
	{
		has_cam_thumb = 0;
		SLOGF(SLOG_INFO, "DOES NOT HAVE CAM THUMBNAIL");
	}

	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		/* Check camera bit mask */
		if(m_bitmask & 1)
		{
			/* Get camera pointer corresponding with board */
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				/* Get CAM port index number on EVB board */
				port_idx = pcam->info.ch - 1;
				//ddr_address = GET_RX_DDR_ADDR(cam_mem_idx[port_idx], mem_idx) + sizeof(lightheader_info_t);
				ddr_address = get_real_rx_ddr_address(cam_mem_idx[port_idx], mem_idx);

				/* Get index of camera thumbnail into cam_mem_idx array */
				if(cam_thumbnail != NULL)
				{
					if(cam_thumbnail->info.module == pcam->info.module)
						cam_thumb_mem_idx = port_idx;
				}
#ifdef MIPI_DEBUG
				SLOGF(SLOG_INFO, "CAM-%X INSTANCE[%d]",pcam->info.module,  port_idx);
				SLOGF(SLOG_INFO, "DDR ADDR: 0x%08X", ddr_address);
				SLOGF(SLOG_INFO, "IMG RESOLUTION: WIDTH[%d] HEIGHT[%d]", width, height);
#endif
				/* Initialize MIPI RX DPHY */
				hal_mipi2axi_init(port_idx);

				/* Configure stream mode */
				mipi2axi_property_t mipi2axi_property;
				mipi2axi_property.vc_option				= MIPI2AXI_VC0_CH;
				mipi2axi_property.addr.dst_addr			= ddr_address;
				mipi2axi_property.addr.alu_src_addr 	= 0; /* don't care */
				mipi2axi_property.img_type.vc_will_captured = MIPI2AXI_VC0_CH;
				mipi2axi_property.img_type.data_type	= image_type;
				mipi2axi_property.img_size.height		= height;
				mipi2axi_property.img_size.width		= width;
				mipi2axi_property.stream_mode 			= MIPI2AXI_SNAPSHOT_MODE;
				hal_mipi2axi_set_stream_property(port_idx, &mipi2axi_property);
				hal_mipi2axi_frame_control_cfg(port_idx, MIPI2AXI_VC0_CH, 0, DISABLE);
				/* Configure wrap */
				hal_mipi2axi_wrap_cfg(port_idx, MIPI2AXI_VC0_CH, height, DISABLE);

				/* Configure interrupt */
				hal_mipi2axi_irq_mask(
						port_idx,
						MIPI2AXI_SIGNAL_MASK_INTR,
						R1_VC0_FRAME_END | R1_VC0_FRAME_START
								| R1_VC0_LINE_START | R1_VC0_LINE_END,
						ENABLE);
				hal_mipi2axi_irq_enable(port_idx, mipi_rx_snapshot_isr, NULL);

				/* Start MIPI 2 AXI */
				hal_mipi2axi_start(port_idx);
			}
		}
		/* Increase camera index */
		cam_idx++;
		m_bitmask >>= 1;
	}
}

static void setup_mipi_tx_snapshot(uint8_t tx_channel, uint8_t vc_num,
		uint8_t thumb_or_image, uint16_t width, uint16_t height,
		uint8_t rx_img_type, uint8_t tx_img_type, EventGroupHandle_t evhdl)
{
	uint32_t interrupt_irq = 0;
	uint8_t fifo_id;

	/* Configure stream property - arguments bellow will not be change */
	axi2mipi_property_t axi2mipi_property;
	axi2mipi_property.img_type.rx_dt		= rx_img_type;
	axi2mipi_property.img_type.tx_dt		= tx_img_type;
	axi2mipi_property.img_type.ff_bypass	= NONE_BYPASS;
	axi2mipi_property.wrap.wrap_en			= DISABLE;
	axi2mipi_property.wrap.wrap_num			= height;
	axi2mipi_property.stream_mode			= AXI2MIPI_SNAPSHOT_MODE;

	/* Select configuration for send image RAW or Thumb nail RAW */
	switch(thumb_or_image)
	{
		case CONFIG_FOR_THUMBNAIL:
			fifo_id = THUMBNAIL_AXI2MIPI_FIFO;
			axi2mipi_property.fifo_x 			= fifo_id;
			axi2mipi_property.src_addr			= GET_RX_DDR_ADDR(cam_mem_idx[cam_thumb_mem_idx], 0) + sizeof(lightheader_info_t);
			axi2mipi_property.img_size.height	= height;
			axi2mipi_property.img_size.width	= width;
			axi2mipi_property.img_type.pack		= AXI2MIPI_PACKED_DATA;

			interrupt_irq = THUMBNAIL_AXI2MIPI_FE_INT_FIFO |
							THUMBNAIL_AXI2MIPI_FS_INT_FIFO;
			break;

		case CONFIG_FOR_SNAPSHOT:
		default:
			fifo_id = SNAPSHOT_AXI2MIPI_FIFO;
			axi2mipi_property.fifo_x 				= fifo_id;
			axi2mipi_property.src_addr				= get_real_tx_ddr_address(0);
			/* Number of lines in raw file */
			axi2mipi_property.img_size.height		= height * cam_total;
			/* Number of lines for Light Header */
			/* 2 * 4160 * 10 / 8 = 10400 bytes */
			axi2mipi_property.img_size.height		+= TX_PADDING_LINE;

			// 16 * 1024 * 1024 * 8 / (10 * 4160) - 3120 => 3226.38769 - 3120 = 107
			axi2mipi_property.img_size.height		+= 107 * cam_total;

			SLOGF(SLOG_DEBUG, "TX transfer #line = %d", axi2mipi_property.img_size.height);
			axi2mipi_property.img_size.width		= width;
			axi2mipi_property.img_type.pack			= AXI2MIPI_PACKED_DATA;
			interrupt_irq = SNAPSHOT_AXI2MIPI_FE_INT_FIFO |
							SNAPSHOT_AXI2MIPI_FS_INT_FIFO;
			break;
	}

	/* Select VC for FIFO */
	if(vc_num >= VC0_FOR_FIFO && vc_num <= VC3_FOR_FIFO)
	{
		axi2mipi_property.vc_for_ff = vc_num;
	}
	else
	{
		axi2mipi_property.vc_for_ff = VC0_FOR_FIFO;
	}

	/* Select MIPI TX instance */
	switch(tx_channel)
	{
		case 0x00:
			axi2mipi_tx_bridge = AXI2MIPI_BRIDGE_0;
			break;
		case 0x01:
			axi2mipi_tx_bridge = AXI2MIPI_BRIDGE_1;
			break;
		default:
			axi2mipi_tx_bridge = AXI2MIPI_BRIDGE_0;
			break;
	}

	/* Select flag interrupt for MIPI TX */
	if(thumb_or_image == CONFIG_FOR_SNAPSHOT)
	{
		snapshot_tx.fe_mask = SNAPSHOT_AXI2MIPI_FE_INT_FIFO;
		snapshot_tx.fs_mask = SNAPSHOT_AXI2MIPI_FS_INT_FIFO ;
#ifdef MIPI_DEBUG
		SLOGF(SLOG_DEBUG, "IMAGE TX FIFO   : %d", SNAPSHOT_AXI2MIPI_FIFO);
		SLOGF(SLOG_DEBUG, "IMAGE TX VC     : %d", vc_num);
		SLOGF(SLOG_DEBUG, "IMAGE VC MASK   : 0x%08X", (int)(snapshot_tx.fe_mask | snapshot_tx.fs_mask));
#endif
	}
	else
	{
		thumbnail_tx.fe_mask = THUMBNAIL_AXI2MIPI_FE_INT_FIFO;
		thumbnail_tx.fs_mask = THUMBNAIL_AXI2MIPI_FS_INT_FIFO;
#ifdef MIPI_DEBUG
		SLOGF(SLOG_DEBUG, "THUMBNAI TX FIFO: %d", THUMBNAIL_AXI2MIPI_FIFO);
		SLOGF(SLOG_DEBUG, "THUMBNAI TX VC  : %d", vc_num);
		SLOGF(SLOG_DEBUG, "THUMBNAI VC MASK: 0x%08X", (int)(thumbnail_tx.fe_mask | thumbnail_tx.fs_mask));
#endif
	}
	/*SLOGF(SLOG_DEBUG, "MIPI TX ISR FLAG: 0x%08X\n\r", interrupt_irq);*/

	/* Initialize MIPI operation */
	hal_axi2mipi_set_stream_property(axi2mipi_tx_bridge, &axi2mipi_property);
	if(thumb_or_image == CONFIG_FOR_SNAPSHOT)
	{
		/* Initialize MIPI interrupt */
		hal_axi2mipi_irq_mask(axi2mipi_tx_bridge, 0xFFFFFFFF, ENABLE);
		interrupt_irq = interrupt_irq;
		hal_axi2mipi_irq_enable(axi2mipi_tx_bridge, mipi_tx_snapshot_isr, NULL);
	}
	/* Configure valid */
	hal_axi2mipi_programming_valid(axi2mipi_tx_bridge, fifo_id, ENABLE);

	/* Configure frame of FIFOA, B is non continuous */
	hal_axi2mipi_set_frame_mode(axi2mipi_tx_bridge, fifo_id, NONE_CONTINUOUS);
}


/* Exported functions --------------------------------------------------------*/
usecase_hires_cfg_status_t usecase_hires_setup(uint32_t m_bitmask,
											uint8_t image_type, uint8_t frame,
											void (*snapshot_done_hdl)(void),
											TaskHandle_t *lhtask)
{
	/* uint8_t image_type; */
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t bitmask;
	cam_typedef_t *pcam;
	uint8_t vc_id_snapshot = 0, vc_id_thumbnail = 0;
	/* TODO: this should be get from cache */
	uint8_t cam_idx;

	/* Reset all global variables */
	clear_global_variable();

	/*
	 * Check image setting
	 */

	bitmask = m_bitmask & light_system->m_filter;
	cam_idx = 0;
	while(bitmask)
	{
		/* Check camera bit mask */
		if(bitmask & 1)
		{
			/*SLOGF(SLOG_DEBUG, "MBitmask: 0x%x", (unsigned int)m_bitmask);*/
			/* Get camera pointer corresponding with board */
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				SLOGF(SLOG_INFO, "READ INFO FROM CAM-%X", pcam->info.module);
				/* Read image width */
				width = pcam->image->x_size;
				if(width == 0)
				{
					width = img_sensor_read_reg(
							pcam->image,
							X_OUTPUT_SIZE_CAM_REG,
							TWO_BYTES);
				}
				/* Read image height */
				height = pcam->image->y_size;
				if(height == 0)
				{
					height = img_sensor_read_reg(
							pcam->image,
							Y_OUTPUT_SIZE_CAM_REG,
							TWO_BYTES);
				}
				/* Read thumbnail VC */
				vc_id_thumbnail = (pcam->settings->stream[1] & THUMBNAIL_VCID_MASK) >> 4;
				/* Read image VC */
				vc_id_snapshot = pcam->settings->stream[1] & IMAGE_VCID_MASK;

				/* Read CSI (AXI2MIPI TX BRIDGE) */
				axi2mipi_tx_bridge = (pcam->settings->stream[0] >> 4);

				switch(axi2mipi_tx_bridge)
				{
					case CSI_0:
						axi2mipi_tx_bridge = AXI2MIPI_BRIDGE_0;
						break;

					case CSI_1:
						axi2mipi_tx_bridge = AXI2MIPI_BRIDGE_1;
						break;

					default:
						SLOGF(
								SLOG_ERROR,
								"Not support CSI[%d]",
								axi2mipi_tx_bridge);
						return USECASE_HIRES_CSI_INVALID;
						break;
				}
				SLOGF(SLOG_DEBUG, "IMAGE WIDTH : %d", width);
				SLOGF(SLOG_DEBUG, "IMAGE HEIGHT: %d", height);
				SLOGF(SLOG_DEBUG, "MIPI TX : %d", (int)axi2mipi_tx_bridge);
				SLOGF(SLOG_DEBUG, "THUMBNAIL VC: %d", vc_id_thumbnail);
				SLOGF(SLOG_DEBUG, "SNAPSHOT  VC: %d", vc_id_snapshot);
				break;
			}
		}
		cam_idx++;
		bitmask >>= 1;
	}

	/* Get total cameras on ASIC board */
	get_global_values(m_bitmask);
#ifdef MIPI_DEBUG
	SLOGF(SLOG_DEBUG, "CAMERA TOTAL: %d", cam_total);
#endif
	num_of_frame = frame;

	/* Check parameter */
	if(IS_MIPI_DATA_TYPE(image_type) == FALSE)
	{
		SLOGF(SLOG_ERROR, "Not support data type: 0x%X", image_type);
		return USECASE_HIRES_CAM_IMG_DATA_TYPE_ERROR;
	}
	else if((cam_total > MAX_NUM_CAPT_CAMERA || cam_total == 0))
	{
		SLOGF(SLOG_ERROR, "Camera total: %d", cam_total);
		return USECASE_HIRES_CAM_TOTAL_ERROR;
	}
	else if(width == 0 || height == 0)
	{
		SLOGF(SLOG_ERROR, "Not support resolution: [%x] [%x]", width, height);
		return USECASE_HIRES_CAM_RESOLUTION_ERROR;
	}
	else if(frame == 0)
	{
		SLOGF(SLOG_ERROR, "Not support burst input = 0!");
		return USECASE_HIRES_CAM_FRAME_ERROR;
	}
	else if(vc_id_snapshot < VC0_FOR_FIFO || vc_id_snapshot > VC3_FOR_FIFO)
	{
		SLOGF(SLOG_ERROR, "Not support VCID snapshot: 0x%X", vc_id_snapshot);
		return USECASE_HIRES_VC_SNAPSHOT_INVALID;
	}
	else if(vc_id_thumbnail < VC0_FOR_FIFO || vc_id_thumbnail > VC3_FOR_FIFO)
	{
		SLOGF(SLOG_ERROR, "Not support VCID thumbnail: 0x%X", vc_id_thumbnail);
		return USECASE_HIRES_VC_THUMBNAIL_INVALID;
	}

	/* Initialize callback function */
	callback_hdl = snapshot_done_hdl;

	/* Initialize image transfer size */
	img_bytes = get_image_size((uint16_t)width, (uint16_t)height, image_type);
	/* Setup MIPI RX and TX */
	setup_mipi_rx_snapshot(
			m_bitmask,
			(uint16_t)width,
			(uint16_t)height,
			MIPI2AXI_RAW10,
			NULL);

	/* Initialize DPHY */
	hal_axi2mipi_init(axi2mipi_tx_bridge);
	priv_data.mbitmask = m_bitmask;
	if(has_cam_thumb)
	{
		/* Configure MIPI TX for thumnail */
		setup_mipi_tx_snapshot(
				axi2mipi_tx_bridge,
				vc_id_thumbnail,
				CONFIG_FOR_THUMBNAIL,
				width,
				height,
				MIPI2AXI_RAW10,
				AXI2MIPI_RAW10,
				NULL);
	}

	/* Configure MIPI TX for Snapshot */
	setup_mipi_tx_snapshot(
			axi2mipi_tx_bridge,
			vc_id_snapshot,
			CONFIG_FOR_SNAPSHOT,
			width,
			height,
			MIPI2AXI_RAW10,
			AXI2MIPI_LIGHT_RAW,
			NULL
			);

	return USERCASE_HIRES_NO_ERROR;
}

void task_tx_transfer(void *params)
{
	uint8_t skip_frame_mask = 0;
	uint8_t cam_module = 0;
	uint8_t fifo_x;
	uint64_t time_out = TX_TRANSFER_TASK_TIMEOUT;

	/* Selecting MIPI TX FIFO */
	if (has_cam_thumb)
		fifo_x = SNAPSHOT_AXI2MIPI_FIFO | THUMBNAIL_AXI2MIPI_FIFO;
	else
		fifo_x = SNAPSHOT_AXI2MIPI_FIFO;

	if(params)
		skip_frame_mask = *(uint8_t *)params;

	while((mipi_tx_fr_cnt < num_of_frame) && time_out)
	{
		uint8_t rx_frame_count = min_frame_done(mipi_rx, 6);
		if((mipi_tx_fr_cnt < rx_frame_count))
		{
			time_out = TX_TRANSFER_TASK_TIMEOUT;
			if (rx_frame_count & skip_frame_mask)
			{
				tx_ddr_mem_idx++;
				tx_ddr_mem_idx =
					(tx_ddr_mem_idx >= MAX_AREA_MEM) ? 0 : tx_ddr_mem_idx;
				light_header_cnt++;
				if(light_header_cnt >= MAX_AREA_MEM)
					light_header_cnt = 0;
				/* Switch next address */
				unsigned int ddr_address = get_real_tx_ddr_address(
					tx_ddr_mem_idx);

				SLOGF(SLOG_DEBUG,"New LRI address: 0x%08x \r\n", (int)ddr_address);

				/* Stop MIPI TX */
				WRITE_AXI2MIPI_REG(axi2mipi_tx_bridge, BRIDGE_CTRL_REG,
					DISABLE);
				/* Change destination memory address */
				WRITE_AXI2MIPI_REG(
					axi2mipi_tx_bridge,
					SNAPSHOT_AXI2MIPI_DMA_SRC_REG,
					ddr_address);
				/* Validate programming */
				WRITE_AXI2MIPI_REG(
					axi2mipi_tx_bridge,
					SNAPSHOT_AXI2MIPI_FRAME_CTRL_REG,
					ENABLE);
				if(has_cam_thumb)
				{
					unsigned int thumb_address = get_real_rx_ddr_address(cam_mem_idx[cam_thumb_mem_idx], tx_ddr_mem_idx);
					SLOGF(SLOG_DEBUG, "New thumb address: 0x%08x \r\n", (int)thumb_address);
					WRITE_AXI2MIPI_REG(
									axi2mipi_tx_bridge,
									THUMBNAIL_AXI2MIPI_DMA_SRC_REG,
									thumb_address);
					WRITE_AXI2MIPI_REG(
									axi2mipi_tx_bridge,
									THUMBNAIL_AXI2MIPI_FRAME_CTRL_REG,
									ENABLE);
				}

				/* Increase the number of frame transfer finished */
				mipi_tx_fr_cnt++;
				SLOGF(SLOG_INFO, "Skipping this frame");
			}
			else
			{
				prepare_capture_info(priv_data.mbitmask, light_header_cnt);
				light_header_cnt++;
				if(light_header_cnt >= MAX_AREA_MEM)
					light_header_cnt = 0;
				/* Start MIPI TX */
				hal_axi2mipi_start(axi2mipi_tx_bridge, fifo_x);

				EventBits_t evb = xEventGroupWaitBits(priv_data.hires_evhdl,
							EVENT_TX_FRAME_COMPLETE, pdTRUE, pdTRUE, 5000);
				if(!(evb & EVENT_TX_FRAME_COMPLETE))
				{
					SLOGF(SLOG_ERROR, "TX transfer timed out");
					goto release;
				}
				mipi_tx_done_flag = 0;
			}
		}
		time_out--;
		vTaskDelay(1);
	}

	if(time_out == 0)
		SLOGF(SLOG_ERROR, "Didn't receive frames from all sensors");

release:
	if(mipi_tx_fr_cnt >= num_of_frame)
	{
		SLOGF(SLOG_INFO, "Snapshot completed");
	}
	else
	{
		SLOGF(SLOG_INFO, "Snapshot fail");
	}
	for(int i = 0; i < MAX_NUM_CAPT_CAMERA; i++)
	{
		if(mipi_rx[i].active)
		{
#if (ASIC_NUM == ASIC1)
		cam_module = (i==0?0xC5:(i==1?0xB5:(i==2?0xB4:(i==3?0xA5:(i==4?0xA1:0xB2)))));
#elif (ASIC_NUM == ASIC2)
		cam_module = (i==0?0xA4:(i==1?0xB3:(i==2?0xC2:(i==3?0xA2:(i==4?0xA3:0xB1)))));
#elif (ASIC_NUM == ASIC3)
		cam_module = (i==0?0xC6:(i==1?0x00:(i==2?0xC1:(i==3?0x00:(i==4?0xC4:0xC3)))));
#endif
			SLOGF(SLOG_INFO,
					"RX[%d]: CAM[%02X] - ACT[%s] - FS[%d] - FE[%d] - DONE[%d]",
					i,
					cam_module,
					((mipi_rx[i].active == 1) ? "ENA" : "DIS"),
					mipi_rx[i].fs_cnt,
					mipi_rx[i].fe_cnt,
					mipi_rx[i].fdone_cnt);
			hal_syncio_disable(i);
		}
	}
	if(priv_data.hires_evhdl)
		vEventGroupDelete(priv_data.hires_evhdl);
	priv_data.hires_evhdl = NULL;
	tx_transfer_done = 1;
	SLOGF(SLOG_DEBUG, "Deleting %s", __FUNCTION__);
	vTaskDelete(NULL);
}
uint8_t snapshot_hdl(uint32_t time_out, uint8_t skip_mask)
{
	uint32_t tx_time_out = 0;
	uint8_t count = 0;
	uint8_t rx_frame_count = 0;
	uint8_t error = 0;
	uint8_t i = 0;
	uint8_t cam_module = 0;

	/* While until snapshot done */
	while((mipi_tx_fr_cnt < num_of_frame) && (count < num_of_frame) && time_out)
	{
		rx_frame_count = min_frame_done(mipi_rx, 6);

		if((mipi_tx_fr_cnt < rx_frame_count))
		{
			/* If we have RX frame */
			count++;
			/* Wait for Light Header to populated */
			EventBits_t evb = xEventGroupWaitBits(priv_data.hires_evhdl,
							EVENT_LIGHT_HEADER_READY, pdTRUE, pdTRUE, time_out);
			if(evb & EVENT_LIGHT_HEADER_READY)
			{
				if (rx_frame_count & skip_mask)
				{
					tx_ddr_mem_idx++;
					tx_ddr_mem_idx =
						(tx_ddr_mem_idx >= MAX_AREA_MEM) ? 0 : tx_ddr_mem_idx;

					/* Switch next address */
					unsigned int ddr_address = get_real_tx_ddr_address(
						tx_ddr_mem_idx);

					SLOGF(SLOG_INFO,"TX next DDR address: 0x%08x \r\n", (int)ddr_address);
					/* Stop MIPI TX */
					WRITE_AXI2MIPI_REG(axi2mipi_tx_bridge, BRIDGE_CTRL_REG,
						DISABLE);
					/* Change destination memory address */
					WRITE_AXI2MIPI_REG(
						axi2mipi_tx_bridge,
						SNAPSHOT_AXI2MIPI_DMA_SRC_REG,
						ddr_address);
					/* Validate programming */
					WRITE_AXI2MIPI_REG(
						axi2mipi_tx_bridge,
						SNAPSHOT_AXI2MIPI_FRAME_CTRL_REG,
						ENABLE);

					/* Increase the number of frame transfer finished */
					mipi_tx_fr_cnt++;
					SLOGF(SLOG_INFO, "Skipping this frame");
				}
				else
				{
					unsigned int ddr_address = READ_AXI2MIPI_REG(axi2mipi_tx_bridge, SNAPSHOT_AXI2MIPI_DMA_SRC_REG);
					SLOGF(SLOG_INFO, "TX Transfer address: 0x%x", ddr_address);
					if (has_cam_thumb)
					{
						/* Start MIPI TX */
						hal_axi2mipi_start(
							axi2mipi_tx_bridge,
							SNAPSHOT_AXI2MIPI_FIFO | THUMBNAIL_AXI2MIPI_FIFO);
					}
					else
					{
						/* Start MIPI TX */
						hal_axi2mipi_start(
							axi2mipi_tx_bridge,
							SNAPSHOT_AXI2MIPI_FIFO);
					}
					tx_time_out = 2000;
					while ((mipi_tx_done_flag == 0) && tx_time_out)
					{
						vTaskDelay(1);
						tx_time_out--;
					}
					mipi_tx_done_flag = 0;
					if (!tx_time_out)
					{
						SLOGF(SLOG_ERROR, "TX time out");
						error = 1;
						break;
					}
				}

			}
			else
			{
				SLOGF(SLOG_ERROR, "Wait Light Header time out");
				vEventGroupDelete(priv_data.hires_evhdl);
				priv_data.hires_evhdl = NULL;
				error = 1;
				break;
			}

		}
		else
		{
			time_out--;
			vTaskDelay(1);
		}
	}

	/* One more chance to check the number of frame */
	if(mipi_tx_fr_cnt >= num_of_frame)
		error = 0; /* No error, snapshot done */
	else
		error = 1; /* Error */

	/* TODO: remove these debug lines */
	for(i = 0; i < MAX_NUM_CAPT_CAMERA; i++)
	{
#if (ASIC_NUM == ASIC1)
		cam_module = (i==0?0xC5:(i==1?0xB5:(i==2?0xB4:(i==3?0xA5:(i==4?0xA1:0xB2)))));
#elif (ASIC_NUM == ASIC2)
		cam_module = (i==0?0xA4:(i==1?0xB3:(i==2?0xC2:(i==3?0xA2:(i==4?0xA3:0xB1)))));
#elif (ASIC_NUM == ASIC3)
		cam_module = (i==0?0xC6:(i==1?0x00:(i==2?0xC1:(i==3?0x00:(i==4?0xC4:0xC3)))));
#endif
		SLOGF(SLOG_INFO,
				"RX[%d]: CAM[%02X] - ACT[%s] - FS[%d] - FE[%d] - DONE[%d]",
				i,
				cam_module,
				((mipi_rx[i].active == 1) ? "ENA" : "DIS"),
				mipi_rx[i].fs_cnt,
				mipi_rx[i].fe_cnt,
				mipi_rx[i].fdone_cnt);
		hal_syncio_disable(i);
	}
	return error;
}


static void syncio_intr(hal_syncio_channel_t chid,
											hal_syncio_irq_mode_t irq_status)
{
	synio_count++;
	//printf("\r\nSYNC IO CH[%d]: Count: %d\r\n",(int)chid, synio_count);
}

uint8_t cam_capture_syncio_config(cam_typedef_t *cam, uint8_t frame_num,
				double latency_time, syncio_trig_mode_t mode)
{
	/* Initialize sync IO */
	hal_syncio_cfg_t syncio_cfg;
	int8_t cam_ch = cam->info.ch - 1;
	if(cam_ch < 0 || cam_ch > 5)
	{
		SLOGF(SLOG_ERROR, "Does not support channel [%d]", cam_ch);
		/* return 1 that mean error */
		return 1;
	}
	SLOGF(SLOG_DEBUG, "Latency time (x10^6): %d",
										(unsigned int)(latency_time * 1000000));
	/* Configures sync IO */
	syncio_cfg.inf_mode = SG_INF_DIS;
	syncio_cfg.repeat = frame_num;
	/* In snapshot flow, the trigger mode should be HW trigger */
	syncio_cfg.trig_mode = mode;
	syncio_cfg.width1 = 1 * SPG_MILISECOND;
	syncio_cfg.lat1 = (uint32_t)((latency_time + 0.001206) * SPG_SECOND);
	syncio_cfg.width2 = 0;
	syncio_cfg.lat2 = 0;
#ifdef MIPI_DEBUG
	SLOGF(SLOG_DEBUG, "Syncio[%d] latency %d, frame_num %d", cam_ch, syncio_cfg.lat1, frame_num);
#endif

	/* Start initialize to default */
	hal_syncio_init(cam_ch);
	hal_syncio_config(cam_ch, &syncio_cfg);
	hal_syncio_irq_t intr;
	intr.irq_modes = SG_INTR_PULSE_DONE;
	intr.callback_handler = syncio_intr;
	hal_syncio_enable_irq(cam_ch, &intr);
	hal_syncio_enable(cam_ch);

	/* Not error */
	return 0;
}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
