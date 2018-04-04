/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_cmd.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of lcc_cmd
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "assert.h"
#include "os.h"
#include "log.h"
#include "i2cs.h"
#include "spis.h"
#include "i2cm.h"
#include "lcc_cmd.h"
#include "lcc_cmd_log.h"
#include "light_system.h"


/* Private define-------------------------------------------------------------*/
#define SLOGF_ID		SLOG_ID_LCC_CMD

/* Configure LCC data buffer size */
#define LCC_CMD_MAX_BUFFER_SIZE             1024

/* Number of LCC commands */
#define LCC_CMD_NBR		(sizeof(lcc_cmd_list)/sizeof(lcc_cmd_list[0]))

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Handler of LCC command structure
 */
typedef struct
{
	uint32_t cmd;
	void (*func)(lcc_cmd_t *in, lcc_cmd_t *out);
} lcc_cmd_handle_t;

/**
 * @brief  Buffer of LCC command structure
 */
typedef struct {
	uint8_t *data;
	uint16_t len;
} lcc_cmd_msg_t;

typedef struct
{
	uint8_t asic2_len;
	uint8_t asic3_len;
	uint32_t asic2_bitmask;
	uint32_t asic3_bitmask;
	uint8_t *asic2_data;
	uint8_t *asic3_data;
} lcc_asic_data_t;

typedef struct
{
	lcc_cmd_store_asic1_t *head;
	lcc_cmd_store_asic1_t *tail;
	uint8_t total;
} asic_command_waiting_list_t;

typedef struct
{
	uint16_t cmd_tid;
	uint16_t cmd_status;
} lcc_intr_src_data_t;

typedef struct {
    uint8_t *buf_start;
    const uint8_t *buf_end;
    uint8_t *wp;
    const uint8_t *rp;
} cmd_rx_buf_t;

typedef struct {
    const uint8_t* msg_start;
    uint16_t len;
    bool buf_overflow;
} cmd_rx_buf_msg_t;

/* Private macro -------------------------------------------------------------*/

#define ADD_LCC_CMD(c_cmd, c_func) {.func = &c_func, .cmd = c_cmd }

#define CMD_RX_COMPLETED			BIT0
#define CMD_TX_REQUESTED			BIT1

/* Private functions ---------------------------------------------------------*/
#if (LOG_I2C_MESSAGE == STD_ON)
/* The function print out full message received from MSG queue for debugging */
static void slogf_message(lcc_cmd_msg_t msg);
#endif /* LOG_VERBOSE == STD_ON */
/* Reset CCI RX buffer */
static void cb_rx_completed(void);
/* CCI data receive call back */
static void cb_rx_not_empty(uint8_t data);
/* Prepare data for reading command*/
static void cb_tx_request(void);

#if (ASIC_NUM == ASIC1)
uint8_t asic1_query_asic2(uint16_t *asic2_status);
uint8_t asic1_query_asic3(uint16_t *asic3_status);
void asic_command_list_init(void);
uint8_t push_asic_command_list(lcc_cmd_store_asic1_t msg);
uint8_t asic1_get_data(lcc_cmd_store_asic1_t *info);
uint8_t asic1_push_read_cmd(lcc_cmd_store_asic1_t *info);
uint8_t asic1_get_intr_source(lcc_intr_src_data_t *asic2,
							lcc_intr_src_data_t *asic3);
uint8_t asic1_init_cache(uint8_t asic_num);
uint8_t asic1_get_eeprom(lcc_cmd_store_asic1_t *info);
uint8_t asic1_get_data(lcc_cmd_store_asic1_t *info);
#endif
/* Private variables ---------------------------------------------------------*/

static const lcc_cmd_handle_t lcc_cmd_list[] =
{
	/* LCC CMD GROUP 0x0000 */
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_OPEN, cmd_cam_module_open),
	ADD_LCC_CMD(LCC_CMD_CAM_STREAMING, cmd_cam_streaming),
	ADD_LCC_CMD(LCC_CMD_CAM_BURST_REQUESTED, cmd_cam_burst_requested),
	ADD_LCC_CMD(LCC_CMD_CAM_BURST_AVAILABLE, cmd_cam_burst_available),
	ADD_LCC_CMD(LCC_CMD_CAM_BURST_ACTUAL, cmd_cam_burst_actual),
	ADD_LCC_CMD(LCC_CMD_CAM_SNAPSHOT_UUID, cmd_cam_snapshot_uuid),
	ADD_LCC_CMD(LCC_CMD_CAM_SNAPSHOT_TID, cmd_cam_snapshot_tid),
	ADD_LCC_CMD(LCC_CMD_CAM_COMMAND_STATUS, cmd_cam_command_status),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_STATUS, cmd_cam_module_status),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_RESOLUTION, cmd_cam_module_resolution),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_SENSITIVITY, cmd_cam_module_sensitivity),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_EXPOSURE_TIME, cmd_cam_module_exposure_time),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_FPS, cmd_cam_module_fps),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_FOCAL_LENGTH, cmd_cam_module_focal_length),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_FOCUS_DISTANCE, cmd_cam_module_focus_distance),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_LENS_POSITION, cmd_cam_module_lens_position),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_MIRROR_POSITION, cmd_cam_module_mirror_position),
	ADD_LCC_CMD(LCC_CMD_CAM_ROI, cmd_cam_roi),
    ADD_LCC_CMD(LCC_CMD_CAM_ROI_TRANSFER, cmd_cam_roi_transfer),
	ADD_LCC_CMD(LCC_CMD_CAM_ROI_CALIBRATION, cmd_cam_roi_calibration),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_UUID, cmd_cam_module_uuid),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_TYPE, cmd_cam_module_type),
	ADD_LCC_CMD(LCC_CMD_I2C_FORWARD, cmd_i2c_forward),
	ADD_LCC_CMD(LCC_CMD_ZOOM_FACTOR, cmd_zoom_factor),
	ADD_LCC_CMD(LCC_CMD_CAM_MODULE_MIRROR_CALIBRATION, cmd_cam_module_mirror_calib),
	ADD_LCC_CMD(LCC_CMD_I2C_SPEED, cmd_i2c_speed),
	ADD_LCC_CMD(LCC_CMD_SPI_SPEED, cmd_spi_speed),
	/* CBB CMD GROUP 0x0200 */
	ADD_LCC_CMD(LCC_CMD_ASIC_LIGHT_PROTOCOL, cmd_asic_light_protocol),
	ADD_LCC_CMD(LCC_CMD_ASIC_FW_VERSION, cmd_asic_fw_version),
	ADD_LCC_CMD(LCC_CMD_ASIC_CALIB_VERSION, cmd_asic_calib_version),
	ADD_LCC_CMD(LCC_CMD_ASIC_STATUS, cmd_asic_status),
	ADD_LCC_CMD(LCC_CMD_ASIC_LOG_CTRL, cmd_asic_log_ctrl),
	ADD_LCC_CMD(LCC_CMD_ASIC_TMPx, cmd_asic_tmpx),
	ADD_LCC_CMD(LCC_CMD_ASIC_CMD_DUMP, cmd_asic_cmd_dump),
	ADD_LCC_CMD(LCC_CMD_ASIC_MODULE_METADATA, cmd_asic_module_metadata),
	ADD_LCC_CMD(LCC_CMD_ASIC_DEVICE_CALIBRATION, cmd_asic_device_calibration),
	ADD_LCC_CMD(LCC_CMD_ASIC_PWR_CTRL, cmd_asic_pwr_ctrl),
	ADD_LCC_CMD(LCC_CMD_AISC_INTR_SRC, cmd_cam_asic_intr_src),
	ADD_LCC_CMD(LCC_CMD_ASIC_FLASH_LIGHT, cmd_asic_flash_light),
	ADD_LCC_CMD(LCC_CMD_ASIC_TOF, cmd_asic_tof),
	ADD_LCC_CMD(LCC_CMD_ASIC_GYRO, cmd_asic_gyro),
#if (ASIC_NUM == ASIC1)
	ADD_LCC_CMD(LCC_CMD_ASIC_CTRL, cmd_asic_control),
	ADD_LCC_CMD(LCC_CMD_ASIC_POWER_INFO, cmd_asic_power_info),
#endif
	ADD_LCC_CMD(LCC_CMD_ASIC_CALIB_DATA, cmd_asic_calib_data),
		ADD_LCC_CMD(LCC_CMD_ASIC_PZT_PWR_CTRL, cmd_asic_pzt_pwr_ctrl),
	/* CBB CMD GROUP 0x1000 */
	ADD_LCC_CMD(LCC_CMD_LIGHT_ACTIVE_UCID, cmd_light_active_ucid)
};

static uint8_t cmd_rx_buf_arr[LCC_CMD_MAX_BUFFER_SIZE];
static cmd_rx_buf_t cmd_rx_buf;
static cmd_rx_buf_msg_t cmd_rx_buf_msg;
static QueueHandle_t cmd_rx_msg_queue;

#if (ASIC_NUM == ASIC1)
static asic_command_waiting_list_t *asic_command_list;
static TaskHandle_t task_read_handle;
TaskHandle_t task_asic_read_handle;
#endif

SemaphoreHandle_t asic_read_sem;
/* Export functions ----------------------------------------------------------*/
#if (ASIC_NUM == ASIC1)
void task_asic_read(void *param)
{
	assert_param(light_system->asic1_read_event);
	EventBits_t event_bits_read;
	lcc_cmd_store_asic1_t cmd_store_tmp;
	lcc_cmd_store_asic1_t *arch_msg = NULL;
	lcc_intr_src_data_t intr_asic2, intr_asic3;
	asic_read_sem = xSemaphoreCreateMutex();
	while(1)
	{
		event_bits_read = xEventGroupWaitBits(light_system->asic1_read_event,
									ASIC_READ_EVENT_ALLS,
									pdTRUE, pdFALSE, portMAX_DELAY);
		xSemaphoreTake(asic_read_sem, portMAX_DELAY);
		if(asic_command_list->total >= 1 && event_bits_read)
		{
			if(event_bits_read & ASIC_READ_ASIC_2)
			{
				asic1_get_intr_source(&intr_asic2, NULL);
				arch_msg = asic_cmd_pop_msg(intr_asic2.cmd_tid);
				if(light_system->asic_intr_queue_num.asic2_intr_num)
					light_system->asic_intr_queue_num.asic2_intr_num--;
				if(arch_msg != NULL)
				{
					memcpy(&cmd_store_tmp, arch_msg, sizeof(lcc_cmd_store_asic1_t));
					/* Reset bits of other ASIC */
					cmd_store_tmp.m_bitmask &= ~light_system->m_filter_asic2;
					if (arch_msg->cmd.read)
					{
						asic1_push_read_cmd(&cmd_store_tmp);
						if((arch_msg->cmd.cmd == LCC_CMD_READ_EEPROM_METADATA_ID) &&
							(arch_msg->cmd.base == LCC_CMD_READ_EEPROM_METADATA_BASE))
							asic1_get_eeprom(&cmd_store_tmp);
						else
							asic1_get_data(&cmd_store_tmp);
					}
					lcc_cmd_log_update_status(swap16(intr_asic2.cmd_tid),
						(intr_asic2.cmd_status), cmd_store_tmp.m_bitmask);
					/* Reset bit of itself */
					arch_msg->m_bitmask &= ~light_system->m_filter_asic1;
					if(!arch_msg->m_bitmask)
						asic_cmd_delete_msg(arch_msg);
				}
				else
				{
					SLOGF(SLOG_ERROR, "Cannot find TID %X", intr_asic2.cmd_tid);
				}
				if(light_system->asic_intr_queue_num.asic2_intr_num)
					xEventGroupSetBits(light_system->asic1_read_event,
						ASIC_READ_ASIC_2);
			}

			if(event_bits_read & ASIC_READ_ASIC_3)
			{
				asic1_get_intr_source(NULL, &intr_asic3);
				arch_msg = asic_cmd_pop_msg(intr_asic3.cmd_tid);
				if(light_system->asic_intr_queue_num.asic3_intr_num)
					light_system->asic_intr_queue_num.asic3_intr_num--;
				if(arch_msg != NULL)
				{
					memcpy(&cmd_store_tmp, arch_msg, sizeof(lcc_cmd_store_asic1_t));
					/* Reset bits of other ASIC */
					cmd_store_tmp.m_bitmask &= ~light_system->m_filter_asic1;
					if (arch_msg->cmd.read)
					{
						asic1_push_read_cmd(&cmd_store_tmp);
						if((arch_msg->cmd.cmd == LCC_CMD_READ_EEPROM_METADATA_ID) &&
							(arch_msg->cmd.base == LCC_CMD_READ_EEPROM_METADATA_BASE))
							asic1_get_eeprom(&cmd_store_tmp);
						else
							asic1_get_data(&cmd_store_tmp);
					}
					lcc_cmd_log_update_status(swap16(intr_asic3.cmd_tid),
						(intr_asic3.cmd_status), cmd_store_tmp.m_bitmask);
					/* Reset bit of itself */
					arch_msg->m_bitmask &= ~light_system->m_filter_asic2;
					if(!arch_msg->m_bitmask)
						asic_cmd_delete_msg(arch_msg);
				}
				else
				{
					SLOGF(SLOG_ERROR, "Cannot find TID %X", intr_asic3.cmd_tid);
				}
				if(light_system->asic_intr_queue_num.asic3_intr_num)
					xEventGroupSetBits(light_system->asic1_read_event,
						ASIC_READ_ASIC_3);
			}
		}
		xSemaphoreGive(asic_read_sem);
	}
}

#define ASIC1_SYNC_TIMEOUT			6000 /* 30s */
void task_sync_asic(void *param)
{
	SLOGF(SLOG_INFO, "Task sync started");
	uint8_t asic2_3_status = 0;
	uint16_t asic2_status = 0, asic3_status = 0;
	uint16_t timeout = ASIC1_SYNC_TIMEOUT;
	hal_gpio_t intr_asic2, intr_asic3;
	intr_asic2.direction = GPIO_DIR_IN;
	intr_asic2.port = GPIO_PORTA;
	intr_asic2.pin = light_system->asic2_intr;
	hal_gpio_init(&intr_asic2);
	intr_asic3.direction = GPIO_DIR_IN;
	intr_asic3.port = GPIO_PORTA;
	intr_asic3.pin = light_system->asic3_intr;
	hal_gpio_init(&intr_asic3);
	while(1)
	{
		if(((asic2_3_status & ASIC_2) != ASIC_2)  && hal_gpio_read(&intr_asic2))
		{
			asic1_query_asic2(&asic2_status);
			asic2_3_status |= ASIC_2;
			asic1_init_cache(ASIC_2);
			SLOGF(SLOG_INFO, "ASIC2 is ready...");
		}
		if(((asic2_3_status & ASIC_3) != ASIC_3)  && hal_gpio_read(&intr_asic3))
		{
			asic1_query_asic3(&asic3_status);
			asic2_3_status |= ASIC_3;
			asic1_init_cache(ASIC_3);
			SLOGF(SLOG_INFO, "ASIC3 is ready...");
		}

		if(asic2_3_status == (ASIC_2 | ASIC_3))
		{
			SLOGF(SLOG_INFO, "Data Sync successes");
			light_system->asic_intr_queue_num.asic2_intr_num = 0;
			light_system->asic_intr_queue_num.asic3_intr_num = 0;
			vTaskDelete(task_read_handle);
		}
		else if(--timeout == 0)
		{
			SLOGF(SLOG_ERROR, "ASIC Sync timeout...");
			vTaskDelete(task_read_handle);
		}
		vTaskDelay(5);
	}
}
#endif

static void lcc_cmd_msg_create(lcc_cmd_msg_t *msg, const cmd_rx_buf_msg_t *rx_msg)
{
    msg->len = rx_msg->len;
    size_t size = msg->len * sizeof(uint8_t);
    msg->data = pvPortMalloc(size);
    const uint8_t *rp_new = rx_msg->msg_start + rx_msg->len;
    // Circular memcpy from cmd_rx_buf to msg->data
    uint16_t buf_arr_rem = cmd_rx_buf.buf_end - rx_msg->msg_start;
    if (buf_arr_rem < rx_msg->len)
    {
        uint16_t len_wrap = rx_msg->len - buf_arr_rem;
        memcpy(msg->data + buf_arr_rem, cmd_rx_buf.buf_start, len_wrap * sizeof(uint8_t));
        size = buf_arr_rem * sizeof(uint8_t);
        rp_new = cmd_rx_buf.buf_start + len_wrap;
    }
    memcpy(msg->data, rx_msg->msg_start, size);
    // Modify rp atomically, to ensure it is seen consistently changed in interrupt
    taskENTER_CRITICAL();
    cmd_rx_buf.rp = rp_new;
    taskEXIT_CRITICAL();
}

/**
 * LCC command task
 */
void task_lcc_cmd(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);

	lcc_cmd_t in;
	in.data = NULL;
	lcc_cmd_t out;
	lcc_cmd_info_t cmd_info;

#if (ASIC_NUM != ASIC1)
	uint32_t cmd_bitmask = 0;
	uint8_t asic_data_idx;
	uint8_t data_cnt = 0;
	uint8_t count_idx = 0;
#endif
#if (ASIC_NUM == ASIC1)
	lcc_cmd_store_asic1_t asic_data;
#endif

	memset(&out, 0, sizeof(lcc_cmd_t));

	/* Start initialize temperature sensor */
	bool exit = false;
	while (!exit)
	{
		/* Task #__task_i2c_ex initialize done for temperature sensor,
		 * #task_console done for slogf */
		if (TASK_READY == task_handler[task_query_tid("console")].state &&
			TASK_READY == task_handler[task_query_tid("queuelog")].state)
			exit = true;
			vTaskDelay(1);
	}

	/* Task ready id */
	hdl->state = TASK_READY;
	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
#if (ASIC_NUM == ASIC1)
	xTaskCreate(task_asic_read, "Read_ASIC2_ASIC3",
		__TASK_STACK_SIZE_128, NULL,
		__TASK_PRIO_HIGHEST - 7, &task_asic_read_handle);

	xTaskCreate(task_sync_asic, "Sync_ASIC_data",
		__TASK_STACK_SIZE_128, NULL,
		__TASK_PRIO_HIGHEST - 8, &task_read_handle);
#endif
	/* Loop forever */
	while (1)
	{
	    cmd_rx_buf_msg_t rx_msg;
		xQueueReceive(cmd_rx_msg_queue, &rx_msg, portMAX_DELAY);

		lcc_cmd_msg_t msg;
		lcc_cmd_msg_create(&msg, &rx_msg);

#if (LOG_I2C_MESSAGE == STD_ON)
		/* Print out all content message received for debugging */
		slogf_message(msg);
#endif
		if (LCC_CMD_M_BITMASK_IDX > msg.len)
		{
			in.status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_ERROR, "%s: Message too short", __FUNCTION__);
			goto EXIT;
		}

		/* Get command transaction id */
		in.tid = *(uint16_t *)(msg.data + LCC_CMD_TRANSACTION_ID_IDX);
		in.cmd = *(uint16_t *)(msg.data + LCC_CMD_COMMAND_IDX);

		uint16_t count = LCC_CMD_M_BITMASK_IDX;
		uint16_t scan_idx = 0;
		/* TODO: optimize the search method */
		/* Scan the matching command in task list */
		for (scan_idx = 0; scan_idx < LCC_CMD_NBR; scan_idx++)
		{
			/* Search the matching command in lcc_cmd_list[] */
			cmd_info = (lcc_cmd_info_t)lcc_cmd_list[scan_idx].cmd;
			if ((in.cmd & 0x7FFF) == (cmd_info.word & 0x7FFF))
			{
				/* Get interrupt signal */
				in.intr = in.cmd >> 15;
				/* Get m_bitmask if this command support */
				if (cmd_info.m_bitmask)
				{
					in.m_bitmask  = (uint32_t)msg.data[count++];
					in.m_bitmask |= (uint32_t)msg.data[count++] << 8;
					in.m_bitmask |= (uint32_t)msg.data[count++] << 16;
					/* Check if m_bitmask is set over 16 camera modules */
					if (in.m_bitmask & (~LCC_CMD_M_BITMASK_MASK))
					{
						in.status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s: m_bitmask invalid",__FUNCTION__);
						break;
					}

					/* Check global in m_bitmask */
					in.global = in.m_bitmask & LCC_CMD_M_BITMASK_GLOBAL_MASK;
					if ((0 == cmd_info.global) && (1 == in.global))
					{
						in.global = 0;		/* Clear global mask */
						in.status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s: Unsupported global bitmask",
											__FUNCTION__);
						break;
					}
					else if (in.global)
					{
						/* Set the module bitmask to all of module belong to
						 * this ASIC */
						in.m_bitmask = light_system->m_bitmask_all;
					}
					/* Get number of bit 1 */
					in.m_number = __builtin_popcount(in.m_bitmask);
				}
				else
				{
					/* if m_bitmask is not apply then it should be zero*/
					in.m_bitmask = 0;
				}

				/* Get ucid if this command support */
				if (cmd_info.ucid)
				{
					in.ucid  = (uint16_t)msg.data[count++];
					in.ucid |= (uint16_t)msg.data[count++] << 8;
					/* Checking UCID value */
					if (UC_MAX <= in.ucid)
					{
						in.status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s: UCID invalid", __FUNCTION__);
						break;
					}
				}

				/* Read condition detected */
				if (count == msg.len)
				{
					if (0 == cmd_info.read)
					{
						in.status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s: Unsupported read action",
											__FUNCTION__);
						break;
					}
					in.action = CMD_READ;
				}
				/* Write condition checking  */
				else if (cmd_info.write)
				{
					uint16_t expected_len = cmd_info.size;
					if (cmd_info.bypass)
						expected_len = msg.len - count;
					else if (!in.global && cmd_info.m_bitmask)
						expected_len = cmd_info.size * in.m_number;

					if (expected_len == msg.len - count)
					{
						/* Set the pointer and length of data position */
						in.len = expected_len;
						in.data = &msg.data[count];
						/* Indicate the writing action for command */
						in.action = CMD_WRITE;
#if (ASIC_NUM == ASIC1)
						uint8_t tmp = msg.data[3];
						if(cmd_info.m_bitmask)
						{
							if((in.m_bitmask & light_system->m_filter_asic1) ||
							(in.m_bitmask & light_system->m_filter_asic2))
							{
								/* Enable interrupt for asic2/asic3 */
								msg.data[3] |= 0x80;
								asic_data.m_bitmask = in.m_bitmask &
									~light_system->m_filter;
								asic_data.tid = in.tid;
								asic_data.cmd = cmd_info;
								asic_data.ucid = in.ucid;
								asic_data.cmd_idx = scan_idx;
								push_asic_command_list(asic_data);
								if(in.m_bitmask & light_system->m_filter_asic1)
									i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
											I2C_FORWARD_SLAVEADDR,
											msg.data, msg.len, NULL, 0);
								if(in.m_bitmask & light_system->m_filter_asic2)
									i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
											I2C_FORWARD_SLAVEADDR,
											msg.data, msg.len, NULL, 0);
							}
						}
						else
						{
							/* Reset intr bit */
							msg.data[3] &= ~0x80;
							i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
										I2C_FORWARD_SLAVEADDR,
										msg.data, msg.len, NULL, 0);
							i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
										I2C_FORWARD_SLAVEADDR,
										msg.data, msg.len, NULL, 0);
						}
						msg.data[3] = tmp;
#endif
					}
					else
					{
						in.status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s: Message length is incorrect",
											__FUNCTION__);
						break;
					}
				}
				else
				{
					in.status = LCC_CMD_INVALID_ARG;
					SLOGF(SLOG_ERROR, "%s: Unsupported write action",
											__FUNCTION__);
						break;
				}
#if (ASIC_NUM != ASIC1)
				/* Parse data respecive with m_bitmask */
				if(cmd_info.m_bitmask)
				{
					cmd_bitmask = in.m_bitmask;
					if((in.action == CMD_WRITE) && (cmd_info.cmd !=
								LCC_CMD_READ_EEPROM_METADATA_ID))
					{
						in.m_bitmask &= light_system->m_filter;
						data_cnt = 0;
						count_idx = count;
						if(in.global)
						{
							in.data = &msg.data[count_idx];
						}
						else if(in.m_bitmask)
						{
							uint8_t *tmp = pvPortMalloc(__builtin_popcount(
								in.m_bitmask) * cmd_info.size);
							for(uint8_t i = 0; i < in.m_number; i++)
							{
								asic_data_idx = __builtin_ctz(cmd_bitmask);
								cmd_bitmask &= ~(1 << asic_data_idx);
								if(light_system->m_filter &
									(1 << asic_data_idx))
								{
									memcpy(&tmp[data_cnt],
										&msg.data[count_idx], cmd_info.size);
									data_cnt += cmd_info.size;
								}
								count_idx += cmd_info.size;
							}
							memcpy(&msg.data[count], tmp, data_cnt);
							in.data = &msg.data[count];
							in.len = data_cnt;
							vPortFree(tmp);
						}
						in.m_number = __builtin_popcount(in.m_bitmask);
					}
				}
#endif
				in.status = LCC_CMD_PENDING;
				light_system->cur_tid = in.tid;
				/* Execute function correspond with command from host  */
                out.len = 0;
                out.data = NULL;
				(*lcc_cmd_list[scan_idx].func)(&in, &out);

				if ((CMD_READ == in.action) && (out.len))
				{
					if (LCC_CMD_MAX_BUFFER_SIZE < out.len)
					{
						SLOGF(SLOG_ERROR, "The return message too long");
						out.len = LCC_CMD_MAX_BUFFER_SIZE;
					}
					i2c_slave_write(out.data, out.len);
					/* Update the command status */
					in.status = LCC_CMD_SUCCESS;
				}
				else
				{
				    vPortFree(out.data);
				}
                out.len = 0;
                out.data = NULL;
				break;		/* Scanning completed */
			}
		}
		if (scan_idx == LCC_CMD_NBR)
		{
			SLOGF(SLOG_WARN, "Unsupported command ID %04X", in.cmd);
		}
EXIT:
#if (ASIC_NUM != ASIC1)
		/* For now, ASIC1 will handle command status so ASIC2/ASIC3
		 * should not care about status of read command and don't
		 * need to put it into log queue */
		if(in.action == CMD_WRITE)
		{
			/* Push command to log */
			lcc_cmd_log_push(in.tid, in.status, in.m_bitmask, in.intr);
		}
#else
		/* Push command to log */
		lcc_cmd_log_push(in.tid, in.status, in.m_bitmask, in.intr);
#endif
		/* TODO: Store message to msg queue, using RTOS QUEUE */
		/* TODO: Trigger interrupt signal if command requested */
		/* Flush input message memory and watting for next command */
		memset(&in, 0, sizeof(lcc_cmd_t));
		/* Free the message memory */
		vPortFree(msg.data);
		msg.data = NULL;
	}
}

void lcc_cmd_init(void)
{
    cmd_rx_buf.buf_start = cmd_rx_buf_arr;
    cmd_rx_buf.buf_end = cmd_rx_buf_arr + LCC_CMD_MAX_BUFFER_SIZE;
    cmd_rx_buf.wp = cmd_rx_buf.buf_start;
    cmd_rx_buf.rp = cmd_rx_buf.buf_start;
    cmd_rx_buf_msg.msg_start = cmd_rx_buf.buf_start;
    cmd_rx_buf_msg.len = 0;
    cmd_rx_buf_msg.buf_overflow = false;

    cmd_rx_msg_queue = xQueueCreate(50, sizeof(cmd_rx_buf_msg_t));

    /* Setup I2C slave to host communication */
    i2c_slave_t i2c_slave;
    i2c_slave.clbk_hdl = cb_rx_not_empty;
    i2c_slave.receiver_hdl = cb_rx_completed;
    i2c_slave.restart_hdl = cb_tx_request;
    i2c_slave_init(&i2c_slave);

    /* Initialize Light system log */
    lcc_cmd_log_init();

#if (ASIC_NUM == ASIC1)
    /* Init linked list read back data from ASIC2/ASIC3 */
    asic_command_list_init();

    i2cm.init(I2C_FORWARD_ASIC2_CHANNEL);
    i2cm.init(I2C_FORWARD_ASIC3_CHANNEL);
#endif

}

/* Private functions ---------------------------------------------------------*/
/**
 * slogf_message
 * The function print out full message received from MSG queue for debugging
 */
#if (LOG_I2C_MESSAGE == STD_ON)
static void slogf_message(lcc_cmd_msg_t msg)
{
	char *str = pvPortMalloc(msg.len * 4);
	char *str_byte = pvPortMalloc(4);
	if((NULL != str) && (NULL != str_byte))
	{
		memset(str, 0, msg.len * 4);
		memset(str_byte, 0, 4);
		/* Dump data message */
		for (int i = 0; i <  msg.len; ++i)
		{
			sprintf(str_byte, "%02X ", msg.data[i]);
			strcat(str, str_byte);
			memset(str_byte, 0, 4);
		}
		SLOGF(SLOG_DEBUG, "I2C msg: %s", str);
	}
	else
	{
		SLOGF(SLOG_WARN, "%s[%d]: Malloc failed", __FUNCTION__, __LINE__);
	}
	/* Free memory, the null pointer will be check inside vPortFree */
	vPortFree(str);
	vPortFree(str_byte);
}
#endif /* LOG_VERBOSE == STD_ON */

/**
 * cb_rx_not_empty
 * I2C Slave receiver buffer is not empty callback function
 */
static void cb_rx_not_empty(uint8_t data)
{
    ++cmd_rx_buf_msg.len;
    if (cmd_rx_buf_msg.buf_overflow)
        return;
    uint8_t* wp_new = cmd_rx_buf.wp + 1;
    if (wp_new >= cmd_rx_buf.buf_end)
        wp_new = cmd_rx_buf.buf_start;
    if (wp_new != cmd_rx_buf.rp)
    {
        *cmd_rx_buf.wp = data;
        cmd_rx_buf.wp = wp_new;
    }
    else
    {
        cmd_rx_buf_msg.buf_overflow = true;
        printf("\r\nError: Command receive buffer overflow\r\n");
    }
}

/**
 * cmd receive call back when i2c is complete
 *
 */
static void cb_rx_completed(void)
{
    if (cmd_rx_buf_msg.buf_overflow)
    {
        printf("\r\nError: Discarding received command due to buffer overflow\r\n");
    }
    else
    {
        BaseType_t task_awoken = pdFALSE;
        if (xQueueSendFromISR(cmd_rx_msg_queue, &cmd_rx_buf_msg, &task_awoken) != pdTRUE)
        {
            printf("\r\nError in %s[%d]: Failed to push received command!\r\n", __FUNCTION__, __LINE__);
        }
        portYIELD_FROM_ISR(task_awoken);
    }
    // Initialize next received message info
    cmd_rx_buf_msg.buf_overflow = false;
    cmd_rx_buf_msg.len = 0;
    cmd_rx_buf_msg.msg_start = cmd_rx_buf.wp;
}

static void cb_tx_request(void)
{
	cmd_rx_buf_msg.buf_overflow = false;
	cmd_rx_buf_msg.len = 0;
	cmd_rx_buf_msg.msg_start = cmd_rx_buf.wp;
}

#if (ASIC_NUM == ASIC1)
void asic_command_list_init(void)
{
	/* Allocate memory */
	asic_command_list = (asic_command_waiting_list_t *)assert_malloc(
		asic_command_list, sizeof(asic_command_waiting_list_t));

	asic_command_list->head = (lcc_cmd_store_asic1_t *)assert_malloc(
		asic_command_list->head, sizeof(*asic_command_list->head));
	asic_command_list->tail = (lcc_cmd_store_asic1_t *)assert_malloc(
		asic_command_list->tail, sizeof(*asic_command_list->tail));
	asic_command_list->head->next =	asic_command_list->tail;
	asic_command_list->tail->next = asic_command_list->tail;
	asic_command_list->total = 0;
}

uint8_t push_asic_command_list(lcc_cmd_store_asic1_t msg)
{
	lcc_cmd_store_asic1_t *buf = NULL;
	lcc_cmd_store_asic1_t *ptr = NULL;
	if(asic_command_list->total > 20)
	{
		/* Delete the oldest message */
		ptr = asic_command_list->head->next;
		asic_cmd_delete_msg(ptr);
	}
	buf = (lcc_cmd_store_asic1_t *)assert_malloc(buf,
		sizeof(lcc_cmd_store_asic1_t));

	buf->m_bitmask = msg.m_bitmask;
	buf->tid = swap16(msg.tid);
	buf->cmd = msg.cmd;
	buf->ucid = msg.ucid;
	buf->cmd_idx = msg.cmd_idx;
	buf->next = asic_command_list->tail;
	ptr = asic_command_list->head;
	while (ptr->next != asic_command_list->tail) ptr = ptr->next;
	ptr->next = buf;
	/* Increment total msg is pushed */
	asic_command_list->total++;
	return 1;
}

void asic_cmd_delete_msg(lcc_cmd_store_asic1_t *in)
{
	if ((in != NULL) && asic_command_list->total)
	{
		/* find the previous node */
		lcc_cmd_store_asic1_t *prev = asic_command_list->head;

		while (prev != asic_command_list->tail && prev->next != NULL && prev->next != in)
		{
			prev = prev->next;
		}

		/* Check if node really exists in Linked List */
		if (prev->next == NULL || prev->next == asic_command_list->tail)
		{
			SLOGF(SLOG_ERROR, "%s:%d: Given node is not present in Linked List",
					__FUNCTION__, __LINE__);
			return;
		}
		/* Remove node from Linked List */
		prev->next = in->next;
		/* Free memory */
		vPortFree(in);
		asic_command_list->total--;
	}
}

lcc_cmd_store_asic1_t *asic_cmd_pop_msg(uint16_t tid)
{
	lcc_cmd_store_asic1_t *buf = NULL;
	buf = asic_command_list->head->next;
	while(buf != asic_command_list->tail)
	{
		if (buf->tid == tid)
		{
			break;
		}
		else
			buf = buf->next;
	}
	if(buf->tid != tid)
		buf = NULL;
	return buf;
}

uint8_t asic1_push_read_cmd(lcc_cmd_store_asic1_t *info)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	uint8_t tmp[2] = {0xFF, 0xEF};
	uint8_t *buf2;
	uint8_t *buf3;
	uint8_t len = 7;
	lcc_asic_data_t asic_data;
	asic_data.asic2_bitmask = info->m_bitmask & light_system->m_filter_asic1;
	asic_data.asic3_bitmask = info->m_bitmask & light_system->m_filter_asic2;

	len += info->cmd.ucid ? 2 : 0;
	if(asic_data.asic2_bitmask)
	{
		buf2 = pvPortMalloc(len);
		memcpy(&buf2[0], &tmp[0], 2);
		buf2[2] = info->cmd.cmd;
		/* Reset bit interrupt if it present */
		buf2[3] = info->cmd.base & 0x7F;
		memcpy(&buf2[4], &asic_data.asic2_bitmask, 3);
		if(info->cmd.ucid)
			memcpy(&buf2[7], &info->ucid, 2);

		status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
									I2C_FORWARD_SLAVEADDR,
									buf2, len, NULL, 0);
		vPortFree(buf2);
	}
	if(asic_data.asic3_bitmask)
	{
		buf3 = pvPortMalloc(len);
		memcpy(&buf3[0], &tmp[0], 2);
		buf3[2] = info->cmd.cmd;
		/* Reset bit interrupt if it present */
		buf3[3] = info->cmd.base & 0x7F;
		memcpy(&buf3[4], &asic_data.asic3_bitmask, 3);
		if(info->cmd.ucid)
			memcpy(&buf3[7], &info->ucid, 2);
		status |= i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
									I2C_FORWARD_SLAVEADDR,
									buf3, len, NULL, 0);
		vPortFree(buf3);
	}
	if(status != I2CM_ERROR_TRANSMITTED)
	{
		return 0;
	}
	vTaskDelay(1);
	return 1;
}

uint8_t asic1_get_intr_source(lcc_intr_src_data_t *asic2,
													lcc_intr_src_data_t *asic3)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	uint8_t buf[4] = {0xFF, 0xEF, 0x7C, 0x02};
	uint8_t back_buf[4] = {0x00, 0x00, 0x00, 0x00};
	if(asic2 != NULL)
	{
		status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
									I2C_FORWARD_SLAVEADDR, buf, 4, NULL, 0);
		vTaskDelay(1);
		status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
								I2C_FORWARD_SLAVEADDR, NULL, 0, back_buf, 4);
		if(status == I2CM_ERROR_RECEIVED)
			memcpy((uint8_t *)asic2, back_buf, sizeof(lcc_intr_src_data_t));
		else
			SLOGF(SLOG_WARN, "ASIC2 read wrong TID");
	}
	if(asic3 != NULL)
	{
		status = i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
									I2C_FORWARD_SLAVEADDR, buf, 4, NULL, 0);
		vTaskDelay(1);
		status = i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
									I2C_FORWARD_SLAVEADDR, NULL, 0,
									back_buf, 4);
		if(status == I2CM_ERROR_RECEIVED)
			memcpy((uint8_t *)asic3, back_buf, sizeof(lcc_intr_src_data_t));
		else
			SLOGF(SLOG_WARN, "ASIC3 read wrong TID");
	}
	return status;
}

uint8_t asic1_init_cache(uint8_t asic_num)
{
	lcc_cmd_store_asic1_t asic_data;
	asic_data.m_bitmask = 0;
	asic_data.tid = 0xFFEF;
	if(asic_num == ASIC_2)
		asic_data.m_bitmask = light_system->m_filter_asic1;
	else if(asic_num == ASIC_3)
		asic_data.m_bitmask = light_system->m_filter_asic2;
	/* Waiting for ASIC2/ASIC3 stable */
	vTaskDelay(100);
	/* For LEN POSITION */
	asic_data.cmd_idx = 15;
	asic_data.cmd = (lcc_cmd_info_t)lcc_cmd_list[15].cmd;
	asic1_push_read_cmd(&asic_data);
	vTaskDelay(1);
	asic1_get_data(&asic_data);
	/* For MIRROR POSITION */
	asic_data.cmd_idx = 16;
	asic_data.cmd = (lcc_cmd_info_t)lcc_cmd_list[16].cmd;
	asic1_push_read_cmd(&asic_data);
	vTaskDelay(1);
	asic1_get_data(&asic_data);
	return 1;
}

uint8_t asic1_query_asic2(uint16_t *asic2_status)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	uint8_t buf[4] = {0xFF, 0xEF, 0x15, 0x02};
	status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
							I2C_FORWARD_SLAVEADDR, buf, 4, NULL, 0);
	if(status != I2CM_ERROR_TRANSMITTED)
		return 0;
	status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
								I2C_FORWARD_SLAVEADDR, NULL, 0,
								(uint8_t *)asic2_status, 2);
	if(status != I2CM_ERROR_RECEIVED)
		return 0;
	return 1;
}

uint8_t asic1_query_asic3(uint16_t *asic3_status)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	uint8_t buf[4] = {0xFF, 0xEF, 0x15, 0x02};
	status = i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
							I2C_FORWARD_SLAVEADDR, buf, 4, NULL, 0);
	if(status != I2CM_ERROR_TRANSMITTED)
		return 0;
	status = i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
								I2C_FORWARD_SLAVEADDR, NULL, 0,
								(uint8_t *)asic3_status, 2);
	if(status != I2CM_ERROR_RECEIVED)
		return 0;
	return 1;
}

uint8_t asic1_get_eeprom(lcc_cmd_store_asic1_t *info)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	uint16_t len = 0;
	cam_typedef_t *p_cam;
	uint8_t cam_idx;
	uint32_t m_bitmask = info->m_bitmask;
	uint8_t *buf;
	uint8_t m_number = __builtin_popcount(m_bitmask);
	for(uint8_t i = 0; i < m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/* Verify object is valid */
		{
			SLOGF(SLOG_ERROR, "%s[%d] ASIC msg queue is duplicate, m_bitmask is"
			" invalid", __FUNCTION__, __LINE__);
			return 0;
		}
		else
		{
			len += p_cam->eeprom.len;
		}
	}
	if(len)
	{
		buf = pvPortMalloc(len);
		if(buf == NULL)
		{
			SLOGF(SLOG_ERROR, "%s:[%d] Malloc failed", __FUNCTION__, __LINE__);
			return 0;
		}
	}
	else
	{
		SLOGF(SLOG_ERROR, "%s[%d] EEPROM read parsing data failed",
			__FUNCTION__, __LINE__);
		return 0;
	}

	m_bitmask = info->m_bitmask;
	if(m_bitmask & light_system->m_filter_asic1)
	{
		status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
								I2C_FORWARD_SLAVEADDR, NULL, 0, buf, len);
	}
	else if(m_bitmask & light_system->m_filter_asic2)
	{
		status = i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
								I2C_FORWARD_SLAVEADDR, NULL, 0, buf, len);
	}
	else
	{
		SLOGF(SLOG_ERROR, "Invalid command queue");
	}

	if(status != I2CM_ERROR_RECEIVED)
	{
		SLOGF(SLOG_ERROR, "%s[%d] Read back eeprom failed",
			__FUNCTION__, __LINE__);
		vPortFree(buf);
		return 0;
	}
	m_number = __builtin_popcount(m_bitmask);
	uint8_t data_cnt = 0;
	for(uint8_t i = 0; i < m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/* Verify object is valid */
		{
			SLOGF(SLOG_ERROR, "Something went wrong");
			return 0;
		}
		else
		{
			memcpy(p_cam->eeprom.buf, &buf[data_cnt], p_cam->eeprom.len);
			data_cnt += p_cam->eeprom.len;
		}
	}
	vPortFree(buf);
	return 1;
}

uint8_t asic1_get_data(lcc_cmd_store_asic1_t *info)
{
	i2cm_error_t status = I2CM_ERROR_NONE;
	lcc_asic_data_t asic_data;
	asic_data.asic2_bitmask = info->m_bitmask & light_system->m_filter_asic1;
	asic_data.asic3_bitmask = info->m_bitmask & light_system->m_filter_asic2;
	asic_data.asic2_len = __builtin_popcount(asic_data.asic2_bitmask);
	asic_data.asic3_len = __builtin_popcount(asic_data.asic3_bitmask);
	asic_data.asic2_data = NULL;
	asic_data.asic3_data = NULL;

	if(asic_data.asic2_len)
	{
		asic_data.asic2_data = pvPortMalloc(asic_data.asic2_len * info->cmd.size);
		status = i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL,
									I2C_FORWARD_SLAVEADDR, NULL, 0,
			asic_data.asic2_data, asic_data.asic2_len * info->cmd.size);
	}
	if(asic_data.asic3_len)
	{
		asic_data.asic3_data = pvPortMalloc(asic_data.asic3_len * info->cmd.size);
		status |= i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL,
			I2C_FORWARD_SLAVEADDR, NULL, 0, asic_data.asic3_data,
			asic_data.asic3_len * info->cmd.size);
	}

	if(status != I2CM_ERROR_RECEIVED)
	{
		SLOGF(SLOG_WARN, "ASIC%d sync command %04X failed",
			asic_data.asic2_len ? 2 : 3, info->cmd.cmd | (info->cmd.base << 8));
	}
	else
	{
		lcc_cmd_t in, out;
		if(asic_data.asic2_len)
		{
			if((info->cmd.cmd == LCC_CMD_STREAM_STATUS_ID) &&
				(info->cmd.base == LCC_CMD_STREAM_STATUS_BASE))
			{
				cam_typedef_t *p_cam = NULL;
				uint32_t m_bitmask = asic_data.asic2_bitmask;
				uint8_t cam_idx = 0, data_idx = 0;
				for(uint8_t i = 0; __builtin_popcount(m_bitmask) > 0; i++)
				{
					cam_idx = __builtin_ctz(m_bitmask);
					m_bitmask &= ~(1 << cam_idx);
					p_cam = idx_to_object(cam_idx);
					if(p_cam)
					{
						memcpy(&p_cam->settings->stream[0],
							&asic_data.asic2_data[data_idx], info->cmd.size);
						data_idx += info->cmd.size;
					}
				}
			}
			else
			{
				in.m_bitmask = asic_data.asic2_bitmask;
				in.data = asic_data.asic2_data;
				in.ucid = info->ucid;
				in.action = CMD_WRITE;
				in.len = asic_data.asic2_len * info->cmd.size;
				in.cmd = info->cmd.cmd | info->cmd.base << 8;
				in.m_number = __builtin_popcount(in.m_bitmask);
				(*lcc_cmd_list[info->cmd_idx].func)(&in, &out);
			}
		}

		if(asic_data.asic3_len)
		{
			if((info->cmd.cmd == LCC_CMD_STREAM_STATUS_ID) &&
				(info->cmd.base == LCC_CMD_STREAM_STATUS_BASE))
			{
				cam_typedef_t *p_cam = NULL;
				uint32_t m_bitmask = asic_data.asic3_bitmask;
				uint8_t cam_idx = 0, data_idx = 0;
				for(uint8_t i = 0; __builtin_popcount(m_bitmask) > 0; i++)
				{
					cam_idx = __builtin_ctz(m_bitmask);
					m_bitmask &= ~(1 << cam_idx);
					p_cam = idx_to_object(cam_idx);
					if(p_cam)
					{
						memcpy(&p_cam->settings->stream[0],
							&asic_data.asic3_data[data_idx], info->cmd.size);
						data_idx += info->cmd.size;
					}
				}
			}
			else
			{
				in.m_bitmask = asic_data.asic3_bitmask;
				in.data = asic_data.asic3_data;
				in.ucid = info->ucid;
				in.action = CMD_WRITE;
				in.len = asic_data.asic3_len * info->cmd.size;
				in.cmd = info->cmd.cmd | info->cmd.base << 8;
				in.m_number = __builtin_popcount(in.m_bitmask);
				(*lcc_cmd_list[info->cmd_idx].func)(&in, &out);
			}
		}
	}
	/* TODO */
	vPortFree(asic_data.asic2_data);
	vPortFree(asic_data.asic3_data);
	return 1;
}
#endif
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
