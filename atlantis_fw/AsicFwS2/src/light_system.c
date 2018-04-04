/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    light_system.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Juk-16-2016
 * @brief   This file contains expand for Light system object
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "assert.h"
#include "log.h"
#include "os.h"
#include "board_config.h"
#include "lcc_system.h"
#include "lcc_cmd.h"
#include "optical.h"
#include "task_cam_ctrl.h"
#include "task_ccb_ctrl.h"
#include "task_af_ctrl.h"
#include "light_system.h"
#include "actuator.h"
#include "flash.h"
#include "lcc_cmd.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_LIGHT_SYSTEM
#define CAM_A_QUANTITY			5
#define CAM_B_QUANTITY			5
#define CAM_C_QUANTITY			6
#define CALIB_DATA_OFFSET           (0x110000)
#define CALIB_DATA_SIZE_OFFSET		(0x140000)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/
/* The camera lookup table with CAMERA_TBL was predefined in lookup_tbl.mk.
 * This table can edit depend on platform or design
 */
const uint16_t lookup_tbl[] = CAMERA_TBL;
const uint8_t modules_tbl[] =
						{	0x00,								/* Global */
							0xA1, 0xA2, 0xA3, 0xA4, 0xA5,		/* Group A */
							0xB1, 0xB2, 0xB3, 0xB4, 0xB5,		/* Group B */
							0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6	/* Group C */
						};
const uint8_t modules_tlb_size = ARRAY_SIZE(modules_tbl);
/*
 * Light System
 */
static light_ccb_t light_sys;
#if(ASIC_NUM == ASIC1)
static hal_gpio_t intr;
static EventGroupHandle_t intr_sync_event;
static xSemaphoreHandle intr_asic2_sync_sem;
static xSemaphoreHandle intr_asic3_sync_sem;
#endif
static QueueHandle_t intr_src_queue = NULL;
static SemaphoreHandle_t intr_semaphore = NULL;
/* Exported Global variables--------------------------------------------------*/
/* Light System #light_system */
light_ccb_t * const light_system = &light_sys;
#if(ASIC_NUM == ASIC1)
/* sync queue to hold the messages from asic forward to asic2 and asic 3*/
asic_sync_t *asic_sync_queue = NULL;
#endif
/* Private function-----------------------------------------------------------*/
static void intr_queue_init(uint8_t queue_size);
static void intr_init(void);
static void set_orientation(cam_typedef_t *pcam)
{
	switch(pcam->info.module)
	{

	case CAM_B4:
	case CAM_B3:
	case CAM_B2:
	case CAM_C2:
	case CAM_C4:
	case CAM_C5:
	case CAM_C6:
		pcam->image->flip = FLIP_VERTICAL;
		break;
	case CAM_B5:
	case CAM_B1:
	case CAM_C1:
	case CAM_C3:
		pcam->image->flip = FLIP_HORIZONTAL;
		break;
	case CAM_A5:
		pcam->image->flip = FLIP_VERTICAL | FLIP_HORIZONTAL;
		break;
	default:
		pcam->image->flip = FLIP_NONE;
		break;
	}
}
/* Exported functions --------------------------------------------------------*/
void light_camera_init(void)
{
	cam_typedef_t *pcam;
	uint16_t *p_focal_length;
	uint8_t i, j, ch;
	flash_init();
	for (i = 0; i < light_system->cam_tbl_size; i++)
	{
		pcam = &light_system->cam_tbl[i];
		/* Image sensor memory allocate */
		pcam->image = assert_malloc(pcam->image, sizeof(img_sensor_t));
		/* Lens optical system memory allocate */
		pcam->optical = assert_malloc(pcam->optical, sizeof(optical_t));

		pcam->settings = assert_malloc(pcam->settings, sizeof(cam_data_t));
		pcam->optical->settings = assert_malloc(pcam->optical->settings,
															sizeof(opt_data_t));
		p_focal_length = (uint16_t *)pcam->optical->settings->focal_length;

		/* Allocate camera data cache for each of UCID */
		for (j = 0; j < UC_MAX; j++)
		{
			pcam->image->settings[j] =
				assert_malloc(pcam->image->settings[j], sizeof(img_data_t));
		}

		/* Configure the Orientation */
		set_orientation(pcam);

		/* Create the semaphore to access settings */
		pcam->semaphore = xSemaphoreCreateMutex();
		pcam->image->semaphore = xSemaphoreCreateMutex();
		pcam->optical->semaphore = xSemaphoreCreateMutex();
		assert_param(pcam->semaphore);
		assert_param(pcam->image->semaphore);
		assert_param(pcam->optical->semaphore);

		/* Identify image sensor interface, just only for camera module manage
		 * by this ASIC */
		if (0 < pcam->info.ch)
		{
			ch = pcam->info.ch - 1;
			/* Assign I2C channel for Sensor Image, EEPROM  */
			pcam->image->i2c_dev = I2C_CH0 + ch;

			/* Assign GPIO Control for CAM_STANDBY_L, CAM_PCLK, pins */
			pcam->image->standby = GPIO_PORT_PIN(GPIO_PORTB, GPIO_PIN_1 + ch);
			if(pcam->info.grp == GRP_A)
			{
				/* Initialize cam module type value for CAM 28MM */
				pcam->settings->type = CAM_TYPE_28MM;
				/* Assign value of focal length */
				*p_focal_length = CAM_28MM;

				/* Allocate system memory for optical VCM */
				pcam->optical->lens = assert_malloc(pcam->optical->lens,
													sizeof(vcm_t));
				vcm_t *plens = (vcm_t *)pcam->optical->lens;

				/* VCM share I2C channel with Sensor Image, epprom */
				plens->i2c = I2C_CH0 + ch;
				/* Some default settings */
				plens->af_data = NULL;
				plens->af_calib = NULL;
				/* Create the semaphore to access settings */
				plens->semaphore = xSemaphoreCreateMutex();
				assert_param(plens->semaphore);
			}
			else /* GRP_B and GRP_C */
			{
				/* Initialize cam module type value for CAM 70MM / CAM 150MM */
				pcam->settings->type = (pcam->info.grp == GRP_B) ?
										CAM_TYPE_70MM : CAM_TYPE_150MM;
				/* Assign value of focal length */
				*p_focal_length = (pcam->info.grp == GRP_B) ?
										CAM_70MM : CAM_150MM;

				/* Allocate system memory for optical lens */
				pcam->optical->lens = assert_malloc(pcam->optical->lens,
														sizeof(actuator_t));
				actuator_t *plens = (actuator_t *)pcam->optical->lens;

				/* Assign parameters for lens */
				plens->cam_info = pcam->info;
				plens->type = ACTUATOR_LENS;
				plens->hall_i2c = I2C_CH6 + ch;
				plens->eeprom_i2c = I2C_CH0 + ch;
				plens->pwm = PWM0_CH1 + ch;

				/* Attempt to create the event group. */
				plens->event = xEventGroupCreate();
				assert_param(plens->event);
				/* Create the semaphore to access settings */
				plens->semaphore = xSemaphoreCreateMutex();
				assert_param(plens->semaphore);
				plens->pwm_semaphore = xSemaphoreCreateBinary();
				assert_param(plens->pwm_semaphore);

				/* Reset status of lens */
				plens->is_initialized = FALSE;
				plens->is_calibrated = FALSE;

				/* Initializes AF control pin */
				hal_gpio_t af_enable;
				af_enable.port = GPIO_PORTC;
				if(pcam->info.module == CAM_B2)
					af_enable.pin = GPIO_PIN_4 + 3;
				else
					af_enable.pin = GPIO_PIN_4 + ch;
				af_enable.direction = GPIO_DIR_OUT;
				hal_gpio_init(&af_enable);
				hal_gpio_set_high(&af_enable);

				if(( pcam->info.module == CAM_B4) ||
					(pcam->info.module == CAM_C5) ||
					(pcam->info.module == CAM_C6))
					continue;
				/* Allocate system memory for optical mirror */
				pcam->optical->mirr = assert_malloc(pcam->optical->mirr,
															sizeof(actuator_t));
				actuator_t *mirr = (actuator_t *)pcam->optical->mirr;

				/* Assign parameters for mirror */
				mirr->cam_info = pcam->info;
				mirr->type = ACTUATOR_MIRROR;
				mirr->hall_i2c = I2C_CH12 + ch;
				mirr->eeprom_i2c = I2C_CH0 + ch;
				mirr->pwm = PWM1_CH1 + ch;

				/* Attempt to create the event group. */
				mirr->event = xEventGroupCreate();
				assert_param(mirr->event);
				/* Create the semaphore to access settings */
				mirr->semaphore = xSemaphoreCreateMutex();
				assert_param(mirr->semaphore);
				mirr->pwm_semaphore = xSemaphoreCreateBinary();
				assert_param(mirr->pwm_semaphore);


				/* Reset status of mirror */
				mirr->is_initialized = FALSE;
				mirr->is_calibrated = FALSE;
			}

			/* Assign SyncIO channel */
			pcam->sync = SG_CHANNEL_0 + ch;
		}
	}

	/* Initialize all camera power */
	hal_gpio_t power_en_gpio;
	power_en_gpio.pin = GPIO_PIN(light_system->cam_power_en);
	power_en_gpio.port = GPIO_PORT(light_system->cam_power_en);
	power_en_gpio.direction = GPIO_DIR_OUT;
	hal_gpio_init(&power_en_gpio);
	hal_gpio_set_high(&power_en_gpio);

	/* Enable power for actuators */
#if(ASIC_NUM == ASIC1)
	*((__IO uint32_t *)0x020000A8) |= 0x00000600;
#endif
	hal_gpio_t af_enable;
	af_enable.port = GPIO_PORTC;
	af_enable.direction = GPIO_DIR_OUT;
	/* Enable piezo power regulator 1*/
	af_enable.pin = GPIO_PIN_12;
	hal_gpio_init(&af_enable);
	hal_gpio_set_high(&af_enable);
	/* Enable piezo power regulator 2*/
	af_enable.pin = GPIO_PIN_13;
	hal_gpio_init(&af_enable);
	hal_gpio_set_high(&af_enable);

	hal_pwm_set_high(PWM0_CH5, PWM_PIN_POS);
	hal_pwm_set_high(PWM0_CH5, PWM_PIN_NEG);
	/* Init interrupt */
	intr_init();
}

void light_camera_task_create(void)
{
	BaseType_t ret = pdTRUE;

	/* Add task to os kernel */
	for (uint8_t i = 0; i < TASK_NUM; i++)
	{
		task_handler[i].idx = i;
		task_handler[i].state = TASK_INITIALIZE;
		task_handler[i].time_sleep = task_list[i].time_sleep;
		ret &= xTaskCreate(
							task_list[i].task,		/* task function */
							task_list[i].name,		/* task name */
							task_list[i].stacksize,	/* stack size */
							&task_handler[i],		/* passing parameters */
							task_list[i].prio,		/* task priority */
							&task_handler[i].handle	/* handle */
							);
		/* Assert the creation task return to make sure all of
			camera task was created */
		assert_param(pdPASS == ret);
	}

	/* Camera task creation */
	task_cam_create();
	/* Actuator task creation for LENS & MIRROR piezo control */
	task_actuator_create();

}

void light_system_init(void)
{
	cam_t info;
	uint8_t i, j, idx;
	/* Scan module table to detect Camera module self manage and sub ASIC's
	 * camera module, the camera table size depend on ASIC */
	light_system->cam_tbl_size = 0;
	light_system->m_bitmask_all = 0;
	light_system->m_filter = 0;
	light_system->m_filter_asic1 = 0;
	light_system->m_filter_asic2 = 0;
#if (ASIC_NUM == ASIC1)
    light_system->asic1_read_event = xEventGroupCreate();
	light_system->asic2_intr = GPIO_PORT_PIN(GPIO_PORTA, GPIO_PIN_6);
	light_system->asic3_intr = GPIO_PORT_PIN(GPIO_PORTA, GPIO_PIN_7);
	light_system->cb_asic2_intr = asic2_intr_handler;
	light_system->cb_asic3_intr = asic3_intr_handler;
	/* Init input interrupt signal for ASIC2 & ASIC3 */
	hal_gpio_t intr;
	hal_gpio_exti_t gpio_intr;
	gpio_intr.int_type = GPIO_EXTI_RISING_EDGE;
	intr.direction = GPIO_DIR_IN;
	intr.port = GPIO_PORTA;
	/* To be trigged from asic 2*/
	intr.pin = light_system->asic2_intr;
	gpio_intr.pin = light_system->asic2_intr;
	gpio_intr.irq_handler = light_system->cb_asic2_intr;
	hal_gpio_init(&intr);
	hal_gpio_enable_exti(&gpio_intr);
	/* To be trigged from asic 3*/
	intr.pin = light_system->asic3_intr;
	gpio_intr.pin = light_system->asic3_intr;
	gpio_intr.irq_handler = light_system->cb_asic3_intr;
	hal_gpio_init(&intr);
	hal_gpio_enable_exti(&gpio_intr);
	/* Create event group for interrupt signal*/
	intr_sync_event = xEventGroupCreate();
	intr_asic2_sync_sem = xSemaphoreCreateMutex();
	intr_asic3_sync_sem = xSemaphoreCreateMutex();
#endif
	/* Get camera module table size */
	for (i = 0; i < CAMERA_MAX(); i++)
	{
		if (lookup_tbl[i] != 0x0000)
			light_system->cam_tbl_size++;
	}

	/* Allocate memory for Light camera object */
	light_system->cam_tbl = assert_malloc(light_system->cam_tbl,
							sizeof(cam_typedef_t)*light_system->cam_tbl_size);
	j = 0;
	for (i = 0; i < CAMERA_MAX(); i++)
	{
		info.ident = lookup_tbl[i];
		if (info.ident != 0x0000)
		{
			light_system->cam_tbl[j++].info.ident = info.ident;

			/* Update camera module bit-mask */
			idx = module_to_idx(info.module);
			/* Bitmask for all camera manage by this ASIC */
			light_system->m_bitmask_all |= (1 << idx);

			/* Bitmask for all camera manage by this ASIC */
			if (info.sub_asic == 0)
				light_system->m_filter |= (1 << idx);
			/* Bitmask for all camera manage by sub ASIC1 */
			else if (info.sub_asic == 1)
				light_system->m_filter_asic1 |= (1 << idx);
			/* Bitmask for all camera manage by sub ASIC2 */
			else if (info.sub_asic == 2)
				light_system->m_filter_asic2 |= (1 << idx);
		}
	}

	/* Allocate memory for Light settings object */
	light_system->settings = assert_malloc(light_system->settings,
								sizeof(ccb_data_t));

	/* Init the log ctrl value */
	light_system->settings->log_ctrl = LOG_VERBOSE_DEFAULT;

	/* Init value of ASIC Light Protocol */
	light_system->settings->light_protocol_version[0] = ASIC_FW_VERSION_MAJOR;
	light_system->settings->light_protocol_version[1] = ASIC_FW_VERSION_MINOR;

	/* Init value of Firmware Version */
	/* uint32_t tmp[2] = {ASIC_FW_VERSION_MAJOR, ASIC_FW_VERSION_MINOR}; */
	strcpy((char *)light_system->settings->fw_version, ASIC_FW_VERSION);

	/* Init value of Calib Version */
	light_system->settings->light_calib_version[0] = ASIC_CALIB_VERSION_MAJOR;
	light_system->settings->light_calib_version[1] = ASIC_CALIB_VERSION_MINOR;

	light_system->cam_power_en = GPIO_PORT_PIN(GPIO_PORTB, GPIO_PIN_0);

    /* Init values for auto-focus */
	light_system->settings->zoom_factor = 1.0f;
	light_system->af_ctrl = assert_malloc(light_system->af_ctrl, sizeof(af_ctrl_t));
    task_af_create();
}

/*
 * Light main program
 */
void light_main(void)
{
	/* Light camera control board initialization */
	light_system_init();
	/* Camera initialization */
	light_camera_init();
	light_camera_task_create();
	/* Initialize command processing */
    lcc_cmd_init();
    /* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();
}

uint32_t object_to_cam_bitmask(cam_typedef_t *pcam)
{
	uint32_t cam_bitmask = 0;
	if(pcam != NULL)
	{
		switch(pcam->info.grp)
		{
			case 0x0A:
				cam_bitmask = 1 << (pcam->info.idx + 0);
				break;
			case 0x0B:
				cam_bitmask = 1 << (pcam->info.idx + CAM_A_QUANTITY);
				break;
			case 0x0C:
				cam_bitmask = 1 << (pcam->info.idx + CAM_A_QUANTITY +
                                    CAM_B_QUANTITY);
				break;
		}
	}
	return cam_bitmask;
}

uint8_t module_to_idx(uint8_t module)
{
	uint8_t idx = ARRAY_SIZE(modules_tbl);
	while ((--idx > 0) && (module != modules_tbl[idx]));
	return idx;
}

uint8_t idx_to_module(uint8_t idx)
{
	return modules_tbl[idx];
}

cam_typedef_t *idx_to_object(uint8_t idx)
{
	cam_typedef_t *pcam;

	for (uint8_t i = 0; i < light_system->cam_tbl_size; i++)
	{
		pcam = &light_system->cam_tbl[i];
		if (pcam->info.module == modules_tbl[idx])
			return pcam;
	}
	SLOGF(SLOG_ERROR, "Cannot find this module");
	return NULL;
}

cam_typedef_t *chan_to_object(uint8_t ch)
{
    cam_typedef_t *pcam;

    for (uint8_t i = 0; i < light_system->cam_tbl_size; i++)
    {
        pcam = &light_system->cam_tbl[i];
        if (pcam->info.ch == ch)
            return pcam;
    }
    SLOGF(SLOG_ERROR, "Cannot find this module");
    return NULL;
}

#if (ASIC_NUM == ASIC1)
void asic2_intr_handler(void)
{
	if(light_system->wait_intr_stream & ASIC_2)
	{
		light_system->wait_intr_stream &= ~ASIC_2;
		xEventGroupSetBitsFromISR(intr_sync_event,
								ASIC_2,
								NULL);
	}
	else
	{
		light_system->asic_intr_queue_num.asic2_intr_num++;
		xEventGroupSetBitsFromISR(light_system->asic1_read_event,
							ASIC_READ_ASIC_2, NULL);
	}

}

void asic3_intr_handler(void)
{
	if(light_system->wait_intr_stream & ASIC_3)
	{
		light_system->wait_intr_stream &= ~ASIC_3;
		xEventGroupSetBitsFromISR(intr_sync_event,
							ASIC_3,
							NULL);
	}
	else
	{
		light_system->asic_intr_queue_num.asic3_intr_num++;
		xEventGroupSetBitsFromISR(light_system->asic1_read_event,
							ASIC_READ_ASIC_3, NULL);
	}
}

EventBits_t wait_intr_signal(asic_id_t asic_id_msk, uint32_t timeout, uint8_t waitall)
{
	uint8_t asic2_is_used = FALSE;
	uint8_t asic3_is_used = FALSE;
	EventBits_t ux_bits;
	/* Reset interrupt mask */
	light_system->asic_intr_mask = 0;
	if(asic_id_msk == 0)
	{
		SLOGF(SLOG_INFO, "Only capture on ASIC1 ");
		return asic_id_msk;
	}
	if(asic_id_msk & ASIC_2)
	{
		/* Waiting for interrupt signal semaphore of aisc2 is released in 1s*/
		if(xSemaphoreTake(intr_asic2_sync_sem, 1000) != pdPASS)
		{
			SLOGF(SLOG_ERROR, "Interrupt signal from ASIC2 is busy");
			asic2_is_used = TRUE;
		}
	}
	/* Waiting for interrupt signal semaphore of aisc3 is released in 1s*/
	if(asic_id_msk & ASIC_3)
	{
		if(xSemaphoreTake(intr_asic3_sync_sem, 1000) != pdPASS)
		{
			SLOGF(SLOG_ERROR, "Interrupt signal from ASIC3 is busy");
			asic3_is_used = TRUE;
		}
	}
	if(waitall && (asic2_is_used || asic3_is_used))
	{
		return FALSE;
	}
	/* Waiting for the interrupt Flags are updated in timeout ms*/
	ux_bits = xEventGroupWaitBits(
								intr_sync_event,
								asic_id_msk,
								pdTRUE,
								waitall ? pdTRUE : pdFALSE,
								timeout);
	/* Release interrupt signal semaphore of asic2*/
	if(asic_id_msk & ASIC_2)
		xSemaphoreGive(intr_asic2_sync_sem);
	/* Release interrupt signal semaphore of asic3*/
	if(asic_id_msk & ASIC_3)
		xSemaphoreGive(intr_asic3_sync_sem);
	return ux_bits;
}
#endif

void send_intr_signal(void)
{
	vTaskSuspendAll();
	unsigned int i = 533;
	hal_pwm_set_high(PWM0_CH4, PWM_PIN_POS);
	while(i)
		i--;
	hal_pwm_set_low(PWM0_CH4, PWM_PIN_POS);
	xTaskResumeAll();
}

void intr_init(void)
{
#if(ASIC_NUM == ASIC1)
	intr.port = GPIO_PORTA;
	intr.direction = GPIO_DIR_OUT;
	intr.pin = GPIO_PIN_8;
	hal_gpio_init(&intr);
#endif
	/* Create mutex to protect shared resource */
	intr_semaphore = xSemaphoreCreateMutex();

	if( intr_semaphore == NULL )
	{
		SLOGF(SLOG_ERROR, "Failed to create mutex");
	}

	intr_queue_init(INTR_QUEUE_SIZE);
}

void intr_signal(void)
{
	if (xSemaphoreTake(intr_semaphore, (TickType_t)10) == pdTRUE)
	{
		uint8_t i = 5;
#if(ASIC_NUM == ASIC1)
		hal_gpio_set_high(&intr);
		while(i)
			i--;
		hal_gpio_set_low(&intr);
#else /* ASIC2 - 3 */
		/* PWM_A_LENS[4]*/
		hal_pwm_set_high(PWM0_CH4, PWM_PIN_POS);
		while(i)
			i--;
		hal_pwm_set_low(PWM0_CH4, PWM_PIN_POS);
#endif
		/* Release resource */
		xSemaphoreGive(intr_semaphore);
	}
}

void set_intr_pin(uint8_t data)
{
	if(data)
		hal_pwm_set_high(PWM0_CH4, PWM_PIN_POS);
	else
		hal_pwm_set_low(PWM0_CH4, PWM_PIN_POS);
}

void disable_asic_sync_intr(uint8_t asic_mask)
{
	if(asic_mask & ASIC_2)
	{

	}
	if(asic_mask & ASIC_3)
	{

	}
}
void enable_asic_sync_intr(uint8_t asic_mask)
{
	if(asic_mask & ASIC_2)
	{

	}
	if(asic_mask & ASIC_3)
	{

	}
}

#define POT_FULL_SCALE		10000
#define POT_SCALE_STEP		128
typedef union
{
	uint8_t data;
	struct
	{
		uint8_t res_low : 3;
		uint8_t res_high : 4;
		uint8_t sign : 1;
	};
} res_tolerance_t;
/**
 * @brief pzt_voltage_config
 * This API will config the piezo suply voltage
 * @param
 * 	pzt: Selct POT for PZT1 or PZT2
 *	vol: voltage need to config
 *	write_eeprom: Select option to write the configuration to eeprom
 * @return: 0-fail / 1-success
 */
uint8_t pzt_voltage_config(pzt_pot_sel_t pzt, float vol, uint8_t write_eeprom)
{
	uint8_t ret = 0;
	uint8_t reg = 0;
	uint16_t r2_val = 0;
	uint16_t actual_res = 0;
	uint8_t i2c_addr;

	i2cm.init(I2C_CH10);
	if(vol > 30 || vol < 12)
	{
		SLOGF(SLOG_ERROR, "Invalid param %f", vol);
		return ret;
	}
	if(pzt == PZT_POT_1)
		i2c_addr = 0x5F >> 1;
	else
		i2c_addr = 0x59 >> 1;
	/* Calculate R2 */
	r2_val = ((1.299 * 133000) / (vol - 1.299)) - 5900;
	if (I2CM_ERROR_TRANSCEIVED == i2cm.read(I2C_CH10, BYTE_ADDR16, i2c_addr,
		0x0601, (uint8_t *)&reg))
	{
		/* Calculate the actual full scale resistor */
		res_tolerance_t res_tol;
		res_tol.data = reg;
		actual_res = POT_FULL_SCALE + ((reg & 0x80 ? 1 : -1) *
			(POT_FULL_SCALE * ((float)(((float)res_tol.res_low * 0.125) +
			res_tol.res_high) / 100)));
		/* Write config value */
		reg = (uint8_t)((POT_SCALE_STEP * r2_val) / actual_res);
		if(I2CM_ERROR_TRANSMITTED == i2cm.write(I2C_CH10, BYTE_ADDR8, i2c_addr,
														0x02, (uint8_t *)&reg))
		{
			if(write_eeprom)
			{
				reg = 0x00;
				if(I2CM_ERROR_TRANSMITTED == i2cm.write(I2C_CH10, BYTE_ADDR8,
											i2c_addr, 0x01, (uint8_t *)&reg))
				{
					ret = 1;
					SLOGF(SLOG_INFO, "Write EEPROM POT DONE");
				}
			}
			else
				ret = 1;
			SLOGF(SLOG_INFO, "Write POT DONE");
		}
		else
		{
			ret = 0;
			SLOGF(SLOG_ERROR, "Write I2C failed");
		}
	}
	else
	{
		ret = 0;
		SLOGF(SLOG_ERROR, "APOT1: reading timed out");
	}
	return ret;
}

/**
 * @brief cam_move_to_distance
 * This API will move a lens/VCM to a distance
 */
cam_return_t cam_move_to_distance(cam_typedef_t *pcam, uint32_t distance,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid)
{
	lcc_cmd_tid_t p_cmd;
	p_cmd.cmd_tid = cmd_tid;
	/* check parameter validation */
	if (pcam == NULL)
		return CAM_INVALID_ARG;
	else
	{
		if(pcam->info.grp == GRP_A)
		{
			vcm_t *vcm = pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
			if (vcm->moving == FALSE)
			{
				/* set moving status */
				vcm->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->focus_distance,
						&distance, sizeof(uint32_t));
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				vcm->cb_func = cb_func;
				vcm->cb_param = cb_param;
				p_cmd.event = IMG_EVENT_VCM_DISTANCE;
				xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : VCM is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}
		else
		{
			actuator_t *plens = (actuator_t *)pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(plens->semaphore, portMAX_DELAY);
			if (plens->moving == FALSE)
			{
				/* set moving status */
				plens->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->focus_distance,
						&distance, sizeof(uint32_t));
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				plens->cb_func = cb_func;
				plens->cb_param = cb_param;

				p_cmd.event = OPT_EVENT_FOCUS_DISTANCE;
				xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : Lens is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}

		return CAM_OK;
	}
}

/**
 * @brief cam_move_lens_to_position
 * This API will move a lens/VCM to a position
 */
cam_return_t cam_move_lens_to_position(cam_typedef_t *pcam, uint16_t position,
                    uint16_t tolerance,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid)
{
	lcc_cmd_tid_t p_cmd;
	p_cmd.cmd_tid = cmd_tid;
	/* check parameter validation */
	if (pcam == NULL)
		return CAM_INVALID_ARG;
	else
	{
		if(pcam->info.grp == GRP_A)
		{
			vcm_t *vcm = pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
			if (vcm->moving == FALSE)
			{
				/* set moving status */
				vcm->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->lens_position,
						&position, sizeof(uint16_t));
				pcam->optical->settings->lens_position_tolerance = tolerance;
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				vcm->cb_func = cb_func;
				vcm->cb_param = cb_param;
				p_cmd.event = IMG_EVENT_VCM_POSITION;
				xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : VCM is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}
		else
		{
			actuator_t *plens = (actuator_t *)pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(plens->semaphore, portMAX_DELAY);
			if (plens->moving == FALSE)
			{
				/* set moving status */
				plens->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->lens_position,
						&position, sizeof(uint16_t));
                pcam->optical->settings->lens_position_tolerance = tolerance;
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				plens->cb_func = cb_func;
				plens->cb_param = cb_param;

				p_cmd.event = OPT_EVENT_LENS_POSITION;
				xQueueSend(plens->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : Lens is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}

		return CAM_OK;
	}
}

/**
 * @brief cam_move_mirr_to_position
 * This API will move a mirror to a position
 */
cam_return_t cam_move_mirr_to_position(cam_typedef_t *pcam, uint16_t position,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid)
{
	lcc_cmd_tid_t p_cmd;
	p_cmd.cmd_tid = cmd_tid;
	/* check parameter validation */
	if ((pcam == NULL) || (pcam->info.grp == GRP_A))
		return CAM_INVALID_ARG;
	else
	{
		actuator_t *pmirr = (actuator_t *)pcam->optical->mirr;
		/* take semaphore to access settings memory */
		xSemaphoreTake(pmirr->semaphore, portMAX_DELAY);
		if (pmirr->moving == FALSE)
		{
			/* set moving status */
			pmirr->moving = TRUE;
			/* release semaphore after accessing data */
			xSemaphoreGive(pmirr->semaphore);
			/* take semaphore to access settings memory */
			xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
			memcpy(pcam->optical->settings->mirr_position,
					&position, sizeof(uint16_t));
			/* release semaphore after writing data */
			xSemaphoreGive(pcam->optical->semaphore);

			pmirr->cb_func = cb_func;
			pmirr->cb_param = cb_param;

			p_cmd.event = OPT_EVENT_MIRROR_POSITION;
			xQueueSend(pmirr->queue, &p_cmd, (TickType_t)10);
		}
		else
		{
			/* release semaphore after accessing data */
			xSemaphoreGive(pmirr->semaphore);
			SLOGF(SLOG_ERROR, "CAM-%X : Mirror is moving", pcam->info.module);
			return CAM_BUSY;
		}

		return CAM_OK;
	}
}

/**
 * @brief cam_nudge_lens
 * This API will nudges a lens/VCM in the desired direction
 */
cam_return_t cam_nudge_lens(cam_typedef_t *pcam, cam_dir_t direction,
uint16_t multiplier, void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid)
{
	lcc_cmd_tid_t p_cmd;
	p_cmd.cmd_tid = cmd_tid;
	/* check parameter validation */
	if ((pcam == NULL) ||
		((direction != CAM_DIR_RETRACT_NARROW) &&
		(direction != CAM_DIR_EXTEND_WIDE)))
		return CAM_INVALID_ARG;
	else
	{
		uint32_t lens_nudge = (multiplier << 8) | direction;
		if(pcam->info.grp == GRP_A)
		{
			vcm_t *vcm = pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
			if (vcm->moving == FALSE)
			{
				/* set moving status */
				vcm->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->lens_nudge, &lens_nudge, 3);
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				vcm->cb_func = cb_func;
				vcm->cb_param = cb_param;
				p_cmd.event = IMG_EVENT_VCM_NUDGE;
				xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(vcm->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : VCM is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}
		else
		{
			actuator_t *plens = (actuator_t *)pcam->optical->lens;
			/* take semaphore to access settings memory */
			xSemaphoreTake(plens->semaphore, portMAX_DELAY);
			if (plens->moving == FALSE)
			{
				/* set moving status */
				plens->moving = TRUE;
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				/* take semaphore to access settings memory */
				xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
				memcpy(pcam->optical->settings->lens_nudge, &lens_nudge, 3);
				/* release semaphore after writing data */
				xSemaphoreGive(pcam->optical->semaphore);

				plens->cb_func = cb_func;
				plens->cb_param = cb_param;

				p_cmd.event = OPT_EVENT_LENS_NUDGE;
				xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
			}
			else
			{
				/* release semaphore after accessing data */
				xSemaphoreGive(plens->semaphore);
				SLOGF(SLOG_ERROR, "CAM-%X : Lens is moving", pcam->info.module);
				return CAM_BUSY;
			}
		}

		return CAM_OK;
	}
}

/**
 * @brief cam_nudge_mirr
 * This API will nudges a mirror in the desired direction
 */
cam_return_t cam_nudge_mirr(cam_typedef_t *pcam, cam_dir_t direction,
uint16_t multiplier, void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid)
{
	lcc_cmd_tid_t p_cmd;
	p_cmd.cmd_tid = cmd_tid;
	/* check parameter validation */
	if ((pcam == NULL) || (pcam->info.grp == GRP_A) ||
		((direction != CAM_DIR_RETRACT_NARROW) &&
		(direction != CAM_DIR_EXTEND_WIDE)))
		return CAM_INVALID_ARG;
	else
	{
		uint32_t mirr_nudge = (multiplier << 8) | direction;
		actuator_t *pmirr = (actuator_t *)pcam->optical->mirr;
		/* take semaphore to access settings memory */
		xSemaphoreTake(pmirr->semaphore, portMAX_DELAY);
		if (pmirr->moving == FALSE)
		{
			/* set moving status */
			pmirr->moving = TRUE;
			/* release semaphore after accessing data */
			xSemaphoreGive(pmirr->semaphore);
			/* take semaphore to access settings memory */
			xSemaphoreTake(pcam->optical->semaphore, portMAX_DELAY);
			memcpy(pcam->optical->settings->mirr_nudge, &mirr_nudge, 3);
			/* release semaphore after writing data */
			xSemaphoreGive(pcam->optical->semaphore);

			pmirr->cb_func = cb_func;
			pmirr->cb_param = cb_param;
			p_cmd.event = OPT_EVENT_MIRROR_NUDGE;
			xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
		}
		else
		{
			/* release semaphore after accessing data */
			xSemaphoreGive(pmirr->semaphore);
			SLOGF(SLOG_ERROR, "CAM-%X : Mirror is moving", pcam->info.module);
			return CAM_BUSY;
		}

		return CAM_OK;
	}
}

static void intr_queue_init(uint8_t queue_size)
{
	/* Create a queue to store intr_src information */
	intr_src_queue = xQueueCreate(queue_size, sizeof(intr_src_t));

	if(intr_src_queue == NULL)
	{
		/* Queue was not created and must not be used. */
	}
}

int intr_queue_push(uint16_t cmd_tid, uint16_t status)
{
	uint8_t timeout = 50;
	intr_src_t p_intr_src_st;
	p_intr_src_st.cmd_tid = cmd_tid;
	p_intr_src_st.status = status;
	/* Push the p_intr_src_st into intr_src_queue to be pop
	whenever there is a request from host*/
	while (xQueueSend(intr_src_queue, &p_intr_src_st, (TickType_t)10) != pdTRUE)
	{
		/* The queue is full */
		intr_src_t tmp;
		xQueueReceive(intr_src_queue, &tmp, (TickType_t)10);
		if (--timeout == 0)
		{
			SLOGF(SLOG_DEBUG, "intr_src_queue full\r\n");
			return -1;
		}
	}
//	SLOGF(SLOG_INFO, "Sending INTR for TID: 0x%04x [%d]", cmd_tid, status);
	intr_signal();
	return 1;
}

int intr_queue_pop(uint8_t *intr_src_data)
{
	intr_src_t p_intr_src_st;
	if(xQueueReceive(intr_src_queue, &p_intr_src_st, (TickType_t)10) != pdTRUE)
	{
		SLOGF(SLOG_DEBUG, "intr_src_queue empty");
		/* If the queue is empty then return dummy data */
		memset(intr_src_data, 0xFF, INTR_DATA_SIZE);
		return -1;
	}
	intr_src_data[0] = p_intr_src_st.cmd_tid >> 8;
	intr_src_data[1] = p_intr_src_st.cmd_tid & 0xFF;
	intr_src_data[2] = p_intr_src_st.status & 0xFF;
	intr_src_data[3] = p_intr_src_st.status >> 8;

#if (ASIC_NUM == ASIC1)
	/* Delete log queue */
	struct lcc_cmd_log_t *del = lcc_cmd_log_pull(p_intr_src_st.cmd_tid);
	if(del != NULL)
		lcc_cmd_log_delete_tid(del);
#endif
	return 1;
}
void intr_queue_reset(void)
{
	xQueueReset(intr_src_queue);
	SLOGF(SLOG_DEBUG, "Reset intr_src_queue");
}
void send_hw_sync_trigger()
{
	unsigned int i = 533;
	vTaskSuspendAll();
	hal_pwm_set_low(PWM1_CH4, PWM_PIN_NEG);
	while(i)
		i--;
	hal_pwm_set_high(PWM1_CH4, PWM_PIN_NEG);
	xTaskResumeAll();
}

uint32_t get_calib_size(void)
{
	uint32_t size = 0;
	uint8_t buf[4];
	uint8_t len = ARRAY_SIZE(buf);
	/* Read data from spi flash */
	flash_read(CALIB_DATA_SIZE_OFFSET, len, buf, SINGLE_MODE);
	for(uint8_t i = 0; i < len; i++)
	{
		size |= (uint32_t)((uint32_t)(buf[i]) << (8 *(len - i - 1)));
	}

	return size;
}

void copy_calib_data(uint32_t offset, uint8_t *dst, uint32_t size)
{
	/* Read data from spi flash */
	flash_read(CALIB_DATA_OFFSET + offset, size, dst, SINGLE_MODE);
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
