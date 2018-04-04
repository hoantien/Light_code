/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    usecase.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-13-2016
 * @brief   This file contain the real implementation for each UCID handler.
 *
 ******************************************************************************/
#include "cortex_r4.h"
#include "std_type.h"
#include "assert.h"
#include "ar1335.h"
#include "hal_axi2mipi.h"
#include "hal_mipi2axi.h"
#include "hal_syncio.h"
#include "hal_pwm.h"
#include "timer.h"
#include "log.h"
#include "light_system.h"
#include "task_cam_ctrl.h"
#include "usecase.h"
#include "usecase_hires.h"
#include "light_header.h"

#define SHIFT_TO_BIT14			14
#define SHIFT_TO_BIT12			12
#define SCU_SOFT_RESET_REG		0x40
#define ISR_SELECT_ALL			(uint32_t)(0xFFFFFFFF)

#define SLOGF_ID				SLOG_ID_LCC_CMD_BASE_0000
#define SPG_TO_IT				(SPG_SECOND / (2 * MHZ_TO_HZ))

#define VC_ID_MASK		0x0f
#define CSI_MASK		0xf0

#define LLPCLK_GRP_AB_RATE				((float)(70 / 28))
#define LLPCLK_GRP_BC_RATE				((float)(150 / 70))
#define LLPCLK_GRP_C_RATE				1
#define LLPCLK_GRP_RATE_DEFAULT			1

extern void task_tx_transfer(void *params);
extern volatile uint8_t tx_transfer_done;
extern volatile uint8_t lh_populate_done;
#if (ASIC_NUM == ASIC1)
extern TaskHandle_t task_asic_read_handle;
#endif

unsigned int prev_tick = 0;
SemaphoreHandle_t preview_sem = NULL;
uint8_t tx_end_flag = 0; /* TX0 - BIT[0-3]: VC0 - VC3; TX1 - BIT[4-7]: VC0 - VC3; */
uint8_t rx_end_flag = 0; /* RX0: BIT0, RX1: BIT1...RX5: BIT 5 */
uint32_t cur_counter = 0;
unsigned int frame_count = 0;
/* Exposure change */
hal_syncio_cfg_t g_spg_cfg;
uint8_t reconfig_syncio;
/* I2C speed change */
#ifdef EVT3_REWORK
float i2c_write_delay_sec_asic1 = I2C_WRITE_DELAY_SEC_ASIC1_400KHZ;
float i2c_write_delay_sec_asic23 = I2C_WRITE_DELAY_SEC_ASIC23_400KHZ;
#else
float i2c_write_delay_sec_asic1 = I2C_WRITE_DELAY_SEC_ASIC1_100KHZ;
float i2c_write_delay_sec_asic23 = I2C_WRITE_DELAY_SEC_ASIC23_100KHZ;
#endif
uint8_t config_llpclk(capt_grp_t cam_group, float *rate_out, uint8_t *grp_change_llpclk);

static void snapshot_timer_irq(void *param)
{

	snapshot_timer_data_t *timer_data = (snapshot_timer_data_t *)param;
	if(timer_data)
	{
		timer_data->irq_cnt ++;
		snapshot_info_t **sinfo = timer_data->sinfo;
		for(int i = 0; i < timer_data->sinfo_cnt; i++)
		{
			if(sinfo[i]->inused && timer_data->irq_cnt >= sinfo[i]->offset)
			{
				sinfo[i]->inused = 0;
				BaseType_t woken;
				if(pdFAIL != xEventGroupSetBitsFromISR(timer_data->evenhdl,
											(1 << sinfo[i]->cam_idx), &woken))
				{
					portYIELD_FROM_ISR(woken);
				}
			}
		}
	}
}

int wait_trigger(hal_syncio_channel_t ch)
{
	volatile uint32_t timeout = 1000000;
	uint32_t cnt = hal_syncio_get_counter(ch);
	while(!cnt && timeout)
	{
		cnt = hal_syncio_get_counter(ch);
		timeout--;
	}
	if(timeout <= 0)
	{
		return 0;
	}
	SLOGF(SLOG_DEBUG, "The counter is: 0x%x ", cnt);
	return 1;
}

void stop_mipi_for_cam(uint8_t cam_ch, uint8_t vc, uint8_t tx_ch)
{
	if(!preview_sem)
		preview_sem = xSemaphoreCreateMutex();
	xSemaphoreTake(preview_sem, portMAX_DELAY);
	tx_end_flag |= ((1 << vc) << (tx_ch * 4));
	rx_end_flag |= (1 << cam_ch);
	SLOGF(SLOG_DEBUG, "RX end mask 0x%x", rx_end_flag);
	xSemaphoreGive(preview_sem);
}

static void snapshot_complete_callback(void)
{
    // printf("\r\nSNAPSHOT COMPLETED CALLBACK\r\n");
}

static void tx_cb(axi2mipi_channel_t channel, axi2mipi_isr_flag_t detail,  void *user_data)
{
#ifdef DEBUG_MIPI
	unsigned int tick = xTaskGetTickCountFromISR();
#endif
	if(detail & 0xFFFF0000)
	{
		printf("\r\nTX error: 0x%x\r\n", detail);
		hal_axi2mipi_irq_disable(channel);
	}
	uint8_t force_stop = 0;
	uint8_t tx_end_shift = 0;
	if(AXI2MIPI_BRIDGE_0 == channel)
	{
		force_stop = tx_end_flag & 0xF & detail;
		tx_end_shift = 0;
	}
	else
	{
		force_stop = (tx_end_flag >> 4) & 0xF & detail;
		tx_end_shift = 4;
	}
	if(detail & FS_INT_FIFO_A)
	{
#ifdef DEBUG_MIPI
		unsigned int tick = xTaskGetTickCountFromISR();
		printf("\r\nTX SOF [ts: %d]\r\n", tick);
		printf("\r\nFrame time: %d, Frame cnt: %d\r\n", (tick - prev_tick), frame_count);
		prev_tick = tick;
#endif
	}
	if(detail & FE_INT_FIFO_A)
	{
#ifdef DEBUG_MIPI
		printf("\r\nTX EOF [ts: %d]\r\n", tick);
#endif
	}
	if(force_stop & FE_INT_FIFO_A)
	{
		/* printf("\r\nTX%d FIFO A stopped\r\n", channel); */
		tx_end_flag &= ~(FE_INT_FIFO_A << tx_end_shift);
	}
	if(force_stop & FE_INT_FIFO_B)
	{
		/* printf("\r\nTX%d FIFO B stopped\r\n", channel); */
		tx_end_flag &= ~(FE_INT_FIFO_B << tx_end_shift);
	}
	if(force_stop & FE_INT_FIFO_C)
	{
		/* printf("\r\nTX%d FIFO C stopped\r\n", channel); */
		tx_end_flag &= ~(FE_INT_FIFO_C << tx_end_shift);
	}
	if(force_stop & FE_INT_FIFO_D)
	{
		/* printf("\r\nTX%d FIFO D stopped\r\n", channel); */
		tx_end_flag &= ~(FE_INT_FIFO_D << tx_end_shift);
	}
}

static void rx_cb(uint8_t iidx,	uint32_t detail, uint32_t error_irq_detail, void *user_data)
{
#ifdef DEBUG_MIPI
	unsigned int tick = xTaskGetTickCountFromISR();
	if(detail & R1_VC0_FRAME_START)
	{
		printf("\r\nRX SOF [ts: %d]\r\n", tick);
	}
#endif
	if(detail & R1_VC0_FRAME_END)
	{
#ifdef DEBUG_MIPI
		unsigned int tick = xTaskGetTickCountFromISR();
		printf("\r\nRX EOF [ts: %d]\r\n", tick);
		frame_count++;
#endif
		if(rx_end_flag & (1 << iidx))
		{
			xSemaphoreTakeFromISR(preview_sem, NULL);
			rx_end_flag &= ~(uint8_t)(1 << iidx);
			hal_syncio_disable(iidx);
			xSemaphoreGiveFromISR(preview_sem, NULL);

		}
	}
#ifdef DEBUG_MIPI
	if(error_irq_detail)
		printf("\r\nRX Error: %d\r\n", (unsigned int)error_irq_detail);
#endif
}

void setup_mipi_rx_preview(cam_typedef_t *pcam,
				streaming_csi_t preview_csi, uint16_t width, uint16_t height)
{
	uint8_t tx_fifo = pcam->settings->stream[1];
	uint8_t preview_ins = pcam->info.ch - 1;

	hal_mipi2axi_init(preview_ins);
	if(preview_csi == CSI_0)
		hal_mipi2axi_set_tx_base_addr(preview_ins, TXS0_BASE);
	else
		hal_mipi2axi_set_tx_base_addr(preview_ins, TXS1_BASE);

	mipi2axi_property_t mipi2axi_property;
	mipi2axi_property.vc_option				= MIPI2AXI_VC0_CH;
	mipi2axi_property.addr.dst_addr			= tx_fifo << 30;
	mipi2axi_property.addr.alu_src_addr		= 0;
	mipi2axi_property.img_type.vc_will_captured	= MIPI2AXI_VC0_CH;
	mipi2axi_property.img_type.data_type	= MIPI2AXI_RAW10;
	mipi2axi_property.img_size.height	= height;
	mipi2axi_property.img_size.width	= width;
	mipi2axi_property.stream_mode		= MIPI2AXI_PREVIEW_MODE;
	hal_mipi2axi_set_stream_property(preview_ins, &mipi2axi_property);
	hal_mipi2axi_wrap_cfg(preview_ins, MIPI2AXI_VC0_CH, height, DISABLE);
	hal_mipi2axi_frame_control_cfg(preview_ins, MIPI2AXI_VC0_CH, 0, DISABLE);
	hal_mipi2axi_irq_mask(
			preview_ins,
			MIPI2AXI_SIGNAL_MASK_INTR,
			R1_VC0_FRAME_START | R1_VC0_FRAME_END,
			ENABLE);

	hal_mipi2axi_irq_enable(preview_ins, rx_cb, NULL);
	hal_mipi2axi_start(preview_ins);
}

void setup_mipi_rx_snapshot(cam_typedef_t *pcam,
				streaming_csi_t preview_csi, uint16_t width, uint16_t height)
{
	uint8_t preview_ins = pcam->info.ch - 1;
	uint32_t dst_addr = 0x03400000;
	hal_mipi2axi_init(preview_ins);
	mipi2axi_property_t mipi2axi_property;
	mipi2axi_property.vc_option				= MIPI2AXI_VC0_CH;
	mipi2axi_property.addr.dst_addr			= dst_addr;
	mipi2axi_property.addr.alu_src_addr		= 0;
	mipi2axi_property.img_type.vc_will_captured	= MIPI2AXI_VC0_CH;
	mipi2axi_property.img_type.data_type	= MIPI2AXI_RAW10;
	mipi2axi_property.img_size.height	= height;
	mipi2axi_property.img_size.width	= width;
	mipi2axi_property.stream_mode		= MIPI2AXI_SNAPSHOT_MODE;
	hal_mipi2axi_set_stream_property(preview_ins, &mipi2axi_property);
	hal_mipi2axi_wrap_cfg(preview_ins, MIPI2AXI_VC0_CH, 10, ENABLE);

	hal_mipi2axi_irq_mask(
			preview_ins,
			MIPI2AXI_SIGNAL_MASK_INTR,
			R1_VC0_FRAME_START | R1_VC0_FRAME_END,
			ENABLE);

	hal_mipi2axi_irq_enable(preview_ins, rx_cb, NULL);
	hal_mipi2axi_start(preview_ins);
}
static uint8_t stream_vc_to_tx_ff(uint8_t vc)
{
	uint8_t ffa = AXI2MIPI_FIFO_A;
	return (ffa << vc);
}

static void syncio_preview_interrupt(hal_syncio_channel_t chid,
											hal_syncio_irq_mode_t irq_status)
{
	if(reconfig_syncio)
	{
		reconfig_syncio = FALSE;
		/*printf("\r\nEXPOSURE CHANGE [%d]:\r\n",(int)chid);*/
		hal_syncio_disable(chid);
		hal_syncio_config(chid, &g_spg_cfg);
		hal_syncio_enable(chid);
		hal_syncio_trigger();
	}
}

static void setup_spg_preview(cam_typedef_t *pcam, int ispreview, int numpulse)
{
	/* The camera channel run from [1-6]
	 * We need to remap to find the RX channel
	 */
	uint8_t rx_ins = pcam->info.ch - 1;
	uint32_t llpclk = (uint32_t)img_sensor_read_reg(pcam->image,
										LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t fll = (uint32_t)img_sensor_read_reg(pcam->image,
											FLL_CAM_REG, TWO_BYTES);
	uint32_t vtpixclk = (uint32_t)img_sensor_get_vt_pix_clk_mhz(pcam->image);
	SLOGF(SLOG_DEBUG, "LLPCLK: 0x%x, FLL: 0x%x, VTPIXCLK: %d \r\n",
									(int)llpclk, (int)fll, (int)vtpixclk);
	hal_syncio_cfg_t spg_cfg;
	spg_cfg.inf_mode = ispreview ? SG_INF_EN : SG_INF_DIS;
	spg_cfg.repeat = numpulse;
	spg_cfg.trig_mode = SG_TRIG_SW;
	spg_cfg.lat1 = 0;
	spg_cfg.width1 = 0;
	spg_cfg.width2 = 1 * SPG_MILISECOND;
	/* 1usec for buffer and subtract with pulse width */
	spg_cfg.lat2 = (uint32_t)((SPG_TO_IT * llpclk * (fll)) / vtpixclk);
    spg_cfg.lat2 -= 0.790 * SPG_MILISECOND;
	SLOGF(SLOG_DEBUG, "SPG latency: %d", spg_cfg.lat2);
	SLOGF(SLOG_DEBUG, "SPG : %d", (unsigned int)((llpclk * fll) / (vtpixclk * 2)));

	reconfig_syncio = FALSE;
	hal_syncio_irq_t intr;
	intr.irq_modes = SG_INTR_FALLING_EDGE;
	intr.callback_handler = syncio_preview_interrupt;
	hal_syncio_enable_irq(rx_ins, &intr);

	hal_syncio_config(rx_ins, &spg_cfg);
	hal_syncio_enable(rx_ins);
}

void reconfig_spg_for_previewing_sensor(cam_typedef_t *pcam)
{


	uint32_t llpclk = (uint32_t)img_sensor_read_reg(pcam->image,
						LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t fll = (uint32_t)img_sensor_read_reg(pcam->image,
					FLL_CAM_REG, TWO_BYTES);
	uint32_t vtpixclk = (uint32_t)img_sensor_get_vt_pix_clk_mhz(pcam->image);
	SLOGF(
			SLOG_DEBUG,
			"llpclk: 0x%x, fll: 0x%x, vtpixclk: 0x%x",
			llpclk,
			fll,
			vtpixclk);

	g_spg_cfg.inf_mode = SG_INF_EN;
	g_spg_cfg.repeat = 1;
	g_spg_cfg.trig_mode = SG_TRIG_SW;
//	spg_cfg.width1 = 1 * SPG_MICROSECOND;
//	spg_cfg.lat1 = (uint32_t)((SPG_TO_IT * llpclk * (fll + 1)) / vtpixclk);
	g_spg_cfg.width1 = 0;
	g_spg_cfg.lat1 = 0;
	g_spg_cfg.width2 = 1 * SPG_MILISECOND;
	g_spg_cfg.lat2 = (uint32_t)((SPG_TO_IT * llpclk * (fll)) / vtpixclk);
	g_spg_cfg.lat2 -= 0.790 * SPG_MILISECOND;

//	spg_cfg.lat2 = spg_cfg.lat2 - 2*SPG_MILISECOND;
	double ft = (llpclk * fll * 1.0)/(2 * vtpixclk * MHZ_TO_HZ);
	SLOGF(SLOG_INFO, "Frame time: %.04f", (float)ft);
	SLOGF(SLOG_DEBUG, "SPG latency: %d", g_spg_cfg.lat2);
	reconfig_syncio = TRUE;
}

void ucid_debug_hdl(void)
{

}
void tx_fix_pattern(uint32_t mbitmask, uint8_t mnumber)
{
	int i = 0;
	uint8_t cam_idx;
	cam_typedef_t *pcam = NULL;
	uint8_t tx_ff_mask[2] = {0};
	axi2mipi_isr_flag_t tx_intr[2] = {0};
	uint8_t preview_vc;
	uint8_t preview_csi;
	uint16_t img_width = 0;
	uint16_t img_height = 0;
	uint32_t tmp_bitmask = mbitmask;
	uint32_t read_src_addr = 0x03400000;

	SLOGF(SLOG_DEBUG, "Starting fix pattern");
	hal_axi2mipi_init(AXI2MIPI_BRIDGE_0);
	hal_axi2mipi_init(AXI2MIPI_BRIDGE_1);

	for(i = 0; i < mnumber; i++)
	{
		cam_idx = __builtin_ctz(tmp_bitmask);
		if(cam_idx == 0)
			break;
		tmp_bitmask &= ~(1 << cam_idx);
		pcam = idx_to_object(cam_idx);
		if(pcam)
		{
			preview_vc = pcam->settings->stream[1];
			preview_csi = (pcam->settings->stream[0] & 0xF0) >> 4;

			if(preview_csi != CSI_0 && preview_csi != CSI_1)
			{
				SLOGF(SLOG_ERROR, "Not support CSI[%d]", (int)preview_csi);
				return;
			}

			if(preview_vc < VC0_FOR_FIFO || preview_vc > VC3_FOR_FIFO)
			{
				SLOGF(SLOG_ERROR, "Not support VC[%d]", (int )preview_vc);
				return;
			}

			/*TODO: Read from resolution setting will be faster*/
			img_width = img_sensor_read_reg(pcam->image,
											X_OUTPUT_SIZE_CAM_REG, TWO_BYTES);
			img_height = img_sensor_read_reg(pcam->image,
											Y_OUTPUT_SIZE_CAM_REG, TWO_BYTES);
			img_width = 320;
			img_height = 240;
			uint8_t tx_ff = stream_vc_to_tx_ff(preview_vc);
			if(preview_csi == CSI_0)
			{
				tx_ff_mask[0] |= tx_ff;
				//tx_intr[0] |= (1 << preview_vc) | ((1 << preview_vc) << 4);
				tx_intr[0] |= 0xFFFFFFFF;
			}
			else
			{
				tx_ff_mask[1] |= tx_ff;
				//tx_intr[1] |= (1 << preview_vc) | ((1 << preview_vc) << 4);
				tx_intr[1] |= 0xFFFFFFFF;
			}
			axi2mipi_property_t axi2mipi_property;
			axi2mipi_property.fifo_x = tx_ff;
			axi2mipi_property.img_type.rx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.tx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.pack = AXI2MIPI_PACKED_DATA;
			axi2mipi_property.img_type.ff_bypass = NONE_BYPASS;
			axi2mipi_property.img_size.width = img_width;
			axi2mipi_property.img_size.height = img_height;
			axi2mipi_property.src_addr = read_src_addr;
			SLOGF(SLOG_DEBUG, "PREVIEW CSI: %x ", preview_csi);
			SLOGF(SLOG_DEBUG, "TX FIFO: %x ", tx_ff);
			SLOGF(SLOG_DEBUG, "PREVIEW VC: %x ", preview_vc);
			SLOGF(SLOG_DEBUG, "IMAGE WIDTH: %x ", img_width);
			SLOGF(SLOG_DEBUG, "IMAGE HEIGHT: %x ", img_height);
			SLOGF(SLOG_DEBUG, "Source address: %x ", read_src_addr);

			axi2mipi_property.stream_mode = AXI2MIPI_SNAPSHOT_MODE;
			axi2mipi_property.vc_for_ff = preview_vc;
			axi2mipi_property.wrap.wrap_en = DISABLE;
			hal_axi2mipi_set_stream_property(preview_csi - 1, &axi2mipi_property);
		}
	}
	if(tx_ff_mask[0] || tx_ff_mask[1])
	{
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], NONE_CONTINUOUS);
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], FRAME_CONTINUOUS);
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], ENABLE);
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], ENABLE);

		if(tx_ff_mask[0])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_0, tx_intr[0], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_0, tx_cb, NULL);
			hal_axi2mipi_start(AXI2MIPI_BRIDGE_0, tx_ff_mask[0]);
			hal_axi2mipi_start(AXI2MIPI_BRIDGE_0, tx_ff_mask[0]);
		}
		if(tx_ff_mask[1])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_1, tx_intr[1], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_1, tx_cb, NULL);
			hal_axi2mipi_start(AXI2MIPI_BRIDGE_1, tx_ff_mask[1]);
		}
		SLOGF(SLOG_DEBUG, "Start writing...\r\n");
	}
}
void ucid_preview_hdl_snapshot(uint32_t mbitmask, uint8_t mnumber)
{
	int i = 0;
	uint8_t cam_idx;
	cam_typedef_t *pcam = NULL;
	uint8_t tx_ff_mask[2] = {0};
	axi2mipi_isr_flag_t tx_intr[2] = {0};
	uint8_t preview_vc;
	uint8_t preview_csi;
	uint16_t img_width = 0;
	uint16_t img_height = 0;
	uint32_t tmp_bitmask = mbitmask;
	SLOGF(SLOG_DEBUG, "Starting preview UCID");
//	hal_axi2mipi_init(AXI2MIPI_BRIDGE_0);
//	hal_axi2mipi_init(AXI2MIPI_BRIDGE_1);

	for(i = 0; i < mnumber; i++)
	{
		cam_idx = __builtin_ctz(tmp_bitmask);
		if(cam_idx == 0)
			break;
		tmp_bitmask &= ~(1 << cam_idx);
		pcam = idx_to_object(cam_idx);
		if(pcam)
		{
			img_sensor_config_mode(pcam->image, SLAVE_MODE);
			img_sensor_pattern_mode(pcam->image, PAT_COLOR_BAR);

			preview_vc = pcam->settings->stream[1] & VC_ID_MASK;
			preview_csi = (pcam->settings->stream[0] & CSI_MASK) >> 4;

			if(preview_csi != CSI_0 && preview_csi != CSI_1)
			{
				SLOGF(SLOG_ERROR, "Not support CSI[%d]", (int)preview_csi);
				return;
			}

			if(preview_vc < VC0_FOR_FIFO || preview_vc > VC3_FOR_FIFO)
			{
				SLOGF(SLOG_ERROR, "Not support VC[%d]", (int )preview_vc);
				return;
			}

			/*TODO: Read from resolution setting will be faster*/

			img_width = img_sensor_read_reg(pcam->image,
											X_OUTPUT_SIZE_CAM_REG, TWO_BYTES);
			img_height = img_sensor_read_reg(pcam->image,
											Y_OUTPUT_SIZE_CAM_REG, TWO_BYTES);

			setup_spg_preview(pcam, 0, 1);
			setup_mipi_rx_snapshot(pcam, preview_csi, img_width, img_height);
			uint8_t tx_ff = stream_vc_to_tx_ff(preview_vc);
			if(preview_csi == CSI_0)
			{
				tx_ff_mask[0] |= tx_ff;
				//tx_intr[0] |= (1 << preview_vc) | ((1 << preview_vc) << 4);
				tx_intr[0] |= 0xFFFFFFFF;
			}
			else
			{
				tx_ff_mask[1] |= tx_ff;
				//tx_intr[1] |= (1 << preview_vc) | ((1 << preview_vc) << 4);
				tx_intr[1] |= 0xFFFFFFFF;
			}
			axi2mipi_property_t axi2mipi_property;
			axi2mipi_property.fifo_x = tx_ff;
			axi2mipi_property.img_type.rx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.tx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.pack = AXI2MIPI_UNPACKED_DATA;
			axi2mipi_property.img_type.ff_bypass = NONE_BYPASS;
			axi2mipi_property.img_size.width = img_width;
			axi2mipi_property.img_size.height = img_height;
			SLOGF(SLOG_DEBUG, "PREVIEW CSI: %x ", preview_csi);
			SLOGF(SLOG_DEBUG, "TX FIFO: %x ", tx_ff);
			SLOGF(SLOG_DEBUG, "PREVIEW VC: %x ", preview_vc);
			SLOGF(SLOG_DEBUG, "IMAGE WIDTH: %x ", img_width);
			SLOGF(SLOG_DEBUG, "IMAGE HEIGHT: %x ", img_height);
			if(preview_csi == CSI_0)
				axi2mipi_property.src_addr = TXS0_BASE;
			else
				axi2mipi_property.src_addr = TXS1_BASE;
			axi2mipi_property.stream_mode = AXI2MIPI_PREVIEW_MODE;
			axi2mipi_property.vc_for_ff = preview_vc;
			axi2mipi_property.wrap.wrap_en = DISABLE;
			hal_axi2mipi_set_stream_property(preview_csi - 1, &axi2mipi_property);
		}
	}
	if(tx_ff_mask[0] || tx_ff_mask[1])
	{
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], FRAME_CONTINUOUS);
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], FRAME_CONTINUOUS);
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], ENABLE);
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], ENABLE);

		if(tx_ff_mask[0])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_0, tx_intr[0], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_0, tx_cb, NULL);
			/*hal_axi2mipi_start(AXI2MIPI_BRIDGE_0, tx_ff_mask[0]);*/
		}
		if(tx_ff_mask[1])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_1, tx_intr[1], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_1, tx_cb, NULL);
			/*hal_axi2mipi_start(AXI2MIPI_BRIDGE_1, tx_ff_mask[1]);*/
		}
		/* Stream on in slave mode */
		for(i = 0; i < mnumber; i++)
		{
			cam_idx = __builtin_ctz(mbitmask);
			if(cam_idx == 0)
				break;
			mbitmask &= ~(1 << cam_idx);
			pcam = idx_to_object(cam_idx);
			if(pcam)
			{
				img_sensor_stream_on(pcam->image, SLAVE_MODE);
				/* Update the cam status */
				uint32_t *cam_status = (uint32_t *) pcam->settings->status;
				*cam_status |= S_MODULE_STREAM_ON;
			}
		}
		SLOGF(SLOG_DEBUG, "Preview started");
		hal_syncio_trigger();
	}
}
lcc_cmd_status_t ucid_preview_hdl(uint32_t mbitmask, uint8_t mnumber)
{
	int i = 0;
	uint8_t cam_idx;
	cam_typedef_t *pcam = NULL;
	uint8_t tx_ff_mask[2] = {0};
	axi2mipi_isr_flag_t tx_intr[2] = {0};
	uint8_t preview_vc;
	uint8_t preview_csi;
	uint16_t img_width = 0;
	uint16_t img_height = 0;
	uint32_t tmp_bitmask = mbitmask & light_system->m_filter;
	uint8_t num_of_preview = 0;
	lcc_cmd_status_t cmd_status = LCC_CMD_SUCCESS;
	SLOGF(SLOG_DEBUG, "Starting preview UCID");

	/* Reset MIPI TX */
	mipi_tx_reset(AXI2MIPI_BRIDGE_0);
	/* mipi_tx_reset(AXI2MIPI_BRIDGE_1); */

	/* Initializing MIPI */
	hal_axi2mipi_init(AXI2MIPI_BRIDGE_0);
	hal_axi2mipi_init(AXI2MIPI_BRIDGE_1);

	for(i = 0; i < mnumber; i++)
	{
		cam_idx = __builtin_ctz(tmp_bitmask);
		if(cam_idx == 0)
			break;
		tmp_bitmask &= ~(1 << cam_idx);
		pcam = idx_to_object(cam_idx);
		if(pcam)
		{
			img_sensor_reset_trigger(pcam->image);
            /*img_sensor_config_mode(pcam->image, I2C_MODE);*/
			img_sensor_config_mode(pcam->image, SLAVE_MODE);
			/*img_sensor_pattern_mode(pcam->image, PAT_COLOR_BAR);*/
			preview_vc = pcam->settings->stream[1];
			preview_csi = (pcam->settings->stream[0] & 0xF0) >> 4;

			if(preview_csi != CSI_0 && preview_csi != CSI_1)
			{
				SLOGF(SLOG_ERROR, "Not support CSI[%d]", (int)preview_csi);
				cmd_status = LCC_CMD_UNSUCCESS;
				goto release_resource;
			}

			if(preview_vc < VC0_FOR_FIFO || preview_vc > VC3_FOR_FIFO)
			{
				SLOGF(SLOG_ERROR, "Not support VC[%d]", (int )preview_vc);
				cmd_status = LCC_CMD_UNSUCCESS;
				goto release_resource;
			}

			cam_ctrl_resolution(pcam);
			cam_ctrl_fps(pcam);
			cam_ctrl_sensitivity(pcam);
			cam_ctrl_exposure(pcam);

            /* Flip the image as necessary here */
            /* If the pixel order is not zero, flip it */
           img_sensor_flip_preview(pcam->image, pcam->image->flip);

			/*TODO: Read from resolution setting will be faster*/
			img_width = img_sensor_read_reg(pcam->image,
											X_OUTPUT_SIZE_CAM_REG, TWO_BYTES);
			img_height = img_sensor_read_reg(pcam->image,
											Y_OUTPUT_SIZE_CAM_REG, TWO_BYTES);
			SLOGF(SLOG_INFO, "Preview resolution %dx%d for CAM-%X",
									img_width, img_height, pcam->info.module);
			setup_spg_preview(pcam, 1, 1);
			setup_mipi_rx_preview(pcam, preview_csi, img_width, img_height);


			uint8_t tx_ff = stream_vc_to_tx_ff(preview_vc);
			if(preview_csi == CSI_0)
			{
				tx_ff_mask[0] |= tx_ff;
				tx_intr[0] |= 0xFFFFFFFF;
			}
			else
			{
				tx_ff_mask[1] |= tx_ff;
				tx_intr[1] |= 0xFFFFFFFF;
			}
			axi2mipi_property_t axi2mipi_property;
			axi2mipi_property.fifo_x = tx_ff;
			axi2mipi_property.img_type.rx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.tx_dt = AXI2MIPI_RAW10;
			axi2mipi_property.img_type.pack = AXI2MIPI_UNPACKED_DATA;
			axi2mipi_property.img_type.ff_bypass = NONE_BYPASS;
			axi2mipi_property.img_size.width = img_width;
			axi2mipi_property.img_size.height = img_height;
			SLOGF(SLOG_DEBUG, "PREVIEW CSI    : %x ", preview_csi);
			SLOGF(SLOG_DEBUG, "PREVIEW TX FIFO: %x ", tx_ff);
			SLOGF(SLOG_DEBUG, "PREVIEW TX VC  : %x ", preview_vc);
			SLOGF(SLOG_DEBUG, "IMAGE WIDTH    : %x ", img_width);
			SLOGF(SLOG_DEBUG, "IMAGE HEIGHT   : %x ", img_height);
			if(preview_csi == CSI_0)
				axi2mipi_property.src_addr = TXS0_BASE;
			else
				axi2mipi_property.src_addr = TXS1_BASE;
			axi2mipi_property.stream_mode = AXI2MIPI_PREVIEW_MODE;
			axi2mipi_property.vc_for_ff = preview_vc;
			axi2mipi_property.wrap.wrap_en = DISABLE;

			hal_axi2mipi_set_stream_property(preview_csi - 1, &axi2mipi_property);

			num_of_preview++;
		}
	}
	if(tx_ff_mask[0] || tx_ff_mask[1])
	{
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], ENABLE);
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], ENABLE);
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_0, tx_ff_mask[0], FRAME_CONTINUOUS);
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_1, tx_ff_mask[1], FRAME_CONTINUOUS);
		if(tx_ff_mask[0])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_0, tx_intr[0], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_0, tx_cb, NULL);
			hal_axi2mipi_start(AXI2MIPI_BRIDGE_0, tx_ff_mask[0]);
			/*hal_mipi2axi_start(0);*/
		}
		if(tx_ff_mask[1])
		{
			hal_axi2mipi_irq_mask(AXI2MIPI_BRIDGE_1, tx_intr[1], ENABLE);
			hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_1, tx_cb, NULL);
			hal_axi2mipi_start(AXI2MIPI_BRIDGE_1, tx_ff_mask[1]);
		}
		/* Stream on in slave mode */
		for(i = 0; i < mnumber; i++)
		{
			cam_idx = __builtin_ctz(mbitmask);
			if(cam_idx == 0)
				break;
			mbitmask &= ~(1 << cam_idx);
			pcam = idx_to_object(cam_idx);
			if(pcam)
			{
				if(num_of_preview > 1)
				{
					pcam->image->old_llpclk = img_sensor_read_reg(pcam->image,
										LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
					uint32_t new_llpclk = pcam->image->old_llpclk * num_of_preview;
					if(new_llpclk > MAX_SENSOR_REG_VAL)
					{
						SLOGF(SLOG_ERROR, "Not support previewing %d cameras",
																num_of_preview);
						cmd_status = LCC_CMD_UNSUCCESS;
						goto release_resource;
					}
					SLOGF(SLOG_DEBUG, "New LLPCLK [0x%x] for multiple preview", (int)new_llpclk);
					img_sensor_write_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG,
														new_llpclk, TWO_BYTES);
				}
				/*img_sensor_stream_on(pcam->image, I2C_MODE);*/
				img_sensor_stream_on(pcam->image, SLAVE_MODE);

				/* Update the cam status */
				uint32_t *cam_status = (uint32_t *) pcam->settings->status;
				*cam_status |= S_MODULE_STREAM_ON;
			}
		}
		SLOGF(SLOG_DEBUG, "Preview started");
		hal_syncio_trigger();
	}
release_resource:
	return cmd_status;
}

uint8_t check_ddr(void)
{
	uint32_t start_addr, end_addr, read_back, i;
	uint8_t ret = 0;
	start_addr = DDR_BASE;
	end_addr = DDR_BASE + 0x10000;

	for(i = start_addr; i <= end_addr; i += 4)
	{
		writel(i, (uint32_t)0x11223344);
		read_back = readl(i);

		if(read_back != (uint32_t)(0x11223344))
		{
			ret = 1;
		}
	}

	if(ret)
	{
		SLOGF(SLOG_ERROR, "TEST DDR FAILT");
	}
	else
	{
		SLOGF(SLOG_INFO, "TEST DDR DONE!!!");
	}
	return ret;
}

static void update_it_tolerance(snapshot_info_t *sinfo, uint8_t idx, uint8_t total)
{
	uint8_t offset = 0;
	sinfo->it_tolerance = 0;
#if (ASIC_NUM == ASIC1)
	offset = (total - idx - 1);
	sinfo->it_tolerance -= (offset * 1.0 * i2c_write_delay_sec_asic1);
#else
	offset = (idx + 1);
	sinfo->it_tolerance += (offset * 1.0 * i2c_write_delay_sec_asic23);
#endif
	SLOGF(SLOG_INFO, "CAM-%X has tolerance %.06f",
						sinfo->pcam->info.module, (float)sinfo->it_tolerance);
}
static void syncio_pulse_interrupt(hal_syncio_channel_t chid,
											hal_syncio_irq_mode_t irq_status)
{
	cur_counter++;
}
static uint8_t snapshot_syncio_config(snapshot_info_t *sinfo, uint8_t numpulse)
{
	hal_syncio_cfg_t cfg;
	hal_syncio_irq_t intr;

	cfg.inf_mode = SG_INF_DIS;
	cfg.repeat = numpulse;
	cfg.trig_mode = SG_TRIG_HW;
	cfg.width1 = 1 * SPG_MILISECOND;
	if(numpulse > light_system->settings->burst[light_system->active_ucid].burst_requested)
	{
		cfg.lat1 = (uint32_t)((sinfo->ft) * SPG_SECOND);
		/* To compensate the pulse */
		cfg.lat1 -= 1 * SPG_MILISECOND;
	}
	else
	{
		cfg.lat1 = (uint32_t)((sinfo->it + sinfo->it_tolerance + 0.001206) * SPG_SECOND);
	}
	cfg.width2 = 0;
	cfg.lat2 = 0;
	sinfo->syncio_lat = cfg.lat1;
	hal_syncio_init(sinfo->syncio_idx);
	hal_syncio_config(sinfo->syncio_idx, &cfg);

	SLOGF(SLOG_INFO, "CAM-%X has latency %d and %d pulses",
				sinfo->pcam->info.module,(unsigned int)cfg.lat1, cfg.repeat);

	intr.irq_modes = SG_INTR_PULSE_DONE;
	intr.callback_handler = syncio_pulse_interrupt;
	hal_syncio_enable_irq(sinfo->syncio_idx, &intr);
	hal_syncio_enable(sinfo->syncio_idx);
	return 0;
}
static void free_snapshot_info_resource(snapshot_info_t **sinfo, uint8_t size)
{
	int i = 0;
	for(i = 0; i < size; i++)
	{
		vPortFree(sinfo[i]);
		sinfo[i] = NULL;
	}
}
static void camera_offset_stream_on(snapshot_info_t **sinfo, uint8_t total, uint8_t streamed_on)
{
	if(streamed_on < total)
	{
		snapshot_timer_data_t *timer_data = NULL;
		timer_data = pvPortMalloc(sizeof(snapshot_timer_data_t));
		if(timer_data == NULL)
		{
			SLOGF(SLOG_ERROR, "%s:%d Malloc failed", __FUNCTION__, __LINE__);
			return;
		}
		timer_data->sinfo = sinfo;
		timer_data->evenhdl = xEventGroupCreate();
		hal_timer_t stimer;
		stimer.chid = HAL_TIM_CH3;
		stimer.period = 10;
		stimer.params = timer_data;
		stimer.callback_handler = snapshot_timer_irq;
		hal_timer_stop(stimer.chid);
		hal_timer_init(&stimer);
		hal_timer_start(stimer.chid);
		while(streamed_on < total)
		{
			EventBits_t event = xEventGroupWaitBits(timer_data->evenhdl,
										0xFFFF, pdTRUE, pdFALSE, portMAX_DELAY);
			if(event)
			{
				for(int i = 0; i < total; i++)
				{
					uint32_t cam_msk = (uint32_t)(1 << sinfo[i]->cam_idx);
					if((cam_msk & event) && (sinfo[i]->inused))
					{
						sinfo[i]->inused = 0;
						img_sensor_stream_on(sinfo[i]->pcam->image, SLAVE_MODE);
						streamed_on ++;
						uint32_t *cam_status = (uint32_t *)sinfo[i]->pcam->settings->status;
						*cam_status |= S_MODULE_STREAM_ON;
					}
				}
			}
		}
		if(timer_data)
		{
			timer_data->sinfo = NULL;
			vEventGroupDelete(timer_data->evenhdl);
			timer_data = NULL;
		}
	}
}

lcc_cmd_status_t ucid_hires_hdl(uint32_t mbitmask, uint8_t mnumber, uint8_t wait_asics)
{
#if (ASIC_NUM == ASIC1)
	vTaskSuspend(task_asic_read_handle);
#endif
	hal_axi2mipi_stop(AXI2MIPI_BRIDGE_0, AXI2MIPI_FIFO_ALL);
	hal_axi2mipi_irq_disable(AXI2MIPI_BRIDGE_0);
	int i = 0;
	int ret_code = 0;
	lcc_cmd_status_t cmd_status = LCC_CMD_SUCCESS;
	uint8_t skip_frame = 0;
	TaskHandle_t tx_hdl = NULL, lh_hdl = NULL;

	uint32_t tmp_bitmask = mbitmask;
	mnumber = __builtin_popcount(mbitmask);

	snapshot_info_t *sinfo[mnumber];
	uint8_t sinfo_cnt = 0;

	snapshot_timer_data_t *timer_data = NULL;

	uint8_t burst_size = light_system->settings->burst[light_system->active_ucid].burst_requested;
	/* Reset SYNCIO interrupt counter */
	cur_counter = 0;

	for(i = 0; i < mnumber; i++)
	{
		uint8_t cam_idx = __builtin_ctz(tmp_bitmask);
		tmp_bitmask &= ~(1 << (cam_idx));
		cam_typedef_t *pcam = idx_to_object(cam_idx);
		if(pcam)
		{
			/* Update the settings for UCID */
			cam_ctrl_resolution(pcam);
			cam_ctrl_fps(pcam);
			cam_ctrl_sensitivity(pcam);
			cam_ctrl_exposure(pcam);

			img_sensor_reset_trigger(pcam->image);
			img_sensor_config_mode(pcam->image, SLAVE_MODE);
#ifdef SENSOR_PATTERN_MODE
			img_sensor_pattern_mode(pcam->image, PAT_COLOR_BAR);
#endif
			sinfo[sinfo_cnt] = pvPortMalloc(sizeof(snapshot_info_t));
			if(sinfo[sinfo_cnt] == NULL)
			{
				SLOGF(SLOG_ERROR, "%s:%d Malloc failed", __FUNCTION__, __LINE__);
				goto release_resource;
			}
			sinfo[sinfo_cnt]->it = img_sensor_get_frame_time(pcam->image,
													&(sinfo[sinfo_cnt]->ft));
			sinfo[sinfo_cnt]->it_tolerance = 0;
			sinfo[sinfo_cnt]->pcam = pcam;
			SLOGF(SLOG_INFO,
				"CAM-%X has integration time %.04f (seconds) and frame time %.04f",
				sinfo[sinfo_cnt]->pcam->info.module,
				(float)sinfo[sinfo_cnt]->it,
				(float)sinfo[sinfo_cnt]->ft);

			if(sinfo[sinfo_cnt]->it < MAX_IT_TO_SKIP || burst_size > 1)
				skip_frame = 1;
			sinfo[sinfo_cnt]->cam_idx = cam_idx;
			sinfo[sinfo_cnt]->syncio_idx = pcam->info.ch - 1;
			sinfo[sinfo_cnt]->offset = 0;
			sinfo[sinfo_cnt]->inused = 1;
			sinfo_cnt ++;
		}
	}
	for(i = 0; i < sinfo_cnt; i++)
	{
		if(!skip_frame)
			update_it_tolerance(sinfo[i], i, sinfo_cnt);
		snapshot_syncio_config(sinfo[i], burst_size + skip_frame);
	}

	ret_code = usecase_hires_setup(mbitmask,
								MIPI2AXI_RAW10,
								burst_size + skip_frame,
								snapshot_complete_callback,
								&lh_hdl);
	if(ret_code)
	{
		SLOGF(SLOG_ERROR, "Setup MIPI TX/RX error [%d] ", ret_code);
		SLOGF(SLOG_ERROR, "Cancelled HIRES capture request");
		cmd_status = LCC_CMD_UNSUCCESS;
		goto release_resource;
	}

	xTaskCreate(task_tx_transfer,
					"TX transferring",
					__TASK_STACK_SIZE_256,
					(void *)&skip_frame,
					__TASK_PRIO_HIGHEST,
					&tx_hdl);
#if (ASIC_NUM == ASIC1)
	/* Wait for feedback from ASIC2 & 3 */
	if(wait_asics)
		SLOGF(SLOG_INFO, "Wait for interrupt signal");

	if(wait_asics == wait_intr_signal(wait_asics, MAX_INTR_WAIT_TIME, 1))
	{
		uint8_t streamed_on_cnt = 0;
		/* Stream on all camera that has offset <= 1 */
		for(i = 0; i < sinfo_cnt; i++)
		{
			if(sinfo[i]->offset == 0)
			{
				sinfo[i]->inused = 0;
				img_sensor_stream_on(sinfo[i]->pcam->image, SLAVE_MODE);
				streamed_on_cnt ++;
				/* Update the cam status */
				uint32_t *cam_status = (uint32_t *)sinfo[i]->pcam->settings->status;
				*cam_status |= S_MODULE_STREAM_ON;
			}
		}

		send_hw_sync_trigger();
		camera_offset_stream_on(sinfo, sinfo_cnt, streamed_on_cnt);
		while(!tx_transfer_done)
			vTaskDelay(10);
	}
	else
	{
		SLOGF(SLOG_ERROR, "Timeout waiting for feedback from ASIC2 and ASIC3");
		cmd_status = LCC_CMD_UNSUCCESS;
		if(lh_hdl)
		{
			vTaskDelete(lh_hdl);
			lh_hdl = NULL;
		}
		if(tx_hdl)
		{
			vTaskDelete(tx_hdl);
			tx_hdl = NULL;
		}
	}
#else

	SLOGF(SLOG_INFO, "Send interrupt signal");
	send_intr_signal();
	if(wait_trigger(sinfo[0]->syncio_idx) == 0)
	{
		SLOGF(SLOG_ERROR, "Timeout waiting trigger from ASIC1");
		cmd_status = LCC_CMD_UNSUCCESS;
		if(lh_hdl)
		{
			vTaskDelete(lh_hdl);
			lh_hdl = NULL;
		}
		if(tx_hdl)
		{
			vTaskDelete(tx_hdl);
			tx_hdl = NULL;
		}
		goto release_resource;
	}

	uint8_t streamed_on_cnt = 0;
	for(i = 0; i < sinfo_cnt; i++)
	{
		if(sinfo[i]->offset == 0)
		{
			sinfo[i]->inused = 0;
			img_sensor_stream_on(sinfo[i]->pcam->image, SLAVE_MODE);
			streamed_on_cnt ++;
			/* Update the cam status */
			uint32_t *cam_status = (uint32_t *)sinfo[i]->pcam->settings->status;
			*cam_status |= S_MODULE_STREAM_ON;
		}
	}
	camera_offset_stream_on(sinfo, sinfo_cnt, streamed_on_cnt);
	while(!tx_transfer_done)
		vTaskDelay(10);

#endif

release_resource:
	/* If the task done, these task handles already deleted */
	lh_hdl = NULL;
	tx_hdl = NULL;
	free_snapshot_info_resource(sinfo, sinfo_cnt);
	SLOGF(SLOG_INFO, "End flow");
	if(timer_data)
	{
		timer_data->sinfo = NULL;
		vEventGroupDelete(timer_data->evenhdl);
		timer_data = NULL;
	}
#if (ASIC_NUM == ASIC1)
	vTaskResume(task_asic_read_handle);
#endif
	return cmd_status;
}

void mipi_rx_reset(mipi2axi_channel_t channel)
{
	uint32_t mipi_rx_rst = 0;

	/*Read MIPI channel on ASB board*/
	mipi_rx_rst = 1 << (SHIFT_TO_BIT14 + channel);

	/*SLOGF(
			SLOG_DEBUG,
			"RESET MIPI RX [%d]: 0x%X",
			(unsigned int )channel,
			mipi_rx_rst);*/

	writel(SCU_BASE + SCU_SOFT_RESET_REG, mipi_rx_rst);
	vTaskDelay(1);
	writel(SCU_BASE + SCU_SOFT_RESET_REG, 0x00);

	/* Disable MIPI interrupt
	 * After reset MIPI, all MIPI RX interrupts will enable by default
	 * (Interrupt reset value is 0, that mean enable interrupt)
	 */
	hal_mipi2axi_irq_mask(
			channel,
			MIPI2AXI_SIGNAL_MASK_INTR,
			ISR_SELECT_ALL,
			DISABLE);

	hal_mipi2axi_irq_mask(
			channel,
			MIPI2AXI_ERROR_MASK_INTR,
			ISR_SELECT_ALL,
			DISABLE);
}

void mipi_tx_reset(axi2mipi_channel_t channel)
{
	uint32_t mipi_tx_rst = 0;

	/*Read MIPI channel on ASB board*/
	mipi_tx_rst = 1 << (SHIFT_TO_BIT12 + channel);

	/*SLOGF(
			SLOG_DEBUG,
			"RESET MIPI TX [%d]: 0x%X",
			(unsigned int )channel,
			mipi_tx_rst);*/

	writel(SCU_BASE + SCU_SOFT_RESET_REG, mipi_tx_rst);
	vTaskDelay(1);
	writel(SCU_BASE + SCU_SOFT_RESET_REG, 0x00);

	/* Disable MIPI interrupt
	 * After reset MIPI, all MIPI TX interrupts will enable by default
	 * (Interrupt reset value is 0, that mean enable interrupt)
	 */
	hal_axi2mipi_irq_mask(channel, ISR_SELECT_ALL, DISABLE);
}

uint8_t config_llpclk(capt_grp_t cam_group, float *rate_out, uint8_t *grp_change_llpclk)
{
	switch(cam_group)
	{
		case CAPT_GRP_AB:
			SLOGF(SLOG_DEBUG, "Capture group AB");
			*rate_out = LLPCLK_GRP_AB_RATE;
			*grp_change_llpclk = GRP_A;
			break;

		case CAPT_GRP_BC:
			SLOGF(SLOG_DEBUG, "Capture group BC");
			*rate_out = LLPCLK_GRP_BC_RATE;
			*grp_change_llpclk = GRP_B;
			break;
		case CAPT_GRP_C:
			SLOGF(SLOG_DEBUG, "Capture group C");
			*grp_change_llpclk = GRP_C;
			*rate_out = LLPCLK_GRP_C_RATE;
			break;
		case CAPT_GRP_ABC:
			SLOGF(SLOG_DEBUG, "Capture group ABC");
			*rate_out = LLPCLK_GRP_RATE_DEFAULT;
			*grp_change_llpclk = 0;
			break;
		case CAPT_GRP_A:
			SLOGF(SLOG_DEBUG, "Capture group A");
			*grp_change_llpclk = 0;
			*rate_out = LLPCLK_GRP_RATE_DEFAULT;
			break;
		case CAPT_GRP_B:
			SLOGF(SLOG_DEBUG, "Capture group B");
			*grp_change_llpclk = 0;
			*rate_out = LLPCLK_GRP_RATE_DEFAULT;
			break;
		default:
			return 1; /* has error */
			break;
	}

	return 0; /* no error */
}

