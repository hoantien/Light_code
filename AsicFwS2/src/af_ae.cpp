/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    af_ae.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-22-2016
 * @brief   Implements auto-exposure and auto-focus
 *
 ******************************************************************************/

#include <cstring>
#include <memory>
#include "operator_new.h"
#include "log.h"
#include "i2cm.h"
#include "board_config.h"
#include "af_helper.h"
#include "disparityfocus.h"
#include "image_raw10.h"
#include "lens_position_control.h"
#include "af_roi_transfer.h"
#include "calib_data_manager.h"
#include "optical_zoom_manager.h"
#include "hal_cache.h"
#include "af_ae.h"

#define SLOGF_ID                SLOG_ID_AF


extern "C" void pri_roi_mipi_callback(uint8_t iidx, uint32_t signal_irq,
        uint32_t error_irq, void *user_data) {
    static_cast<af_ae*>(user_data)->pri_mipi2axi_cb(iidx, signal_irq, error_irq);
}

extern "C" void aux_roi_mipi_callback(uint8_t iidx, uint32_t signal_irq,
        uint32_t error_irq, void *user_data) {
    static_cast<af_ae*>(user_data)->aux_mipi2axi_cb(iidx, signal_irq, error_irq);
}

extern "C" void execute_af_ae(void) {
    roi_rectangle_t roi_rect;
    std::memcpy(&roi_rect, light_system->settings->roi_af, sizeof(roi_rect));
    std::int16_t left_x = roi_rect.left_x;
    std::int16_t top_y = roi_rect.top_y;
    std::uint16_t width = roi_rect.width;
    std::uint16_t height = roi_rect.height;
    rectangle pri_roi{left_x, top_y, width, height};
    af_ae af;
    af.run(pri_roi);
}

namespace {
#if (ASIC_NUM == ASIC1)
    void lens_movement_done_cb(void* user_data, bool success, std::int16_t pos) {
        xSemaphoreGive(*static_cast<SemaphoreHandle_t*>(user_data));
    }
#endif
}

af_ae::af_ae() :
    pri_capture_sem_{xSemaphoreCreateBinary()},
    aux_capture_sem_{xSemaphoreCreateBinary()} {
}

af_ae::~af_ae() {
    vSemaphoreDelete(pri_capture_sem_);
    vSemaphoreDelete(aux_capture_sem_);
}

void af_ae::run(const rectangle& pri_roi) {
    // Start moving mirrors to zoom position
    optical_zoom_manager::instance().start_mirrors_move();

#if (ASIC_NUM == ASIC1)
    float zoom_factor = light_system->settings->zoom_factor;
    bool is_ab_mode = (zoom_factor < 2.5f);
    // Start moving both camera modules to same focus position
    cam_typedef_t* pcam_pri = idx_to_object(is_ab_mode ? 1 : 9); // A1 or B4
    cam_typedef_t* pcam_aux = idx_to_object(5); // A5
    lens_position_control lpc_pri(*pcam_pri);
    lens_position_control lpc_aux(*pcam_aux);
    std::uint32_t init_distance_mm = is_ab_mode ? 800 : 1450;
    std::int16_t hc_pri = lpc_pri.get_hall_code(init_distance_mm, 300); // FIXME module temperature
    std::int16_t hc_aux = lpc_aux.get_hall_code(init_distance_mm, 300); // FIXME module temperature
    lpc_pri.start_move_lens(hc_pri, 10,
            lens_position_control::move_done_callback_t{&lens_movement_done_cb, &pri_capture_sem_});
    lpc_aux.start_move_lens(hc_aux, 10,
            lens_position_control::move_done_callback_t{&lens_movement_done_cb, &aux_capture_sem_});

    // Find minimum capture rectangle needed from aux camera
    ltaf::DisparityFocus df(calib_data_manager::instance().geometric_calib());

    std::uint16_t capture_mode = is_ab_mode ? 0 : 1;
    std::uint16_t scale = 2; //1;
    std::uint32_t resolution[2] = {X_3M_P2, Y_3M_P2}; //{X_13M, Y_13M};
    SLOGF(SLOG_INFO, "Pri roi: left_x: %d, top_y: %d, width: %d, height: %d",
            pri_roi.left_x(), pri_roi.top_y(), pri_roi.width(), pri_roi.height());
    rectangle aux_capture_rect{df.computeAuxRectangle(pri_roi, capture_mode)};
    SLOGF(SLOG_INFO, "Aux capture rectangle: left_x: %d, top_y: %d, width: %d, height: %d",
            aux_capture_rect.left_x(), aux_capture_rect.top_y(), aux_capture_rect.width(), aux_capture_rect.height());

    // Choose exposure time and gain
    std::uint64_t exposure_ns{0};
    float gain{1.0f};
    std::memcpy(&exposure_ns, pcam_pri->image->settings[UC_FOCAL_STACKING]->exposure, sizeof(exposure_ns));
    std::memcpy(&gain, pcam_pri->image->settings[UC_FOCAL_STACKING]->sensitivity, sizeof(gain));
    if (exposure_ns == 0) {
        std::memcpy(&exposure_ns, pcam_pri->image->settings[UC_PREVIEW]->exposure, sizeof(exposure_ns));
        std::memcpy(&gain, pcam_pri->image->settings[UC_PREVIEW]->sensitivity, sizeof(gain));
    }
    if (exposure_ns == 0) {
        exposure_ns = 30000000;
        gain = 1.0f;
    }
    SLOGF(SLOG_DEBUG, "Selected gain/exposure_ns for disparity: %f/%llu", gain, (unsigned long long)exposure_ns);
    // Request capture for pri
    af_setting_t afs_pri;
    afs_pri.cam = pcam_pri;
    afs_pri.flip_mode = FLIP_MODE_CAPTURE;
    flip_mode_t flip_mode_pri{get_af_flip_mode(afs_pri.flip_mode, afs_pri.cam->image)};
    rectangle abs_pri_capture_rect{calib_rectangle_to_abs(pri_roi, flip_mode_pri).minimum_capture_rectangle(scale)};
    afs_pri.mipi_rx_chid = afs_pri.cam->info.ch - 1;
    afs_pri.x_position = abs_pri_capture_rect.left_x();
    afs_pri.y_position = abs_pri_capture_rect.top_y();
    afs_pri.x_width = abs_pri_capture_rect.width();
    afs_pri.y_width = abs_pri_capture_rect.height();
    // Allocate DDR memory for pri roi capture
    std::size_t pri_roi_img_size = abs_pri_capture_rect.image_size(scale);
    std::unique_ptr<char[]> pri_roi_img_ptr{new(Heap::DDR) char[pri_roi_img_size]};
    SLOGF(SLOG_DEBUG, "Pri capture image addr: %x, size: %d", (int)pri_roi_img_ptr.get(), (int)pri_roi_img_size);
    afs_pri.destination = reinterpret_cast<std::uint32_t>(pri_roi_img_ptr.get());
    afs_pri.af_cb = &pri_roi_mipi_callback;
    afs_pri.cb_user_data = this;
    // Exposure, fps, resolution, sensitivity settings...
    std::memcpy(afs_pri.cam_settings.exposure, &exposure_ns, sizeof(afs_pri.cam_settings.exposure));
    std::uint16_t fps = 30;
    std::memcpy(afs_pri.cam_settings.fps, &fps, sizeof(afs_pri.cam_settings.fps));
    std::memcpy(afs_pri.cam_settings.resolution, &resolution, sizeof(afs_pri.cam_settings.resolution));
    afs_pri.scale = scale;
    std::memcpy(afs_pri.cam_settings.sensitivity, &gain, sizeof(afs_pri.cam_settings.sensitivity));
    // Configure pri sensor for captures
    af_configure_sensor_stream(&afs_pri, false);

    af_setting_t afs_aux;
    afs_aux.cam = pcam_aux;
    afs_aux.flip_mode = FLIP_MODE_CAPTURE;
    flip_mode_t flip_mode_aux{get_af_flip_mode(afs_aux.flip_mode, afs_aux.cam->image)};
    rectangle abs_aux_capture_rect{calib_rectangle_to_abs(aux_capture_rect, flip_mode_aux).minimum_capture_rectangle(scale)};
    aux_capture_rect = abs_rectangle_to_calib(abs_aux_capture_rect, flip_mode_aux);
    SLOGF(SLOG_INFO, "Abs aux capture rectangle: left_x: %d, top_y: %d, width: %d, height: %d",
            abs_aux_capture_rect.left_x(), abs_aux_capture_rect.top_y(), abs_aux_capture_rect.width(), abs_aux_capture_rect.height());
    SLOGF(SLOG_INFO, "Updated aux capture rectangle: left_x: %d, top_y: %d, width: %d, height: %d",
            aux_capture_rect.left_x(), aux_capture_rect.top_y(), aux_capture_rect.width(), aux_capture_rect.height());
    afs_aux.mipi_rx_chid = afs_aux.cam->info.ch - 1;
    afs_aux.x_position = abs_aux_capture_rect.left_x();
    afs_aux.y_position = abs_aux_capture_rect.top_y();
    afs_aux.x_width = abs_aux_capture_rect.width();
    afs_aux.y_width = abs_aux_capture_rect.height();
    // Allocate DDR memory for aux roi capture
    std::size_t aux_roi_img_size = aux_capture_rect.image_size(scale);
    std::unique_ptr<char[]> aux_roi_img_ptr{new(Heap::DDR) char[aux_roi_img_size]};
    SLOGF(SLOG_DEBUG, "Aux capture image addr: %x, size: %d", (int)aux_roi_img_ptr.get(), (int)aux_roi_img_size);
    afs_aux.destination = reinterpret_cast<std::uint32_t>(aux_roi_img_ptr.get());
    afs_aux.af_cb = &aux_roi_mipi_callback;
    afs_aux.cb_user_data = this;
    afs_aux.cam_settings = afs_pri.cam_settings;
    afs_aux.scale = afs_pri.scale;
    // Configure pri sensor for captures
    af_configure_sensor_stream(&afs_aux, false);

    // Wait for lens moves done before starting capture
    xSemaphoreTake(pri_capture_sem_, portMAX_DELAY);
    xSemaphoreTake(aux_capture_sem_, portMAX_DELAY);
    // Trigger captures
    trigger_sensor(&afs_pri);
    trigger_sensor(&afs_aux);
    // Wait for captures done
    xSemaphoreTake(pri_capture_sem_, portMAX_DELAY);
    xSemaphoreTake(aux_capture_sem_, portMAX_DELAY);

    SLOGF(SLOG_INFO, "Both images captured, will compute depth");

    // Invalidate cache to ensure images are read from DDR
    invalidate_cache(afs_pri.destination, pri_roi_img_size);
    invalidate_cache(afs_aux.destination, aux_roi_img_size);

    // Compute depth
    rectangle pri_capture_rect{abs_rectangle_to_calib(abs_pri_capture_rect, flip_mode_pri)};
    rectangle aux_roi;
    float depth = df.computeDepth(pri_roi_img_ptr.get(), pri_capture_rect, pri_roi,
                                  aux_roi_img_ptr.get(), aux_capture_rect, capture_mode, scale, &aux_roi);
    SLOGF(SLOG_INFO, "Computed depth: %f", depth);

    SLOGF(SLOG_DEBUG, "Aux roi: left_x: %d, top_y: %d, width: %d, height: %d",
            aux_roi.left_x(), aux_roi.top_y(), aux_roi.width(), aux_roi.height());

#if 0
    SLOGF(SLOG_DEBUG, "Primary roi:");
    debug_print_image(pri_roi_img_ptr.get(), pri_capture_rect, pri_roi, flip_mode_pri, scale);

    SLOGF(SLOG_DEBUG, "Aux capture:");
    debug_print_image(aux_roi_img_ptr.get(), aux_capture_rect, aux_capture_rect, flip_mode_aux, scale);

    SLOGF(SLOG_INFO, "Aux roi:", depth);
    debug_print_image(aux_roi_img_ptr.get(), aux_capture_rect, aux_roi, flip_mode_aux, scale);
#endif

    depth = std::min(depth, 65535.0f);
    depth = std::max(depth, 100.0f);
    std::uint32_t distance_mm = depth;
    std::uint8_t ref_mod_idx = 1; // A1
    roi_rectangle_t pri_roi_r{pri_roi.left_x(), pri_roi.top_y(), pri_roi.width(), pri_roi.height()};
    light_system->settings->roi_transfer.distance_mm = distance_mm;
    light_system->settings->roi_transfer.ref_mod_idx = ref_mod_idx;
    light_system->settings->roi_transfer.ref_mod_roi = pri_roi_r;

    // Send command to transfer ROI
    std::uint32_t m_bitmask;
    std::size_t num_cam_transfer;
    constexpr std::size_t max_num_cam_transfer = 12;
    if (is_ab_mode) {
        // Transfer ROI to As, Bs and C5
        m_bitmask = 0x87fe;
        num_cam_transfer = 11;
    } else {
        // Transfer ROI to A1, all Bs and all Cs
        m_bitmask = 0x1ffc2;
        num_cam_transfer = 12;
    }
    constexpr std::size_t roi_transfer_fixed_size = 7;
    constexpr std::size_t roi_transfer_var_size = 13;
    lcc_cmd_info_t cmd;
    cmd.word = LCC_CMD_CAM_ROI_TRANSFER;
    std::uint8_t roi_transfer_msg[roi_transfer_fixed_size + max_num_cam_transfer * roi_transfer_var_size];
    roi_transfer_msg[0] = 0;
    roi_transfer_msg[1] = 0;
    roi_transfer_msg[2] = cmd.cmd;
    roi_transfer_msg[3] = cmd.base;
    roi_transfer_msg[4] = m_bitmask & 0xff;
    roi_transfer_msg[5] = (m_bitmask >> 8) & 0xff;
    roi_transfer_msg[6] = (m_bitmask >> 16) & 0xff;
    std::memcpy(roi_transfer_msg + 7, &distance_mm, 4);
    roi_transfer_msg[11] = ref_mod_idx;    // reference module A1
    std::memcpy(roi_transfer_msg + 12, &pri_roi_r, sizeof(pri_roi_r));

    for (std::size_t i = 1; i < num_cam_transfer; ++i) {
        std::memcpy(roi_transfer_msg + roi_transfer_fixed_size + roi_transfer_var_size*i,
                roi_transfer_msg + roi_transfer_fixed_size, roi_transfer_var_size);
    }
    const std::size_t roi_transfer_msg_size = roi_transfer_fixed_size + num_cam_transfer * roi_transfer_var_size;
    i2cm.transceiver(I2C_FORWARD_ASIC2_CHANNEL, I2C_FORWARD_SLAVEADDR, roi_transfer_msg, roi_transfer_msg_size, NULL, 0);
    i2cm.transceiver(I2C_FORWARD_ASIC3_CHANNEL, I2C_FORWARD_SLAVEADDR, roi_transfer_msg, roi_transfer_msg_size, NULL, 0);

    m_bitmask &= light_system->m_bitmask_all;
    std::uint32_t cam_channel_bitmask = 0;
    while (m_bitmask != 0) {
        int cam_idx = __builtin_ctz(m_bitmask);
        m_bitmask &= ~(1 << cam_idx);
        std::uint32_t ch = idx_to_object(cam_idx)->info.ch - 1;
        cam_channel_bitmask |= 1 << ch;
    }
    af_roi_transfer_execute(cam_channel_bitmask);
#endif
}

void af_ae::pri_mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq) {
    if ((signal_irq & R1_VC0_FRAME_END) != 0) {
        BaseType_t higher_pri_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(pri_capture_sem_, &higher_pri_task_woken);
        portEND_SWITCHING_ISR(higher_pri_task_woken);
    }
}

void af_ae::aux_mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq) {
    if ((signal_irq & R1_VC0_FRAME_END) != 0) {
        BaseType_t higher_pri_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(aux_capture_sem_, &higher_pri_task_woken);
        portEND_SWITCHING_ISR(higher_pri_task_woken);
    }
}
