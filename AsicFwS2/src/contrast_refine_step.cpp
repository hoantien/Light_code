/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_refine_step.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-8-2016
 * @brief   Implements one step of contrast refinement iteration
 *
 ******************************************************************************/

#include <cstring>
#include <memory>
#include "cortex_r4.h"
#include "board_config.h"
#include "log.h"
#include "hal_mipi2axi.h"
#include "af_helper.h"
#include "image_raw10.h"
#include "contrast_refine_step.h"

#define SLOGF_ID                SLOG_ID_AF


contrast_refine_step::contrast_refine_step(cam_typedef_t& cam, std::uint16_t scale,
        const rectangle& min_capture_rect, void* image_buffer_p,
        EventGroupHandle_t event_group,
        EventBits_t lens_move_done_event, EventBits_t capture_done_event) :
                len_pos_control_(cam),
                capture_rectangle_{min_capture_rect},
                event_group_{event_group},
                lens_move_done_event_{lens_move_done_event},
                capture_done_event_{capture_done_event},
                image_ptr_{image_buffer_p} {
    afs_.cam = &cam;
    afs_.mipi_rx_chid = afs_.cam->info.ch - 1;
    afs_.x_position = capture_rectangle_.left_x();
    afs_.y_position = capture_rectangle_.top_y();
    afs_.x_width = capture_rectangle_.width();
    afs_.y_width = capture_rectangle_.height();
    afs_.destination = reinterpret_cast<std::uint32_t>(image_ptr_);
    afs_.af_cb = &crs_mipi2axi_callback;
    afs_.cb_user_data = this;
    afs_.flip_mode = FLIP_MODE_NONE;
    // Exposure, fps, resolution, sensitivity settings...
    std::uint64_t exposure_ns{0};
    float gain{1.0f};
    std::memcpy(&exposure_ns, cam.image->settings[UC_FOCAL_STACKING]->exposure, sizeof(exposure_ns));
    std::memcpy(&gain, cam.image->settings[UC_FOCAL_STACKING]->sensitivity, sizeof(gain));
    if (exposure_ns == 0) {
        std::memcpy(&exposure_ns, cam.image->settings[UC_PREVIEW]->exposure, sizeof(exposure_ns));
        std::memcpy(&gain, cam.image->settings[UC_PREVIEW]->sensitivity, sizeof(gain));
    }
    if (exposure_ns == 0) {
        exposure_ns = 30000000;
        gain = 1.0f;
    }
    std::memcpy(afs_.cam_settings.exposure, &exposure_ns, sizeof(afs_.cam_settings.exposure));
    std::uint16_t fps = 30;
    std::memcpy(afs_.cam_settings.fps, &fps, sizeof(afs_.cam_settings.fps));
    std::uint32_t res[2];
    if (scale == 1) {
        res[0] = X_13M;
        res[1] = Y_13M;
    } else {
        res[0] = X_3M_P2;
        res[1] = Y_3M_P2;
    }
    std::memcpy(afs_.cam_settings.resolution, &res, sizeof(afs_.cam_settings.resolution));
    afs_.scale = scale;
    std::memcpy(afs_.cam_settings.sensitivity, &gain, sizeof(afs_.cam_settings.sensitivity));
    // Configure sensor for captures
    af_configure_sensor_stream(&afs_, false);
}

contrast_refine_step::~contrast_refine_step() {
}

namespace {
    std::atomic<std::uint32_t> num_pending_frame_end_callbacks{0};
}

extern "C" void crs_mipi2axi_callback(uint8_t iidx, uint32_t signal_irq,
        uint32_t error_irq, void *user_data) {
    if (num_pending_frame_end_callbacks <= 0) {
        printf("\r\nError: crs_mipi2axi_callback called when no capture requested\r\n");
        return;
    }
    static_cast<contrast_refine_step*>(user_data)->mipi2axi_cb(iidx, signal_irq, error_irq);
}

void lens_movement_done_cb(void* user_data, bool success, std::int16_t pos) {
    static_cast<contrast_refine_step*>(user_data)->lens_movement_done_cb(success, pos);
}

void contrast_refine_step::start_move_lens(std::int16_t pos, std::uint16_t tolerance) {
    // Send move request with callback
    len_pos_control_.start_move_lens(pos, tolerance,
            lens_position_control::move_done_callback_t{::lens_movement_done_cb, this});
}

void contrast_refine_step::start_capture_frame() {
    // Start capture of one frame
    is_capture_ongoing_ = true;
    num_pending_frame_end_callbacks++;
    trigger_sensor(&afs_);
}

void contrast_refine_step::lens_movement_done_cb(bool success, std::int16_t pos) {
    measurement_position_ = pos;
    xEventGroupSetBits(event_group_, lens_move_done_event_);
}

void contrast_refine_step::frame_captured_cb() {
    if (!is_capture_ongoing_) {
        printf("\r\nError: frame_captured_cb called when no capture requested\r\n");
        return;
    }
    num_pending_frame_end_callbacks--;
    is_capture_ongoing_ = false;
    BaseType_t higher_pri_task_woken = pdFALSE;
    xEventGroupSetBitsFromISR(event_group_, capture_done_event_, &higher_pri_task_woken);
    portEND_SWITCHING_ISR(higher_pri_task_woken);
}

void contrast_refine_step::mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq) {
    if ((signal_irq & R1_VC0_FRAME_END) != 0)
        frame_captured_cb();
}

namespace {
#if 0
    void remove_data_pedestal(std::uint64_t* value, std::uint64_t pedestal) {
        if (*value <= pedestal)
            *value = 0;
        else
            *value -= pedestal;
    }

    void remove_data_pedestal(contrast_refine_step::metrics_per_color* metric, std::uint64_t pedestal) {
        remove_data_pedestal(&metric->green, pedestal);
        remove_data_pedestal(&metric->red, pedestal);
        remove_data_pedestal(&metric->blue, pedestal);
    }
#endif

    void remove_data_pedestal1(std::uint64_t* value, std::uint64_t pedestal) {
        if (*value <= pedestal)
            *value = 1;
        else
            *value -= pedestal;
    }

    void remove_data_pedestal1(contrast_refine_step::metrics_per_color* metric, std::uint64_t pedestal) {
        remove_data_pedestal1(&metric->green, pedestal);
        remove_data_pedestal1(&metric->red, pedestal);
        remove_data_pedestal1(&metric->blue, pedestal);
    }
}

void contrast_refine_step::compute_sharpness_metric(const rectangle& roi, std::uint32_t metrics_mask) {
    // Compute metric on ROI
    std::uint16_t scale = afs_.scale;
    std::uint16_t x_scaled_offset = (roi.left_x() - capture_rectangle_.left_x() + scale - 1) / scale;
    x_scaled_offset = (x_scaled_offset + 1) / 2 * 2; // must be even to preserve Bayer pattern
    std::uint16_t right_x_offset = roi.left_x() + roi.width() - capture_rectangle_.left_x();
    std::uint16_t roi_scaled_width = right_x_offset / scale - x_scaled_offset;
    std::unique_ptr<contrast_processor> cp{new contrast_processor(0, roi_scaled_width/2)};
    cp->enable_metrics(metrics_mask);
    //cp->enable_metric(contrast_processor::GRADIENT);
    //cp->enable_metric(contrast_processor::INTENSITY);
    image_raw10 img(image_ptr_, capture_rectangle_.width()/scale);
    std::uint16_t y_scaled_offset = (roi.top_y() - capture_rectangle_.top_y() + scale - 1) / scale;
    y_scaled_offset = (y_scaled_offset + 1) / 2 * 2; // must be even to preserve Bayer pattern
    std::uint16_t bottom_y_offset = roi.top_y() + roi.height() - capture_rectangle_.top_y();
    std::uint16_t roi_scaled_height = bottom_y_offset / scale - y_scaled_offset;
    for (std::uint16_t y = 0; y < roi_scaled_height; ++y) {
        cp->process_line(img.get_iter(y + y_scaled_offset, x_scaled_offset));
    }

    metrics_arr_ = cp->get_metrics_array();

    const metrics_per_color& metric_i = metrics_arr_[contrast_processor::INTENSITY];
    const metrics_per_color& metric_e = metrics_arr_[contrast_processor::ENERGY];
    std::uint32_t roi_width_color = roi.width() / 2;
    std::uint32_t roi_height_green = roi.height();
    std::uint32_t roi_height_rb = roi_height_green / 2;
    std::uint32_t roi_area_green = roi_width_color * roi_height_green;
    std::uint32_t roi_area_rb = roi_width_color * roi_height_rb;
    metrics_per_color metric_iavg;
    metric_iavg.green = metric_i.green / roi_area_green; // per pixel average value
    metric_iavg.red = metric_i.red / roi_area_rb;
    metric_iavg.blue = metric_i.blue / roi_area_rb;
    metrics_per_color metric_eavg;
    metric_eavg.green = metric_e.green / roi_area_green;
    metric_eavg.red = metric_e.red / roi_area_rb;
    metric_eavg.blue = metric_e.blue / roi_area_rb;
    metrics_per_color metric_var;
    metric_var.green = metric_eavg.green - metric_iavg.green * metric_iavg.green;
    metric_var.red = metric_eavg.red - metric_iavg.red * metric_iavg.red;
    metric_var.blue = metric_eavg.blue - metric_iavg.blue * metric_iavg.blue;

#if 1
    const std::uint16_t data_pedestal = 42;
    remove_data_pedestal1(&metric_iavg, data_pedestal);

    std::uint32_t roi_area_green_metric_g = (roi_width_color-1) * (roi_height_green-1);
    const std::uint64_t quant_noise2 = 3;
    const std::int32_t digital_to_photon_q16 = (static_cast<std::int32_t>(1) << 16) * 10 / 55;
    std::uint64_t green_g_noise = 4 * roi_area_green_metric_g
            * (((metric_iavg.green * digital_to_photon_q16) >> 16) + quant_noise2);
    std::int64_t iavg = metric_iavg.total_metric();
    const metrics_per_color& metric_g = metrics_arr_[contrast_processor::GRADIENT];
    std::int64_t green_gnorm = metric_g.green - green_g_noise;
    green_gnorm = (green_gnorm / iavg) * 600 / iavg * 600 / iavg;

    std::uint32_t roi_area_rb_metric_g = (roi_width_color-1) * (roi_height_rb-1);
    std::uint64_t red_g_noise = 4 * roi_area_rb_metric_g
            * (((metric_iavg.red * digital_to_photon_q16) >> 16) + quant_noise2);
    std::int64_t red_gnorm = metric_g.red - red_g_noise;
    red_gnorm = (red_gnorm / iavg) * 600 / iavg * 600 / iavg;

    std::uint64_t blue_g_noise = 4 * roi_area_rb_metric_g
            * (((metric_iavg.blue * digital_to_photon_q16) >> 16) + quant_noise2);
    std::int64_t blue_gnorm = metric_g.blue - blue_g_noise;
    blue_gnorm = (blue_gnorm / iavg) * 600 / iavg * 600 / iavg;
#endif

    sharpness_metric_gradient_norm_ = green_gnorm + red_gnorm + blue_gnorm;
    sharpness_metric_variance_ = metric_var.total_metric();

    //log_metrics(metrics_mask);

#if 0
    // PAA_DEBUG Copy image:
    static char* dst = reinterpret_cast<char*>(DDR_BASE + 0x8000000);
    const char* src = static_cast<const char*>(img.get_iter(0,0).current_pixelp());
    const char* src_end = static_cast<const char*>(img.get_iter(capture_rectangle_.height(),0).current_pixelp());
    std::size_t img_size = src_end - src;
    std::memcpy(dst, src, img_size);
    SLOGF(SLOG_DEBUG, "Copied image to address 0x%08llx, size: %d", reinterpret_cast<std::uint64_t>(dst), (int)img_size);
    dst += 0x1400000;
#endif
}

void contrast_refine_step::log_metrics(std::uint32_t metrics_mask) {
    const metrics_per_color& mgth = metrics_arr_[contrast_processor::GRADIENT_TH];
    const metrics_per_color& mg = metrics_arr_[contrast_processor::GRADIENT];
    const metrics_per_color& mi = metrics_arr_[contrast_processor::INTENSITY];
    const metrics_per_color& me = metrics_arr_[contrast_processor::ENERGY];
    const metrics_per_color& ml2 = metrics_arr_[contrast_processor::LAPLACIAN2];
    const metrics_per_color& md4gth = metrics_arr_[contrast_processor::DECIMATED4_GRADIENT_TH];
    const metrics_per_color& mfgth = metrics_arr_[contrast_processor::FILTERED_GRADIENT_TH];

    SLOGF(SLOG_DEBUG, "CAM-%X Contrast metrics green:  %11llu, %11llu, %11llu, %11llu, %11llu, %11llu, %11llu",
            afs_.cam->info.module, mgth.green, mg.green, mi.green, me.green, ml2.green, md4gth.green, mfgth.green);
    SLOGF(SLOG_DEBUG, "CAM-%X Contrast metrics red:    %11llu, %11llu, %11llu, %11llu, %11llu, %11llu, %11llu",
            afs_.cam->info.module, mgth.red, mg.red, mi.red, me.red, ml2.red, md4gth.red, mfgth.red);
    SLOGF(SLOG_DEBUG, "CAM-%X Contrast metrics blue:   %11llu, %11llu, %11llu, %11llu, %11llu, %11llu, %11llu",
            afs_.cam->info.module, mgth.blue, mg.blue, mi.blue, me.blue, ml2.blue, md4gth.blue, mfgth.blue);
    SLOGF(SLOG_DEBUG, "CAM-%X Contrast metrics gnorm, var: %lld, %llu",
            afs_.cam->info.module, sharpness_metric_gradient_norm_, sharpness_metric_variance_);
}
