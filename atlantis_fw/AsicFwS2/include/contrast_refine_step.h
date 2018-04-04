/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_refine_step.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-8-2016
 * @brief   Implements one step of contrast refinement iteration
 *
 ******************************************************************************/

#ifndef __CONTRAST_REFINE_STEP_H__
#define __CONTRAST_REFINE_STEP_H__

#include "os.h"
#include "af_helper.h"
#include "lens_position_control.h"
#include "rectangle.h"
#include "contrast_processor.h"

#ifdef __cplusplus

#include <atomic>
#include <cstdint>

extern "C" void crs_mipi2axi_callback(uint8_t iidx, uint32_t signal_irq_detail,
        uint32_t error_irq_detail, void *user_data);

void lens_movement_done_cb(void* user_data, bool success, std::int16_t pos);

/**
 * Implements one step of contrast refinement iteration
 */
class contrast_refine_step {
    friend void crs_mipi2axi_callback(uint8_t iidx, uint32_t signal_irq_detail,
            uint32_t error_irq_detail, void *user_data);

    friend void lens_movement_done_cb(void* user_data, bool success, std::int16_t pos);

public:
    /**
     * Constructor
     * @param cam Camera module object
     * @param scale Set to 1 for native resolution, 2 for scale/bin of 2 in x and y
     * @param min_capture_rect Minimum rectangle that needs to be captured to fit ROIs
     * @param event_group Event group for lens move done and capture done events
     * @param lens_move_done_event Event to signal that lens move is completed
     * @param capture_done_event Event to signal that roi image capture is completed
     */
    contrast_refine_step(cam_typedef_t& cam, std::uint16_t scale,
            const rectangle& min_capture_rect, void *image_buffer_p,
            EventGroupHandle_t event_group,
            EventBits_t lens_move_done_event, EventBits_t capture_done_event);

    ~contrast_refine_step();

    /**
     * Start moving lens
     * @param pos Target lens position
     * @param tolerance Tolerance for target lens position
     */
    void start_move_lens(std::int16_t pos, std::uint16_t tolerance);

    /**
     *  Start capturing frame
     */
    void start_capture_frame();

    /**
     * Compute sharpness metric
     */
    void compute_sharpness_metric(const rectangle& roi, std::uint32_t metrics_mask);

    /**
     * @return Actual measurement lens position
     */
    std::int16_t measurement_position() const {
        return measurement_position_;
    }

    using metric_type = contrast_processor::metric_type;
    using metrics_per_color = contrast_processor::metrics_per_color;

    /**
     * @return Return sharpness metric
     */
    contrast_processor::metrics_per_color get_metric(metric_type metric) const {
        return metrics_arr_[metric];
    }

    /**
     * @return Return normalized gradient sharpness metric
     */
    std::int64_t sharpness_metric_gradient_norm() const {
        return sharpness_metric_gradient_norm_;
    }

    /**
     * @return Return variance metric
     */
    std::uint64_t sharpness_metric_variance() const {
        return sharpness_metric_variance_;
    }

    cam_typedef_t* camera() {
        return afs_.cam;
    }

    const rectangle& capture_rectangle() const {
        return capture_rectangle_;
    }

private:

    void mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq);

    void lens_movement_done_cb(bool success, std::int16_t pos);

    void frame_captured_cb();

    void compute_sharpness_metric();

    void log_metrics(std::uint32_t metrics_mask);

    std::uint16_t frame_capture_timeout_ms();

    lens_position_control len_pos_control_;
    rectangle capture_rectangle_{0,0,0,0};
    af_setting_t afs_;
    std::atomic<bool> is_capture_ongoing_{false};
    contrast_processor::metrics_array metrics_arr_;
    std::int64_t sharpness_metric_gradient_norm_{0};
    std::uint64_t sharpness_metric_variance_{0};
    EventGroupHandle_t event_group_;
    EventBits_t lens_move_done_event_;
    EventBits_t capture_done_event_;
    std::int16_t measurement_position_{0};
    void* image_ptr_{nullptr};
};

#endif
#endif /* __CONTRAST_REFINE_STEP_H__ */
