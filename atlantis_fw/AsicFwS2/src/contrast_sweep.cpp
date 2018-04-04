/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_sweep.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-8-2016
 * @brief   Implements contrast optimization full sweep
 *
 ******************************************************************************/

#include "assert.h"
#include "log.h"
#include "contrast_sweep.h"

#define SLOGF_ID                SLOG_ID_AF


contrast_sweep::contrast_sweep(cam_typedef_t& cam, std::uint16_t scale,
        const rectangle& min_capture_rect, void* image_buffer_p,
        EventGroupHandle_t event_group,
        EventBits_t lens_move_done_event, EventBits_t capture_done_event) :
            contrast_step_(cam, scale, min_capture_rect, image_buffer_p,
                    event_group, lens_move_done_event, capture_done_event) {
}

void contrast_sweep::add_iter_param(contrast_processor::metric_type metric1_t,
        contrast_processor::metric_type metric2_t,
        std::unique_ptr<contrast_pass> contrast_pass_p) {
    assert((num_pass_ + 1) < MAX_NUM_PASS);
    pass_param_arr_[num_pass_] = pass_param{metric1_t, metric2_t, std::move(contrast_pass_p)};
    ++num_pass_;
}

void contrast_sweep::initialize_sweep(const rectangle& roi) {
    roi_ = roi;
    pass_i_ = 0;
    if (num_pass_ != 0)
        pass_param_arr_[0].contrast_pass_p->initialize();
}

void contrast_sweep::check_initialize_next_pass() {
    const contrast_pass& cp = *pass_param_arr_[pass_i_].contrast_pass_p;
    if (cp.is_done()) {
        SLOGF(SLOG_INFO, "Contrast sweep pass completed, best_pos: %d, best_metric: %lld",
                (int)cp.out_best_pos(), (long long int)cp.out_best_metric());
        ++pass_i_;
        if (pass_i_ != num_pass_)
            pass_param_arr_[pass_i_].contrast_pass_p->initialize(cp);
    }
}

void contrast_sweep::start_lens_move() {
    if (pass_i_ == num_pass_) {
        std::int16_t best_pos = pass_param_arr_[num_pass_-1].contrast_pass_p->out_best_pos();
        contrast_step_.start_move_lens(best_pos, 0);
        cam_typedef_t& cam = *contrast_step_.camera();
        lens_position_control lpc{cam};
        std::uint32_t focus_d_mm = lpc.get_object_distance_mm(best_pos, 300); // FIXME temperature
        SLOGF(SLOG_DEBUG, "Contrast sweep completed, moving %X to best lens position %d which focuses at object distance %umm",
                cam.info.module, best_pos, (unsigned int)focus_d_mm);
    } else {
        contrast_pass& cp = *pass_param_arr_[pass_i_].contrast_pass_p;
        std::int16_t next_pos = cp.next_pos();
        contrast_step_.start_move_lens(next_pos, cp.tolerance());
    }
}

void contrast_sweep::start_capture_frame() {
    contrast_step_.start_capture_frame();
}

bool contrast_sweep::process_captured_frame() {
    contrast_pass::measurement_t meas;
    meas.pos = contrast_step_.measurement_position();
    SLOGF(SLOG_DEBUG, "Will compute contrast metrics at position %d", (int)meas.pos);
    std::uint32_t metrics_mask = 1 << pass_param_arr_[pass_i_].metric1_t;
    metrics_mask |= 1 << pass_param_arr_[pass_i_].metric2_t;
    metrics_mask |= 1 << contrast_processor::FILTERED_GRADIENT_TH;
    metrics_mask |= 1 << contrast_processor::GRADIENT_TH;
    metrics_mask |= 1 << contrast_processor::INTENSITY;
    contrast_step_.compute_sharpness_metric(roi_, metrics_mask);
    std::int64_t metric_gnorm = contrast_step_.sharpness_metric_gradient_norm();
    std::uint64_t metric_gth = contrast_step_.get_metric(contrast_processor::GRADIENT_TH).total_metric();
    std::uint64_t metric_g = contrast_step_.get_metric(contrast_processor::GRADIENT).total_metric();
    std::uint64_t metric_var = contrast_step_.sharpness_metric_variance();
    std::uint64_t metric_d4gth = contrast_step_.get_metric(contrast_processor::DECIMATED4_GRADIENT_TH).total_metric();
    std::uint64_t metric_fgth = contrast_step_.get_metric(contrast_processor::FILTERED_GRADIENT_TH).total_metric();
    meas.metric1 = contrast_step_.get_metric(pass_param_arr_[pass_i_].metric1_t).total_metric();
    meas.metric2 = contrast_step_.get_metric(pass_param_arr_[pass_i_].metric2_t).total_metric();
    SLOGF(SLOG_DEBUG, "Metric at position %d is %lld, %lld (%lld, %lld, %lld, %lld, %lld, %lld)", (int)meas.pos,
            meas.metric1, meas.metric2, metric_gth, metric_g, metric_d4gth, metric_fgth, metric_gnorm, metric_var);

    pass_param_arr_[pass_i_].contrast_pass_p->set_measurement(meas);
    check_initialize_next_pass();
    return (pass_i_ == num_pass_);
}
