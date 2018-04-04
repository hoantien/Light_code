/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    uniform_contrast_pass.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass with uniformly spaced measurements
 *
 ******************************************************************************/

#include "log.h"
#include "uniform_contrast_pass.h"

#define SLOGF_ID                SLOG_ID_AF

uniform_contrast_pass::uniform_contrast_pass(std::int16_t start_pos, std::int16_t end_pos,
        std::int16_t delta, std::uint16_t tolerance, bool use_metric1, bool interval_shrink) :
            start_pos_{start_pos},
            end_pos_{end_pos},
            delta_{delta},
            tolerance_{tolerance},
            next_pos_{start_pos},
            use_metric1_{use_metric1},
            interval_shrink_{interval_shrink} {
}

void uniform_contrast_pass::initialize(const contrast_pass& prev_pass) {
    // Update start and end positions
    start_pos_ = prev_pass.out_min_pos();
    end_pos_ = prev_pass.out_max_pos();
    // Compute initial next_pos_
    initialize();
}

void uniform_contrast_pass::initialize() {
    // Reset best position/metric before each pass
    best_meas_.pos = start_pos_;
    best_meas_.metric1 = METRIC_MIN;
    best_meas_.metric2 = METRIC_MIN;

    std::int32_t pos_range = end_pos_;
    pos_range -= start_pos_;
    std::uint16_t num_steps = static_cast<std::uint16_t>((pos_range + delta_ - 1) / delta_);
    if (num_steps == 0)
        num_steps = 1;  // do 1 step even if (start_pos == end_pos)
    std::uint16_t tot_d = (num_steps - 1) * delta_;
    std::int16_t start_d = static_cast<std::int16_t>((pos_range - tot_d) / 2);
    next_pos_ = start_pos_ + start_d;
    SLOGF(SLOG_INFO, "Starting contrast sweep pass with start_pos: %d, end_pos: %d, delta: %d", (int)start_pos_, (int)end_pos_, (int)delta_);
}

bool uniform_contrast_pass::is_done() const {
    return (next_pos_ > end_pos_);
}

std::int16_t uniform_contrast_pass::next_pos() {
    return next_pos_;
}

std::uint16_t uniform_contrast_pass::tolerance() const {
    return tolerance_;
}

void uniform_contrast_pass::set_measurement(const measurement_t& meas) {
    if (measurement_t::is_better(use_metric1_, meas, best_meas_))
        best_meas_ = meas;
    next_pos_ += delta_;
}

std::int16_t uniform_contrast_pass::out_min_pos() const {
    // New start position
    std::int16_t d = delta_;
    if (interval_shrink_)
        d = (d + 1) / 2;
    std::int16_t best_pos = best_meas_.pos;
    // Limit next pass start to requested interval [start_pos, end_pos]
    std::int16_t sp = (best_pos >= (start_pos_ + d)) ? (best_pos - d) : start_pos_;
    if (sp > end_pos_)
        sp = end_pos_;
    return sp;
}

std::int16_t uniform_contrast_pass::out_max_pos() const {
    // New end position
    std::int16_t d = delta_;
    if (interval_shrink_)
        d = (d + 1) / 2;
    std::int16_t best_pos = best_meas_.pos;
    // Limit next pass end to requested interval [start_pos, end_pos]
    std::int16_t ep = (best_pos <= (end_pos_ - d)) ? best_pos + d : end_pos_;
    if (ep < start_pos_)
        ep = start_pos_;
    return ep;
}

std::int16_t uniform_contrast_pass::out_best_pos() const {
    return best_meas_.pos;
}

std::int64_t uniform_contrast_pass::out_best_metric() const {
    return use_metric1_ ? best_meas_.metric1 : best_meas_.metric2;
}

std::size_t uniform_contrast_pass::num_measurements() const {
    return 0;
}

auto uniform_contrast_pass::measurements() const -> const measurement_t* {
    return nullptr;
}
