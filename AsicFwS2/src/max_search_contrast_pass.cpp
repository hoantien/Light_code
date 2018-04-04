/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    max_search_contrast_pass.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass which searches for a local maximum
 *
 ******************************************************************************/

#include "log.h"
#include "max_search_contrast_pass.h"

#define SLOGF_ID                SLOG_ID_AF

max_search_contrast_pass::max_search_contrast_pass(std::int16_t start_pos, std::int16_t delta, std::uint16_t tolerance,
        bool use_metric1) :
            start_pos_{start_pos},
            delta_{delta},
            tolerance_{tolerance},
            use_metric1_{use_metric1},
            next_pos_{start_pos} {
}

void max_search_contrast_pass::initialize(const contrast_pass& prev_pass) {
    // Update start and end positions
    start_pos_ = prev_pass.out_best_pos();
    // Compute initial next_pos_
    initialize();
}

void max_search_contrast_pass::initialize() {
    // Reset best position/metric before each pass
    best_meas_.pos = start_pos_;
    best_meas_.metric1 = METRIC_MIN;
    best_meas_.metric2 = METRIC_MIN;

    next_pos_ = start_pos_;
    state_ = State::IDLE;

    SLOGF(SLOG_INFO, "Starting contrast max search pass with start_pos: %d, delta: %d", (int)start_pos_, (int)delta_);
}

bool max_search_contrast_pass::is_done() const {
    return (state_ == State::DONE);
}

std::int16_t max_search_contrast_pass::next_pos() {
    return next_pos_;
}

std::uint16_t max_search_contrast_pass::tolerance() const {
    return tolerance_;
}

void max_search_contrast_pass::set_measurement(const measurement_t& meas) {
    if (state_ == State::IDLE) {
        best_meas_ = meas;
        state_ = State::STARTING;
        next_pos_ = start_pos_ + delta_;
        meas_arr_[0] = meas;
    } else {
        bool is_new_best{measurement_t::is_better(use_metric1_, meas, best_meas_)};
        if (is_new_best)
            best_meas_ = meas;
        if (state_ == State::STARTING) {
            meas_arr_[1] = meas;
            if (is_new_best)
                state_ = State::MOVING_UP;
            else
                state_ = State::MOVING_DOWN;
        } else {
            // State is MOVING_UP or MOVING_DOWN
            std::size_t idx_best = (state_ == State::MOVING_UP) ? 1 : 0;
            std::size_t idx_second = 1 - idx_best;
            bool meas_is_third = !(is_new_best || measurement_t::is_better(use_metric1_, meas, meas_arr_[idx_second]));
            if (!meas_is_third) {
                meas_arr_[idx_second] = meas_arr_[idx_best];
                meas_arr_[idx_best] = meas;
            }
            if (!is_new_best)
                state_ = State::DONE;
        }
        if (state_ == State::MOVING_UP)
            next_pos_ = meas_arr_[1].pos + delta_;
        else if (state_ == State::MOVING_DOWN)
            next_pos_ = meas_arr_[0].pos - delta_;
    }
}

std::int16_t max_search_contrast_pass::out_min_pos() const {
    // New start position
    return meas_arr_[0].pos;
}

std::int16_t max_search_contrast_pass::out_max_pos() const {
    // New end position
    return meas_arr_[1].pos;
}

std::int16_t max_search_contrast_pass::out_best_pos() const {
    return best_meas_.pos;
}

std::int64_t max_search_contrast_pass::out_best_metric() const {
    return use_metric1_ ? best_meas_.metric1 : best_meas_.metric2;
}

std::size_t max_search_contrast_pass::num_measurements() const {
    return meas_arr_.size();
}

auto max_search_contrast_pass::measurements() const -> const measurement_t* {
    return meas_arr_.data();
}
