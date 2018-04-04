/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    max_search_contrast_pass.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass which searches for a local maximum
 *
 ******************************************************************************/

#ifndef __MAX_SEARCH_CONTRAST_PASS_H__
#define __MAX_SEARCH_CONTRAST_PASS_H__

#include <array>
#include "contrast_pass.h"

class max_search_contrast_pass: public contrast_pass {
public:

    max_search_contrast_pass(std::int16_t start_pos, std::int16_t delta, std::uint16_t tolerance, bool use_metric1);

    void initialize(const contrast_pass& prev_pass) override;

    void initialize() override;

    bool is_done() const override;

    std::int16_t next_pos() override;

    std::uint16_t tolerance() const override;

    void set_measurement(const measurement_t& meas) override;

    std::int16_t out_min_pos() const override;

    std::int16_t out_max_pos() const override;

    std::int16_t out_best_pos() const override;

    std::int64_t out_best_metric() const override;

    std::size_t num_measurements() const override;

    const measurement_t* measurements() const override;

private:
    enum class State {
        IDLE,
        STARTING,
        MOVING_UP,
        MOVING_DOWN,
        DONE
    };
    std::int16_t start_pos_{0};
    std::int16_t delta_{1};
    std::uint16_t tolerance_{0};
    bool use_metric1_{true};
    std::int16_t next_pos_{0};
    measurement_t best_meas_{0, 0, 0};
    std::array<measurement_t, 2> meas_arr_;
    State state_{State::IDLE};
};

#endif /* __MAX_SEARCH_CONTRAST_PASS_H__ */
