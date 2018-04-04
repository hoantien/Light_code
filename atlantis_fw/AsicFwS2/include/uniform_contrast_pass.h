/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    uniform_contrast_pass.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass with uniformly spaced measurements
 *
 ******************************************************************************/

#ifndef __UNIFORM_CONTRAST_PASS_H__
#define __UNIFORM_CONTRAST_PASS_H__

#include "contrast_pass.h"

class uniform_contrast_pass: public contrast_pass {
public:

    uniform_contrast_pass(std::int16_t start_pos, std::int16_t end_pos,
            std::int16_t delta, std::uint16_t tolerance, bool use_metric1, bool interval_shrink);

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
    std::int16_t start_pos_{0};
    std::int16_t end_pos_{0};
    std::int16_t delta_{1};
    std::uint16_t tolerance_{0};
    std::int16_t next_pos_{0};
    bool use_metric1_{true};
    bool interval_shrink_{false};
    measurement_t best_meas_{0, 0, 0};
};

#endif /* __UNIFORM_CONTRAST_PASS_H__ */
