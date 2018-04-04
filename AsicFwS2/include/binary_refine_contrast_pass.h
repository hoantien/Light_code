/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    binary_refine_contrast_pass.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass which aggressively refines search interval
 *
 ******************************************************************************/

#ifndef __BINARY_REFINE_CONTRAST_PASS_H__
#define __BINARY_REFINE_CONTRAST_PASS_H__

#include <array>
#include "contrast_pass.h"

class binary_refine_contrast_pass: public contrast_pass {
public:

    binary_refine_contrast_pass(std::uint16_t tolerance, bool use_metric1);

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
    std::uint16_t tolerance_{0};
    bool use_metric1_{true};
    std::int16_t delta_{1};
    std::int16_t next_pos_{0};
    std::array<measurement_t, 2> meas_arr_;
    bool best_meas_idx_{0};
    bool second_meas_ongoing_{false};
    bool done_{false};
};

#endif /* __BINARY_REFINE_CONTRAST_PASS_H__ */
