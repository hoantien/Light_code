/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_pass.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sept-14-2016
 * @brief   Interface for one pass of contrast computation
 *
 ******************************************************************************/

#ifndef __CONTRAST_PASS_H__
#define __CONTRAST_PASS_H__

#ifdef __cplusplus

#include <cstdint>
#include <limits>


class contrast_pass {
public:
    struct measurement_t {
        std::int16_t pos;
        std::int64_t metric1;
        std::int64_t metric2;

        static bool is_better(bool use_metric1, const measurement_t& meas1, const measurement_t& meas2) {
            return use_metric1 ? (meas1.metric1 > meas2.metric1) : (meas1.metric2 > meas2.metric2);
        }
    };

    static const std::int64_t METRIC_MIN{std::numeric_limits<std::int64_t>::min()};

    virtual ~contrast_pass() = default;

    virtual void initialize(const contrast_pass& prev_pass) = 0;

    virtual void initialize() = 0;

    virtual bool is_done() const = 0;

    virtual std::int16_t next_pos() = 0;

    virtual std::uint16_t tolerance() const = 0;

    virtual void set_measurement(const measurement_t& meas) = 0;

    virtual std::int16_t out_min_pos() const = 0;

    virtual std::int16_t out_max_pos() const = 0;

    virtual std::int16_t out_best_pos() const = 0;

    virtual std::int64_t out_best_metric() const = 0;

    virtual std::size_t num_measurements() const = 0;

    virtual const measurement_t* measurements() const = 0;
};

#endif

#endif /* __CONTRAST_PASS_H__ */
