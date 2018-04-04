/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_processor.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-14-2016
 * @brief   Contrast processing for auto focus refinement
 *
 ******************************************************************************/

#ifndef __CONTRAST_PROCESSOR_H__
#define __CONTRAST_PROCESSOR_H__

#ifdef __cplusplus
extern "C" {
#endif
void process_contrast_test(unsigned int img_addr);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstdint>
#include <array>
#include <bitset>
#include "image_pixel_iter.h"

template<typename T, std::size_t S>
class circular_cache_buffer {
public:
    void advance() {
        if (++index_ == S)
            index_ = 0;
    }

    T& ref(std::uint16_t delta = 0) {
        return array_[circ_index(delta)];
    }

    const T& ref(std::uint16_t delta = 0) const {
        return array_[circ_index(delta)];
    }

private:
    std::size_t circ_index(std::size_t delta) const {
        std::size_t ci;
        delta = delta % S;
        if (delta <= index_)
            ci = index_ - delta;
        else
            ci = S - delta + index_;
        return ci;
    }

    std::size_t index_{0};
    std::array<T, S> array_;
};

class contrast_processor {
public:

    enum metric_type {
        GRADIENT,
        LAPLACIAN1,
        LAPLACIAN2,
        DECIMATED2_GRADIENT,
        DECIMATED4_GRADIENT,
        INTENSITY,
        ENERGY,
        GRADIENT_TH,
        DECIMATED4_GRADIENT_TH,
        FILTERED_GRADIENT_TH,
        NUM_METRICS
    };

	contrast_processor(std::uint16_t first_roi_pixel, std::uint16_t roi_width);

	void process_line(image_pixel_iter line_start);

	std::uint64_t get_metric_green(metric_type metric) const {
		return metrics_arr_[metric].green;
	}

	std::uint64_t get_metric_red(metric_type metric) const {
        return metrics_arr_[metric].red;
	}

	std::uint64_t get_metric_blue(metric_type metric) const {
        return metrics_arr_[metric].blue;
	}

	std::uint64_t get_metric(metric_type metric) const {
		return metrics_arr_[metric].total_metric();
	}

    static const std::size_t MAX_ROI_WIDTH = 180;
    static const std::size_t NUM_LINES_CACHE = 5;
    using roi_color_line = std::array<std::int16_t, MAX_ROI_WIDTH>;
    using roi_color_cache = circular_cache_buffer<roi_color_line, NUM_LINES_CACHE>;

    struct metric_algo_desc {
        std::uint16_t min_lines;
        std::uint64_t (*get_green_line_metric_fun)(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset);
        std::uint64_t (*get_rb_line_metric_fun)(const roi_color_cache& color_cache, std::uint16_t width);
    };

    void enable_metric(metric_type mt) {
        metrics_enable_[mt] = true;
    }

    void disable_metric(metric_type mt) {
        metrics_enable_[mt] = false;
    }

    void enable_all_metrics() {
        metrics_enable_.set();
    }

    void disable_all_metrics() {
        metrics_enable_.reset();
    }

    void enable_metrics(std::uint32_t metrics_mask);

    struct metrics_per_color {
        std::uint64_t green;
        std::uint64_t red;
        std::uint64_t blue;

        metrics_per_color() :
                green{0}, red{0}, blue{0} {
        }

        std::uint64_t total_metric() const {
            return green + red + blue;
        }
    };

    using metrics_array = std::array<metrics_per_color, NUM_METRICS>;

    const metrics_array& get_metrics_array() const {
        return metrics_arr_;
    }

private:
    void get_color_roi(image_pixel_iter line,
            roi_color_cache* color_cache_even, roi_color_cache* color_cache_odd);

    std::uint64_t get_rb_line_metric(metric_type metric, const roi_color_cache& color_cache);

    std::uint64_t get_green_line_metric(metric_type metric, std::int16_t prev_line_offset);

    void update_red_metric(metric_type metric);

    void update_blue_metric(metric_type metric);

    void update_green_metric_even(metric_type metric);

    void update_green_metric_odd(metric_type metric);

    void update_metrics_even(std::uint16_t num_lines);

    void update_metrics_odd(std::uint16_t num_lines);

    std::uint16_t first_roi_pixel_;
	std::uint16_t roi_width_;
	std::uint16_t line_cnt_{0};

	metrics_array metrics_arr_;
	std::bitset<NUM_METRICS> metrics_enable_;
    roi_color_cache green_roi_cache_;
    roi_color_cache red_roi_cache_;
    roi_color_cache blue_roi_cache_;

    static std::array<metric_algo_desc, NUM_METRICS> metric_algo_arr_;
};

#endif
#endif /* __CONTRAST_PROCESSOR_H__ */
