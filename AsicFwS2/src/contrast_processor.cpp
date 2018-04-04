/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_processor.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-14-2016
 * @brief   Contrast processing for auto focus refinement
 *
 ******************************************************************************/

#include "board_config.h"
#include "log.h"
#include "image_raw10.h"
#include "contrast_processor.h"

#define SLOGF_ID                SLOG_ID_AF

namespace {
    using roi_color_cache = contrast_processor::roi_color_cache;
    using roi_color_line = contrast_processor::roi_color_line;

    const std::int16_t data_pedestal = 42;

    std::uint32_t square_diff_th(std::int32_t x, std::int32_t y, std::int32_t factor_q16,
            std::int16_t additive_noise_var) {
        std::int32_t th = (factor_q16 * (x + y - 2*data_pedestal)) / 65536;
        th += 2 * additive_noise_var;
        std::int32_t d = x - y;
        std::int32_t s = d * d;
        if (s <= th)
            s = 0;
        else
            s -= th;
        return s;
    }

    std::uint64_t get_rb_line_metric_gradient_th_factor(const roi_color_cache& color_cache, std::uint16_t width,
            std::int32_t th_factor_q16, std::int16_t additive_noise_var) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        const roi_color_line& prev_line = color_cache.ref(1);
        for (std::size_t i = 1; i < width; ++i) {
            std::int16_t v = curr_line[i];
            ret += square_diff_th(v, curr_line[i-1], th_factor_q16, additive_noise_var);
            ret += square_diff_th(v, prev_line[i], th_factor_q16, additive_noise_var);
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_gradient_th_factor(const roi_color_cache& green_cache, std::uint16_t width,
            std::int16_t prev_line_offset, std::int32_t th_factor_q16, std::int16_t additive_noise_var) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = green_cache.ref();
        const roi_color_line& prev_line = green_cache.ref(1);
        std::size_t i_begin{0};
        std::size_t i_end{width};
        if (prev_line_offset > 0)
            i_end -= prev_line_offset;
        else
            i_begin -= prev_line_offset;
        for (std::size_t i = i_begin; i < i_end; ++i) {
            std::int16_t v = curr_line[i];
            ret += square_diff_th(v, prev_line[i + prev_line_offset], th_factor_q16, additive_noise_var);
            ret += square_diff_th(v, prev_line[i], th_factor_q16, additive_noise_var);
        }
        return ret;
    }

    std::uint64_t get_rb_line_metric_gradient(const roi_color_cache& color_cache, std::uint16_t width) {
        return get_rb_line_metric_gradient_th_factor(color_cache, width, 0, 0);
    }

    std::uint64_t get_green_line_metric_gradient(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_green_line_metric_gradient_th_factor(green_cache, width, prev_line_offset, 0, 0);
    }

    const std::int32_t gradient_threshold_factor_q16 = 4 * (static_cast<std::int32_t>(1) << 16) / 5;
    const std::int16_t additive_data_noise_var = 3 * 4;

    std::uint64_t get_rb_line_metric_gradient_th(const roi_color_cache& color_cache, std::uint16_t width) {
        return get_rb_line_metric_gradient_th_factor(color_cache, width,
                gradient_threshold_factor_q16, additive_data_noise_var);
    }

    std::uint64_t get_green_line_metric_gradient_th(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_green_line_metric_gradient_th_factor(green_cache, width, prev_line_offset,
                gradient_threshold_factor_q16, additive_data_noise_var);
    }

    std::uint64_t get_rb_line_metric_laplacian1(const roi_color_cache& color_cache, std::uint16_t width) {
        std::uint64_t ret{0};
        const roi_color_line& next_line = color_cache.ref();
        const roi_color_line& curr_line = color_cache.ref(1);
        const roi_color_line& prev_line = color_cache.ref(2);
        for (std::size_t i = 1; (i + 1) < width; ++i) {
            std::int16_t v = 4 * curr_line[i];
            std::int32_t d = v - curr_line[i-1] - curr_line[i+1] - prev_line[i] - next_line[i];
            ret += d * d;
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_laplacian1(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        std::uint64_t ret{0};
        const roi_color_line& next_line = green_cache.ref();
        const roi_color_line& curr_line = green_cache.ref(1);
        const roi_color_line& prev_line = green_cache.ref(2);
        std::size_t i_begin{0};
        std::size_t i_end{width};
        prev_line_offset = -prev_line_offset;
        if (prev_line_offset > 0)
            i_end -= prev_line_offset;
        else
            i_begin -= prev_line_offset;
        for (std::size_t i = i_begin; i < i_end; ++i) {
            std::int16_t v = 4 * curr_line[i];
            std::int32_t d = v - prev_line[i + prev_line_offset] - prev_line[i]
                               - next_line[i + prev_line_offset] - next_line[i];
            ret += d * d;
        }
        return ret;
    }

    std::uint64_t get_rb_line_metric_laplacian2(const roi_color_cache& color_cache, std::uint16_t width) {
        std::uint64_t ret{0};
        const roi_color_line& next_line2 = color_cache.ref();
        const roi_color_line& next_line1 = color_cache.ref(1);
        const roi_color_line& curr_line = color_cache.ref(2);
        const roi_color_line& prev_line1 = color_cache.ref(3);
        const roi_color_line& prev_line2 = color_cache.ref(4);
        for (std::size_t i = 2; (i + 2) < width; ++i) {
            std::int16_t v = curr_line[i];
            std::int16_t n1 = curr_line[i-1] + curr_line[i+1] + prev_line1[i] + next_line1[i];
            std::int16_t n2 = curr_line[i-2] + curr_line[i+2] + prev_line2[i] + next_line2[i]
                              + prev_line1[i-1] + prev_line1[i+1] + next_line1[i-1] + next_line1[i+1];
            std::int32_t d = 16 * v - 2 * n1 - n2;
            ret += d * d;
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_laplacian2(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        std::uint64_t ret{0};
        const roi_color_line& next_line2 = green_cache.ref();
        const roi_color_line& next_line1 = green_cache.ref(1);
        const roi_color_line& curr_line = green_cache.ref(2);
        const roi_color_line& prev_line1 = green_cache.ref(3);
        const roi_color_line& prev_line2 = green_cache.ref(4);
        for (std::size_t i = 1; (i + 1) < width; ++i) {
            std::int16_t v = curr_line[i];
            std::int32_t n1 = prev_line1[i] + next_line1[i] + prev_line1[i + prev_line_offset] + next_line1[i + prev_line_offset];
            std::int32_t n2 = curr_line[i-1] + curr_line[i+1] + prev_line2[i] + next_line2[i];
            std::int32_t d = 12 * v - 2 * n1 - n2;
            ret += d * d;
        }
        return ret;
    }

    std::uint64_t get_rb_line_metric_decimated2_gradient(const roi_color_cache& color_cache, std::uint16_t width) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        const roi_color_line& prev_line2 = color_cache.ref(2);
        for (std::size_t i = 2; i < width; ++i) {
            std::int16_t v = curr_line[i];
            std::int32_t d = v - curr_line[i-2];
            ret += d * d;
            d = v - prev_line2[i];
            ret += d * d;
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_decimated2_gradient(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_rb_line_metric_decimated2_gradient(green_cache, width);
    }

    std::uint64_t get_rb_line_metric_decimated4_gradient_th_factor(const roi_color_cache& color_cache, std::uint16_t width,
            std::int32_t th_factor_q16, std::int16_t additive_noise_var) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        const roi_color_line& prev_line4 = color_cache.ref(4);
        for (std::size_t i = 4; i < width; ++i) {
            std::int16_t v = curr_line[i];
            ret += square_diff_th(v, curr_line[i-4], th_factor_q16, additive_noise_var);
            ret += square_diff_th(v, prev_line4[i], th_factor_q16, additive_noise_var);
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_decimated4_gradient_th_factor(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset,
            std::int32_t th_factor_q16, std::int16_t additive_noise_var) {
        return get_rb_line_metric_decimated4_gradient_th_factor(green_cache, width, th_factor_q16, additive_noise_var);
    }

    std::uint64_t get_rb_line_metric_decimated4_gradient(const roi_color_cache& color_cache, std::uint16_t width) {
        return get_rb_line_metric_decimated4_gradient_th_factor(color_cache, width, 0, 0);
    }

    std::uint64_t get_green_line_metric_decimated4_gradient(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_green_line_metric_decimated4_gradient_th_factor(green_cache, width, prev_line_offset, 0, 0);
    }

    std::uint64_t get_rb_line_metric_decimated4_gradient_th(const roi_color_cache& color_cache, std::uint16_t width) {
        return get_rb_line_metric_decimated4_gradient_th_factor(color_cache, width, gradient_threshold_factor_q16, additive_data_noise_var);
    }

    std::uint64_t get_green_line_metric_decimated4_gradient_th(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_green_line_metric_decimated4_gradient_th_factor(green_cache, width, prev_line_offset, gradient_threshold_factor_q16, additive_data_noise_var);
    }

    std::uint64_t get_rb_line_metric_intensity(const roi_color_cache& color_cache, std::uint16_t width) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        for (std::size_t i = 0; i < width; ++i) {
            ret += curr_line[i];
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_intensity(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_rb_line_metric_intensity(green_cache, width);
    }

    std::uint64_t get_rb_line_metric_energy(const roi_color_cache& color_cache, std::uint16_t width) {
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        for (std::size_t i = 0; i < width; ++i) {
            std::int16_t v = curr_line[i];
            ret += v * v;
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_energy(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        return get_rb_line_metric_energy(green_cache, width);
    }

    std::uint64_t get_rb_line_metric_filtered_gradient_th(const roi_color_cache& color_cache, std::uint16_t width) {
        const std::int32_t threshold_factor_q16 = 3 * 2 * (static_cast<std::int32_t>(1) << 16) / 5;
        const std::int16_t additive_noise_var = 3 * 14;
        std::uint64_t ret{0};
        const roi_color_line& curr_line = color_cache.ref();
        for (std::size_t i = 5; i < width; ++i) {
            std::int16_t v1 = 3 * curr_line[i] + 2 * curr_line[i-1] + curr_line[i-2];
            std::int16_t v2 = curr_line[i-3] + 2 * curr_line[i-4] + 3 * curr_line[i-5];
            ret += square_diff_th(v1, v2, threshold_factor_q16, additive_noise_var);
            v1 = 3 * curr_line[i] + 2 * color_cache.ref(1)[i] + color_cache.ref(2)[i];
            v2 = color_cache.ref(3)[i] + 2 * color_cache.ref(4)[i] + 3 * color_cache.ref(5)[i];
            ret += square_diff_th(v1, v2, threshold_factor_q16, additive_noise_var);
        }
        return ret;
    }

    std::uint64_t get_green_line_metric_filtered_gradient_th(const roi_color_cache& green_cache, std::uint16_t width, std::int16_t prev_line_offset) {
        const std::int32_t threshold_factor_q16 = 3 * 2 * (static_cast<std::int32_t>(1) << 16) / 5;
        const std::int16_t additive_noise_var = 3 * 14;
        std::uint64_t ret{0};
        const roi_color_line& curr_line = green_cache.ref();
        const roi_color_line& prev_line1 = green_cache.ref(1);
        const roi_color_line& prev_line2 = green_cache.ref(2);
        const roi_color_line& prev_line3 = green_cache.ref(3);
        const roi_color_line& prev_line4 = green_cache.ref(4);
        const roi_color_line& prev_line5 = green_cache.ref(5);
        std::size_t i_begin;
        std::size_t i_end;
        if (prev_line_offset > 0) {
            i_begin = 2 * prev_line_offset;
            i_end = width - 3 * prev_line_offset;
        } else {
            i_begin = -3 * prev_line_offset;
            i_end = width + 2 * prev_line_offset;
        }
        for (std::size_t i = i_begin; i < i_end; ++i) {
            std::int16_t v1 = 3 * curr_line[i] + 2 * prev_line1[i] + prev_line2[i-prev_line_offset];
            std::int16_t v2 = prev_line3[i-prev_line_offset] + 2 * prev_line4[i-2*prev_line_offset] + 3 * prev_line5[i-2*prev_line_offset];
            ret += square_diff_th(v1, v2, threshold_factor_q16, additive_noise_var);
            v1 = 3 * curr_line[i] + 2 * prev_line1[i+prev_line_offset] + prev_line2[i+prev_line_offset];
            v2 = prev_line3[i+2*prev_line_offset] + 2 * prev_line4[i+2*prev_line_offset] + 3 * prev_line5[i+3*prev_line_offset];
            ret += square_diff_th(v1, v2, threshold_factor_q16, additive_noise_var);
        }
        return ret;
    }
}

std::array<contrast_processor::metric_algo_desc, contrast_processor::NUM_METRICS> contrast_processor::metric_algo_arr_{
    metric_algo_desc{ 2, &get_green_line_metric_gradient, &get_rb_line_metric_gradient },
    metric_algo_desc{ 3, &get_green_line_metric_laplacian1, &get_rb_line_metric_laplacian1 },
    metric_algo_desc{ 5, &get_green_line_metric_laplacian2, &get_rb_line_metric_laplacian2 },
    metric_algo_desc{ 3, &get_green_line_metric_decimated2_gradient, &get_rb_line_metric_decimated2_gradient },
    metric_algo_desc{ 5, &get_green_line_metric_decimated4_gradient, &get_rb_line_metric_decimated4_gradient },
    metric_algo_desc{ 1, &get_green_line_metric_intensity, &get_rb_line_metric_intensity },
    metric_algo_desc{ 1, &get_green_line_metric_energy, &get_rb_line_metric_energy },
    metric_algo_desc{ 2, &get_green_line_metric_gradient_th, &get_rb_line_metric_gradient_th },
    metric_algo_desc{ 5, &get_green_line_metric_decimated4_gradient_th, &get_rb_line_metric_decimated4_gradient_th },
    metric_algo_desc{ 6, &get_green_line_metric_filtered_gradient_th, &get_rb_line_metric_filtered_gradient_th },
};

contrast_processor::contrast_processor(std::uint16_t first_roi_pixel, std::uint16_t roi_width) :
        first_roi_pixel_{first_roi_pixel},
        roi_width_{roi_width} {
}

void contrast_processor::enable_metrics(std::uint32_t metrics_mask) {
    std::size_t i = 0;
    while (metrics_mask != 0) {
        if ((metrics_mask & 1) != 0)
            metrics_enable_[i] = true;
        metrics_mask >>= 1;
        ++i;
    }
}

void contrast_processor::update_red_metric(metric_type metric) {
    metrics_arr_[metric].red += get_rb_line_metric(metric, red_roi_cache_);
}

void contrast_processor::update_blue_metric(metric_type metric) {
    metrics_arr_[metric].blue += get_rb_line_metric(metric, blue_roi_cache_);
}

void contrast_processor::update_green_metric_even(metric_type metric) {
    metrics_arr_[metric].green += get_green_line_metric(metric, -1);
}

void contrast_processor::update_green_metric_odd(metric_type metric) {
    metrics_arr_[metric].green += get_green_line_metric(metric, 1);
}

std::uint64_t contrast_processor::get_rb_line_metric(metric_type metric, const roi_color_cache& color_cache) {
    return (*metric_algo_arr_[metric].get_rb_line_metric_fun)(color_cache, roi_width_);
}

std::uint64_t contrast_processor::get_green_line_metric(metric_type metric, std::int16_t prev_line_offset) {
    return (*metric_algo_arr_[metric].get_green_line_metric_fun)(green_roi_cache_, roi_width_, prev_line_offset);
}

void contrast_processor::process_line(image_pixel_iter line_start) {
    roi_color_cache* even_cache;
    roi_color_cache* odd_cache;
    if ((line_cnt_ & 1) == 0) {
        even_cache = &green_roi_cache_;
        odd_cache = &red_roi_cache_;
    } else {
        even_cache = &blue_roi_cache_;
        odd_cache = &green_roi_cache_;
    }
    get_color_roi(line_start, even_cache, odd_cache);
    std::uint16_t num_lines_green = line_cnt_ + 1;
    if ((line_cnt_ & 1) == 0) {
        update_metrics_even(num_lines_green);
    } else {
        update_metrics_odd(num_lines_green);
    }
    ++line_cnt_;
}

void contrast_processor::update_metrics_even(std::uint16_t num_lines) {
    for (unsigned int i = 0; i < NUM_METRICS; ++i) {
        if (metrics_enable_[i]) {
            metric_type mt = static_cast<metric_type>(i);
            metric_algo_desc& ma = metric_algo_arr_[i];
            if (num_lines >= ma.min_lines)
                update_green_metric_even(mt);
            std::uint16_t num_lines_red = (num_lines + 1) / 2;
            if (num_lines_red >= ma.min_lines)
                update_red_metric(mt);
        }
    }
}

void contrast_processor::update_metrics_odd(std::uint16_t num_lines) {
    for (unsigned int i = 0; i < NUM_METRICS; ++i) {
        if (metrics_enable_[i]) {
            metric_type mt = static_cast<metric_type>(i);
            metric_algo_desc& ma = metric_algo_arr_[i];
            if (num_lines >= ma.min_lines)
                update_green_metric_odd(mt);
            std::uint16_t num_lines_blue = num_lines / 2;
            if (num_lines_blue >= ma.min_lines)
                update_blue_metric(mt);
        }
    }
}

void contrast_processor::get_color_roi(image_pixel_iter line,
        roi_color_cache* color_cache_even, roi_color_cache* color_cache_odd) {
    color_cache_even->advance();
    color_cache_odd->advance();
    roi_color_line& even_line = color_cache_even->ref();
    roi_color_line& odd_line = color_cache_odd->ref();
    for (std::size_t i = 0; i < roi_width_; ++i) {
        even_line[i] = line.next_pixel_value();
        odd_line[i] = line.next_pixel_value();
//        if (i == 0)
//            SLOGF(SLOG_DEBUG, "New line");
//        SLOGF(SLOG_DEBUG, "Even/odd line %d values: %d / %d", (int)i, (int)(even_line[i]), (int)(odd_line[i]));
    }
}
