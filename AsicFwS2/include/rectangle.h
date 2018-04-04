/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    roi.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-8-2016
 * @brief   Region of interest rectangle
 *
 ******************************************************************************/

#ifndef __RECTANGLE_H__
#define __RECTANGLE_H__

#ifdef __cplusplus

#include <cstdint>
#include "img_sensor.h"

class rectangle {
public:
    rectangle(std::int16_t left_x, std::int16_t top_y, std::uint16_t width, std::uint16_t height);

    rectangle() :
            rectangle{0,0,0,0} {
    }

    void set(std::int16_t left_x, std::int16_t top_y, std::uint16_t width, std::uint16_t height);

    std::int16_t left_x() const {
        return left_x_;
    }

    std::int16_t top_y() const {
        return top_y_;
    }

    std::uint16_t width() const {
        return width_;
    }

    std::uint16_t height() const {
        return height_;
    }

    std::uint32_t area() const {
        return static_cast<std::uint32_t>(width_) * height_;
    }

    void translate(std::int16_t delta_x, std::int16_t delta_y);

    static std::size_t scaled_width(std::uint16_t width, std::uint16_t scale) {
        return (width + scale - 1) / scale;
    }

    static std::size_t scaled_height(std::uint16_t height, std::uint16_t scale) {
        return (height + scale - 1) / scale;
    }

    static std::size_t image_line_size(std::uint16_t width, std::uint16_t scale) {
        return (10*static_cast<std::uint32_t>(scaled_width(width, scale)) + 7) / 8;
    }

    static std::size_t image_line_size_padded(std::uint16_t width, std::uint16_t scale) {
        return ((image_line_size(width, scale) + 15) / 16) * 16;
    }

    std::size_t image_line_size(std::uint16_t scale) const {
        return image_line_size(width_, scale);
    }

    std::size_t image_line_size_padded(std::uint16_t scale) const {
        return image_line_size_padded(width_, scale);
    }

    std::size_t image_size(std::uint16_t scale) const {
        return image_line_size_padded(scale) * scaled_height(height_, scale);
    }

    rectangle minimum_capture_rectangle(std::uint16_t scale = 1) const;

private:
    std::int16_t left_x_;
    std::int16_t top_y_;
    std::uint16_t width_;
    std::uint16_t height_;
};

rectangle rectangle_flip(const rectangle& r, flip_mode_t flip_mode);

rectangle calib_rectangle_to_abs(const rectangle& r, flip_mode_t flip_mode);

rectangle abs_rectangle_to_calib(const rectangle& r, flip_mode_t flip_mode);

void debug_print_image(const void* img, const rectangle& capt_rect, const rectangle& roi_rect,
        flip_mode_t flip_mode, std::uint16_t scale = 1);

#endif
#endif   /* __RECTANGLE_H__ */
