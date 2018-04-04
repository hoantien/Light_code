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

#include "os.h"             // for debug_print_image()
#include "image_raw10.h"    // for debug_print_image()
#include "rectangle.h"

void debug_print_image(const void* img, const rectangle& capt_rect, const rectangle& roi_rect,
        flip_mode_t flip_mode, std::uint16_t scale) {
    vTaskDelay(50);
    std::uint16_t num_bytes_per_line = roi_rect.image_line_size_padded(scale);
    std::uint16_t scaled_w = rectangle::scaled_width(roi_rect.width(), scale);
    std::uint16_t scaled_h = rectangle::scaled_height(roi_rect.height(), scale);
    std::uint16_t x_off = (roi_rect.left_x() - capt_rect.left_x()) / scale;
    std::uint16_t y_off = (roi_rect.top_y() - capt_rect.top_y()) / scale;
    bool flip_horizontal = ((flip_mode & FLIP_HORIZONTAL) != 0);
    flip_horizontal = (flip_horizontal != ((x_off & 1) != 0));
    bool flip_vertical = ((flip_mode & FLIP_VERTICAL) != 0);
    flip_vertical = (flip_vertical != ((y_off & 1) != 0));
    printf("\r\n\r\n##Image## %u %u %u %u",
            (unsigned int)(scaled_w), (unsigned int)(scaled_h),
            (unsigned int)flip_horizontal, (unsigned int)flip_vertical);
    image_raw10 img_r10(img, capt_rect.width()/scale);
    for (std::uint16_t y = 0; y < scaled_h; ++y) {
        image_pixel_iter p_it = img_r10.get_iter(y_off + y, x_off);
        const std::uint8_t* p = static_cast<const std::uint8_t*>(p_it.current_pixelp());
        std::uint8_t bit_off = p_it.current_pixel_bit();
        std::uint16_t value_carry = *p >> bit_off;
        ++p;
        for (std::uint16_t x = 0; x < num_bytes_per_line; ++x) {
            if ((x & 31) == 0) {
                printf("\r\n");
            }
            std::uint16_t v = *p;
            std::uint16_t v_aligned = ((v << (8 - bit_off)) & 0xff) | value_carry;
            printf("%02x ", (unsigned int)v_aligned);
            value_carry = v >> bit_off;
            ++p;
        }
        printf("\r\n");
    }
}

rectangle::rectangle(std::int16_t left_x, std::int16_t top_y, std::uint16_t width, std::uint16_t height) {
    set(left_x, top_y, width, height);
}

void rectangle::set(std::int16_t left_x, std::int16_t top_y, std::uint16_t width, std::uint16_t height) {
    std::uint16_t even_mask = 1;
    even_mask = ~even_mask;
    left_x_ = left_x & even_mask;
    top_y_ = top_y & even_mask;
    width_ = (width + 1) & even_mask;
    height_ = (height + 1) & even_mask;
}

void rectangle::translate(std::int16_t delta_x, std::int16_t delta_y) {
    left_x_ += delta_x;
    top_y_ += delta_y;
}

rectangle rectangle::minimum_capture_rectangle(std::uint16_t scale) const {
    std::int16_t c_left_x{left_x_};
    std::uint16_t c_width{width_};
    if (c_left_x < 8) {
        c_width -= (8 - c_left_x);
        c_left_x = 8;
    }
    if ((c_left_x & 1) != 0) {
        --c_left_x;
        ++c_width;
    }
    std::int16_t c_top_y{top_y_};
    std::uint16_t c_height{height_};
    if (c_top_y < 8) {
        c_height -= (8 - c_top_y);
        c_top_y = 8;
    }
    if ((c_top_y & 1) != 0) {
        --c_top_y;
        ++c_height;
    }
    if ((c_height & 1) != 0)
        ++c_height;

    // X output size must be multiple of 16 pixels for raw10 output, so we enforce here a scaled capture width multiple of 16
    std::uint16_t c_scaled_width = scaled_width(c_width, scale);
    c_scaled_width = (c_scaled_width + 15) / 16 * 16;
    c_width = scale * c_scaled_width;
    const std::int16_t right_x_max{4208+16+8};
    const std::uint16_t scaled_width_min = 320; // minimum capture width 320 per sensor spec
    if (c_scaled_width < scaled_width_min)
        c_width = scale * scaled_width_min;
    if ((c_left_x + c_width) > right_x_max)
        c_left_x = right_x_max - c_width;

    // Y output size must be even
    std::uint16_t c_scaled_height = scaled_height(c_height, scale);
    if ((c_scaled_height & 1) != 0)
        ++c_scaled_height;
    c_height = scale * c_scaled_height;
    const std::int16_t bottom_y_max{3120+16+8};
    if ((c_top_y + c_height) > bottom_y_max)
        c_top_y = bottom_y_max - c_height;
    return rectangle{c_left_x, c_top_y, c_width, c_height};
}

const std::int16_t abs_pixel_offset_x{40};
const std::int16_t abs_pixel_offset_y{16};
const std::uint16_t full_res_width{4160};
const std::uint16_t full_res_height{3120};

rectangle rectangle_flip(const rectangle& r, flip_mode_t flip_mode) {
    std::int16_t left_x{r.left_x()};
    std::uint16_t width{r.width()};
    if ((flip_mode & FLIP_HORIZONTAL) != 0)
        left_x = full_res_width - (left_x + width);
    std::int16_t top_y{r.top_y()};
    std::uint16_t height{r.height()};
    if ((flip_mode & FLIP_VERTICAL) != 0)
        top_y = full_res_height - (top_y + height);
    return rectangle{left_x, top_y, width, height};
}

rectangle calib_rectangle_to_abs(const rectangle& r, flip_mode_t flip_mode) {
    rectangle r_abs{rectangle_flip(r, flip_mode)};
    r_abs.translate(abs_pixel_offset_x,abs_pixel_offset_y);
    return r_abs;
}

rectangle abs_rectangle_to_calib(const rectangle& r, flip_mode_t flip_mode) {
    rectangle r_tmp{r};
    r_tmp.translate(-abs_pixel_offset_x,-abs_pixel_offset_y);
    return rectangle_flip(r_tmp, flip_mode);
}
