/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    image_raw10.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-15-2016
 * @brief   Raw10 image pixel forward iterator
 *
 ******************************************************************************/

#include "rectangle.h"
#include "image_raw10.h"

image_raw10::image_raw10(const void* image_startp, std::uint16_t width) :
        image_startp_{static_cast<const unsigned char*>(image_startp)},
        width_{width} {
}

image_pixel_iter image_raw10::get_iter(std::uint16_t line_n, std::uint16_t pixel_n) {
    std::uint32_t line_size16 = rectangle::image_line_size_padded(width_, 1);
    const unsigned char* pp = image_startp_ + line_n * line_size16;
    std::uint32_t bits_to_skip = pixel_n * 10;
    pp += bits_to_skip / 8;
    std::uint8_t bits_rem = bits_to_skip & 0x7;
    return image_pixel_iter{pp, bits_rem};
}
