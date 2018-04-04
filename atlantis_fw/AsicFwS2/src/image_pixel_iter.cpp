/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    image_pixel_iter.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-15-2016
 * @brief   Raw10 image pixel forward iterator
 *
 ******************************************************************************/

#include "image_pixel_iter.h"

image_pixel_iter::image_pixel_iter(const void* current_pixelp, std::uint8_t bitp) :
        current_pixelp_{static_cast<const unsigned char*>(current_pixelp)},
        bitp_{bitp} {
}

std::int16_t image_pixel_iter::next_pixel_value() {
    std::int16_t value = *current_pixelp_ >> bitp_;
    ++current_pixelp_;
    value |= (static_cast<std::uint16_t>(*current_pixelp_) << (8 - bitp_)) & 0x3ff;
    bitp_ += 2;
    if (bitp_ == 8) {
        ++current_pixelp_;
        bitp_ = 0;
    }
    return value;
}
