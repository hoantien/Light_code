/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    image_pixel_iter.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-15-2016
 * @brief   Raw10 image pixel forward iterator
 *
 ******************************************************************************/

#ifndef __IMAGE_PIXEL_ITER_H__
#define __IMAGE_PIXEL_ITER_H__

#include <cstdint>

class image_pixel_iter {
public:
    explicit image_pixel_iter(const void* current_pixelp, std::uint8_t bitp = 0);

    std::int16_t next_pixel_value();

    const void* current_pixelp() const {
        return current_pixelp_;
    }

    std::uint8_t current_pixel_bit() const {
        return bitp_;
    }

private:
    const unsigned char* current_pixelp_;
    std::uint8_t bitp_{0};
};

#endif /* __IMAGE_PIXEL_ITER_H__ */
