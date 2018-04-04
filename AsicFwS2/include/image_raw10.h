/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    image_raw10.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-15-2016
 * @brief   Raw10 image pixel forward iterator
 *
 ******************************************************************************/

#ifndef __IMAGE_RAW10_H__
#define __IMAGE_RAW10_H__

#include <cstdint>

#include "image_pixel_iter.h"

/// Represents RAW10 image with Light's format
class image_raw10 {
public:
    explicit image_raw10(const void* image_startp, std::uint16_t width = 0);

    /// @return iterator
    image_pixel_iter get_iter(std::uint16_t line_n, std::uint16_t pixel_n);

private:
    const unsigned char* image_startp_; ///< pointer to first byte of image
    std::uint16_t width_;               ///< width in pixels
};

#endif /* __IMAGE_RAW10_H__ */
