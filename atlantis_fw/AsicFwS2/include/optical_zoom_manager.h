/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    optical_zoom_manager.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-13-2016
 * @brief   Optical zoom manager singleton
 *
 ******************************************************************************/

#ifndef __OPTICAL_ZOOM_MANAGER_H__
#define __OPTICAL_ZOOM_MANAGER_H__

#include "os.h"

#ifdef __cplusplus
extern "C" {
#endif

void initialize_optical_zoom_manager(void);

#ifdef __cplusplus
}

#include <cstdint>

class optical_zoom_manager {
public:
    static optical_zoom_manager& instance() {
        return instance_;
    }

    optical_zoom_manager();

    ~optical_zoom_manager();

    void start_mirrors_move();

    void wait_for_mirrors_move_done();

    void initialize();

private:

    bool check_start_mirror_move(std::uint8_t module, std::int16_t hc);

    static constexpr std::size_t MAX_NUM_MIRROR = 6;

    static optical_zoom_manager instance_;

    SemaphoreHandle_t mirror_move_done_sem_;
    std::uint8_t num_mirrors_moving_{0};
};

#endif
#endif /* __OPTICAL_ZOOM_MANAGER_H__ */
