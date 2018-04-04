/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    optical_zoom_manager.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-13-2016
 * @brief   Optical zoom manager singleton
 *
 ******************************************************************************/

#include "light_system.h"
#include "log.h"
#include "calib_data_manager.h"
#include "optical_zoom_manager.h"

#define SLOGF_ID                SLOG_ID_AF


extern "C" void initialize_optical_zoom_manager() {
    optical_zoom_manager::instance().initialize();
}

optical_zoom_manager optical_zoom_manager::instance_{};

optical_zoom_manager::optical_zoom_manager() :
        mirror_move_done_sem_{nullptr} {
}

optical_zoom_manager::~optical_zoom_manager() {
    if (mirror_move_done_sem_ != nullptr)
        vSemaphoreDelete(mirror_move_done_sem_);
}

void optical_zoom_manager::initialize() {
    mirror_move_done_sem_ = xSemaphoreCreateCounting(MAX_NUM_MIRROR , 0);
}

namespace {
    void move_mirror_callback(void* user_data) {
        SemaphoreHandle_t sem = *static_cast<SemaphoreHandle_t*>(user_data);
        xSemaphoreGive(sem);
    }
}

bool optical_zoom_manager::check_start_mirror_move(std::uint8_t module, std::int16_t hc) {
    if (hc < 0) {
        SLOGF(SLOG_ERROR, "Will not move mirror %02X as optical zoom computation failed", module);
        return false;
    }
    std::uint32_t mf = light_system->m_filter;
    std::uint8_t idx = module_to_idx(module);
    if (((mf >> idx) & 1) != 0) {
        cam_typedef_t* pcam = idx_to_object(idx);
        SLOGF(SLOG_DEBUG, "Requesting to move mirror %02X to %d", pcam->info.module, hc);
        cam_move_mirr_to_position(pcam, hc, &move_mirror_callback, &mirror_move_done_sem_, 0);
        ++num_mirrors_moving_;
        return true;
    }
    return false;
}

void optical_zoom_manager::start_mirrors_move() {
    // Get mirror hall codes from zoom factor
    const ltaf::OpticalZoomASIC& oz = calib_data_manager::instance().optical_zoom();
    float focal_length = light_system->settings->zoom_factor * 28.0f;
    std::int16_t mirror_hc[ltaf::kNumMirrorModules];
    oz.getHallCodesByFocalLength(focal_length, mirror_hc);
    // Start moving mirrors
    // Always move Bs
    check_start_mirror_move(0xB1, mirror_hc[0]);
    check_start_mirror_move(0xB2, mirror_hc[1]);
    check_start_mirror_move(0xB3, mirror_hc[2]);
    check_start_mirror_move(0xB5, mirror_hc[3]);
    if (focal_length >= 70.0f) {
        // BC mode, also move Cs
        check_start_mirror_move(0xC1, mirror_hc[4]);
        check_start_mirror_move(0xC2, mirror_hc[5]);
        check_start_mirror_move(0xC3, mirror_hc[6]);
        check_start_mirror_move(0xC4, mirror_hc[7]);
    }
}

void optical_zoom_manager::wait_for_mirrors_move_done() {
    for (std::uint8_t i = 0; i < num_mirrors_moving_; ++i) {
        xSemaphoreTake(mirror_move_done_sem_, portMAX_DELAY);
    }
    num_mirrors_moving_ = 0;
}

