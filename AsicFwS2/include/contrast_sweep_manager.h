/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_sweep_manager.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-15-2016
 * @brief   Implements contrast optimization full sweep of several modules in parallel
 *
 ******************************************************************************/

#ifndef __CONTRAST_SWEEP_MANAGER_H__
#define __CONTRAST_SWEEP_MANAGER_H__

#include "os.h"
#include "light_system.h"
#include "contrast_sweep.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Runs full contrast sweep on given camera modules
 * @param cam_channel_bitmask Bitmask of camera module channels on which to run full sweep
 * @return Camera module bitmask used to update command status
 */
uint32_t contrast_full_sweep(uint32_t cam_channel_bitmask);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <array>
#include <memory>

class contrast_sweep_manager {
public:
    contrast_sweep_manager();

    ~contrast_sweep_manager();

    void init_full_sweep(std::uint32_t cam_channel_bitmask);

    void init_refinement(std::uint32_t cam_channel_bitmask, std::uint32_t distance_mm);

    std::uint32_t execute_sweep();

private:
    static constexpr std::size_t MAX_NUM_CAM_CH = 6;
    static const EventBits_t CAPTURE_EVENT_CH0 = 1 << MAX_NUM_CAM_CH;
    static const EventBits_t MOVE_EVENT_CH0 = 1;

    EventBits_t process_move_events(EventBits_t events);

    struct contrast_sweep_rec {
        std::unique_ptr<contrast_sweep> cs;
        std::unique_ptr<char[]> image_buf;
        char* image_buf_raw_p;
        std::size_t image_size;
        bool final_move_started{false};
    };

    std::uint32_t cam_bitmask_{0};
    std::uint32_t cam_channel_notdone_bitmask_{0};
    std::array<contrast_sweep_rec, MAX_NUM_CAM_CH> csrec_arr_;
    std::size_t btcm_used_size_{0};
    EventGroupHandle_t event_group_;
};

#endif

#endif /* __CONTRAST_SWEEP_MANAGER_H__ */
