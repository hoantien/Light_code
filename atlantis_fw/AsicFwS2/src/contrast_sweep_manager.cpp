/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_sweep_manager.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-15-2016
 * @brief   Implements contrast optimization full sweep of several modules in parallel
 *
 ******************************************************************************/

#include <memory>
#include "operator_new.h"
#include "log.h"
#include "cortex_r4.h"
#include "uniform_contrast_pass.h"
#include "max_search_contrast_pass.h"
#include "binary_refine_contrast_pass.h"
#include "hal_cache.h"
#include "contrast_sweep_manager.h"


#define SLOGF_ID                SLOG_ID_AF

extern "C" uint32_t contrast_full_sweep(uint32_t cam_channel_bitmask) {
    contrast_sweep_manager csm;
    csm.init_full_sweep(cam_channel_bitmask);
    return csm.execute_sweep();
}

contrast_sweep_manager::contrast_sweep_manager() :
        event_group_(xEventGroupCreate()) {
}

contrast_sweep_manager::~contrast_sweep_manager() {
    vEventGroupDelete(event_group_);
}

EventBits_t contrast_sweep_manager::process_move_events(EventBits_t events) {
    for (std::size_t i = 0; i < MAX_NUM_CAM_CH; ++i) {
        EventBits_t move_event = MOVE_EVENT_CH0 << i;
        if ((events & move_event) != 0) {
            events &= ~move_event;
            contrast_sweep_rec& csr = csrec_arr_[i];
            if (!csr.final_move_started)
                csr.cs->start_capture_frame();
            else
                cam_channel_notdone_bitmask_ &= ~(std::uint32_t(1) << i); // done for this lens
        }
    }
    return events;
}

namespace {

    const std::uint16_t CONTRAST_REFINE_SCALE{2};

}

void contrast_sweep_manager::init_full_sweep(std::uint32_t cam_channel_bitmask) {
    cam_bitmask_ = 0;
    cam_channel_notdone_bitmask_ = cam_channel_bitmask;
    btcm_used_size_ = 0;
    std::uint16_t scale{CONTRAST_REFINE_SCALE};
    for (std::size_t i = 0; i < MAX_NUM_CAM_CH; ++i) {
        if ((cam_channel_bitmask & (1 << i)) != 0) {
            // Channel i full sweep
            SLOGF(SLOG_INFO, "Starting full sweep of cam ch %d", (int)i);
            cam_typedef_t *pcam = chan_to_object(i+1);
            cam_bitmask_ |= object_to_cam_bitmask(pcam);

            std::uint16_t left_x = pcam->cam_common_data.roi_rectangle.left_x;
            std::uint16_t top_y = pcam->cam_common_data.roi_rectangle.top_y;
            std::uint16_t width = pcam->cam_common_data.roi_rectangle.width;
            std::uint16_t height = pcam->cam_common_data.roi_rectangle.height;
            SLOGF(SLOG_DEBUG, "ROI: (%d, %d) - w: (%d, %d)", (int)left_x, (int)top_y, (int)width, (int)height);
            rectangle roi(left_x, top_y, width, height);
            rectangle capture_rect{roi.minimum_capture_rectangle(scale)};
            std::size_t img_size = capture_rect.image_size(scale);
            csrec_arr_[i].image_size = img_size;
            std::size_t btcm_free_size = BTCM_SIZE - btcm_used_size_;
            if (img_size <= btcm_free_size) {
                csrec_arr_[i].image_buf_raw_p = reinterpret_cast<char*>(BTCM_BASE + btcm_used_size_);
                btcm_used_size_ += img_size;
            } else {
                csrec_arr_[i].image_buf_raw_p = new(Heap::DDR) char[img_size];
                csrec_arr_[i].image_buf.reset(csrec_arr_[i].image_buf_raw_p);
            }
            EventBits_t move_event = MOVE_EVENT_CH0 << i;
            EventBits_t capture_event = CAPTURE_EVENT_CH0 << i;
            csrec_arr_[i].cs.reset(new contrast_sweep(*pcam, scale, capture_rect, csrec_arr_[i].image_buf_raw_p,
                    event_group_, move_event, capture_event));
            contrast_sweep& cs = *csrec_arr_[i].cs.get();
            lens_position_control lpc(*pcam);
            std::int16_t hc_min{0};
            std::int16_t hc_max{0};
            lpc.get_hard_stop_hall_codes(&hc_min, &hc_max);
            SLOGF(SLOG_DEBUG, "Hard stops: %d, %d", (int)hc_min, (int)hc_max);
            contrast_processor::metric_type metric_coarse{contrast_processor::FILTERED_GRADIENT_TH};
            contrast_processor::metric_type metric_fine{contrast_processor::GRADIENT_TH};
            if (lpc.is_vcm()) {
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 4096, 1, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 2048, 1, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 1024, 1, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 512, 1, false, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 256, 1, false, true}));
#if 0
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 128, 1, false, true}));
#endif
            } else {
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 128, 8, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 64, 4, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 32, 4, true, false}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 16, 2, false, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 8, 2, false, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 4, 2, false, true}));
#if 0
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new uniform_contrast_pass{hc_min, hc_max, 2, 1, false, true}));
#endif
            }
            cs.initialize_sweep(roi);
            cs.start_lens_move();
        }
    }
}

void contrast_sweep_manager::init_refinement(std::uint32_t cam_channel_bitmask, std::uint32_t distance_mm) {
    cam_bitmask_ = 0;
    cam_channel_notdone_bitmask_ = cam_channel_bitmask;
    btcm_used_size_ = 0;
    std::uint16_t scale{CONTRAST_REFINE_SCALE};
    for (std::size_t i = 0; i < MAX_NUM_CAM_CH; ++i) {
        if ((cam_channel_bitmask & (1 << i)) != 0) {
            // Channel i refinement
            SLOGF(SLOG_INFO, "Starting refinement of cam ch %d", (int)i);
            cam_typedef_t *pcam = chan_to_object(i+1);
            cam_bitmask_ |= object_to_cam_bitmask(pcam);

            std::uint16_t left_x = pcam->cam_common_data.roi_rectangle.left_x;
            std::uint16_t top_y = pcam->cam_common_data.roi_rectangle.top_y;
            std::uint16_t width = pcam->cam_common_data.roi_rectangle.width;
            std::uint16_t height = pcam->cam_common_data.roi_rectangle.height;
            SLOGF(SLOG_DEBUG, "ROI: (%d, %d) - w: (%d, %d)", (int)left_x, (int)top_y, (int)width, (int)height);
            rectangle roi(left_x, top_y, width, height);
            rectangle capture_rect{roi.minimum_capture_rectangle(scale)};
            std::size_t img_size = capture_rect.image_size(scale);
            csrec_arr_[i].image_size = img_size;
            std::size_t btcm_free_size = BTCM_SIZE - btcm_used_size_;
            if (img_size <= btcm_free_size) {
                csrec_arr_[i].image_buf_raw_p = reinterpret_cast<char*>(BTCM_BASE + btcm_used_size_);
                csrec_arr_[i].image_buf.reset();
                btcm_used_size_ += img_size;
            } else {
                csrec_arr_[i].image_buf_raw_p = new(Heap::DDR) char[img_size];
                csrec_arr_[i].image_buf.reset(csrec_arr_[i].image_buf_raw_p);
            }
            EventBits_t move_event = MOVE_EVENT_CH0 << i;
            EventBits_t capture_event = CAPTURE_EVENT_CH0 << i;
            csrec_arr_[i].cs.reset(new contrast_sweep(*pcam, scale, capture_rect, csrec_arr_[i].image_buf_raw_p,
                    event_group_, move_event, capture_event));
            contrast_sweep& cs = *csrec_arr_[i].cs.get();
            lens_position_control lpc(*pcam);
            std::int16_t hc_disparity = lpc.get_hall_code(distance_mm, 300); // FIXME module temperature
            SLOGF(SLOG_DEBUG, "Hall code estimation for %umm distance: %d", (unsigned int)distance_mm, hc_disparity);
            contrast_processor::metric_type metric_coarse{contrast_processor::FILTERED_GRADIENT_TH};
            contrast_processor::metric_type metric_fine{contrast_processor::GRADIENT_TH};
            if (lpc.is_vcm()) {
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new max_search_contrast_pass{hc_disparity, 2048, 1, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new binary_refine_contrast_pass{1, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new binary_refine_contrast_pass{1, false}));
            } else {
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new max_search_contrast_pass{hc_disparity, 64, 2, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new binary_refine_contrast_pass{1, true}));
                cs.add_iter_param(metric_coarse, metric_fine,
                        std::unique_ptr<contrast_pass>(new binary_refine_contrast_pass{1, false}));
            }
            cs.initialize_sweep(roi);
            cs.start_lens_move();
        }
    }
}

std::uint32_t contrast_sweep_manager::execute_sweep() {
    EventBits_t events = 0;
    EventBits_t all_events = (1 << 2*MAX_NUM_CAM_CH) - 1;;
    do {
        TickType_t wait_delay = (events == 0) ? portMAX_DELAY : 0;
        events |= xEventGroupWaitBits(event_group_, all_events, pdTRUE, pdFALSE, wait_delay);
        events = process_move_events(events);
        for (std::size_t i = 0; i < MAX_NUM_CAM_CH; ++i) {
            EventBits_t capture_event = CAPTURE_EVENT_CH0 << i;
            if (events & capture_event) {
                events &= ~capture_event;
                contrast_sweep_rec& csr = csrec_arr_[i];
                // Flush cache if image is in DDR
                if (csr.image_buf) {
                    invalidate_cache(reinterpret_cast<std::uint32_t>(csr.image_buf.get()), csr.image_size);
                }
                bool done = csr.cs->process_captured_frame();
                if (done)
                    csr.final_move_started = true;
                csr.cs->start_lens_move();
                break;  // process only one capture, then poll for more move events before processing another capture
            }
        }
    } while (cam_channel_notdone_bitmask_ != 0);
    // Release image buffers
    for (auto& csr : csrec_arr_) {
        csr.image_size = 0;
        csr.image_buf_raw_p = nullptr;
        csr.image_buf.reset();
    }
    SLOGF(SLOG_DEBUG, "Sweep has run");
    return cam_bitmask_;
}
