/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    contrast_sweep.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-8-2016
 * @brief   Implements contrast optimization full sweep
 *
 ******************************************************************************/

#ifndef __CONTRAST_SWEEP_H__
#define __CONTRAST_SWEEP_H__

#include "os.h"
#include "light_system.h"
#include "contrast_refine_step.h"
#include "contrast_pass.h"
#include "rectangle.h"

#ifdef __cplusplus

#include <cstdint>
#include <array>
#include <limits>
#include <memory>

/// Implements contrast optimization sweep
class contrast_sweep {
public:
    /**
     * Constructor
     * @param cam Camera module object
     * @param scale Set to 1 for native resolution, 2 for scale/bin of 2 in x and y
     * @param min_capture_rect Rectangle that should be captured
     * @param image_buffer_p Pointer to image buffer (can hold min_capture_rect.image_size())
     * @param event_group Event group for lens move done and capture done events
     * @param lens_move_done_event Event to signal that lens move is completed
     * @param capture_done_event Event to signal that roi image capture is completed
     */
    contrast_sweep(cam_typedef_t& cam, std::uint16_t scale,
            const rectangle& min_capture_rect, void* image_buffer_p,
            EventGroupHandle_t event_group,
            EventBits_t lens_move_done_event, EventBits_t capture_done_event);

    /**
     * Configures one pass of contrast sweep operation
     * @param metric1_t Type of metric1 to be computed
     * @param metric1_t Type of metric2 to be computed
     * @param contrast_pass_p Pointer to contrast_pass object
     */
    void add_iter_param(contrast_processor::metric_type metric1_t,
            contrast_processor::metric_type metric2_t,
            std::unique_ptr<contrast_pass> contrast_pass_p);

    /**
     * Initialize sweep with passes as specified through add_iter_param()
     */
    void initialize_sweep(const rectangle& roi);

    /**
     * Start lens move
     */
    void start_lens_move();

    /**
     * Start frame capture
     */
    void start_capture_frame();

    /**
     * Process captured frame, compute metrics
     * @return true iff sweep is completed
     * After true is returned, a last call to start_lens_move() is needed to move the lens to the optimal position
     */
    bool process_captured_frame();

    /**
     * @return Optimal lens position computed in last call to run_sweep().
     */
    std::int16_t optimal_position() const {
        return pass_param_arr_[num_pass_-1].contrast_pass_p->out_best_pos();
    }

    const rectangle& capture_rectangle() const {
        return contrast_step_.capture_rectangle();
    }

private:
    struct pass_param {
        contrast_processor::metric_type metric1_t;
        contrast_processor::metric_type metric2_t;
        std::unique_ptr<contrast_pass> contrast_pass_p;
    };

    void check_initialize_next_pass();

    static constexpr std::uint8_t MAX_NUM_PASS = 10;
    std::array<pass_param, MAX_NUM_PASS> pass_param_arr_;
    std::uint8_t num_pass_{0};
    contrast_refine_step contrast_step_;
    rectangle roi_{0,0,0,0};
    std::size_t pass_i_;
};

#endif
#endif   /* __CONTRAST_SWEEP_H__ */
