/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    af_roi_transfer.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-15-2016
 * @brief   Implements ROI transfer
 *
 ******************************************************************************/

#include <memory>
#include "os.h"
#include "operator_new.h"
#include "log.h"
#include "cortex_r4.h"
#include "actuator.h"
#include "roitransfer.h"
#include "contrast_sweep_manager.h"
#include "calib_data_manager.h"
#include "optical_zoom_manager.h"
#include "af_roi_transfer.h"


#define SLOGF_ID                SLOG_ID_AF

extern "C" uint32_t af_roi_transfer_execute(uint32_t cam_channel_bitmask) {
    optical_zoom_manager::instance().wait_for_mirrors_move_done();
    af_roi_transfer afrt;
    return afrt.run(cam_channel_bitmask);
}

std::uint32_t af_roi_transfer::run(std::uint32_t cam_channel_bitmask) {

    // Transfer ROI to requested camera modules
    for (std::size_t i = 0; i < MAX_NUM_CAM_CH; ++i) {
        if ((cam_channel_bitmask & (1 << i)) != 0) {
            // Channel i refinement
            SLOGF(SLOG_INFO, "ROI transfer of cam ch %d", (int)i);
            cam_typedef_t *pcam = chan_to_object(i+1);
            std::uint8_t ref_mod_idx = light_system->settings->roi_transfer.ref_mod_idx;
            std::uint16_t left_x = light_system->settings->roi_transfer.ref_mod_roi.left_x;
            std::uint16_t top_y = light_system->settings->roi_transfer.ref_mod_roi.top_y;
            std::uint16_t width = light_system->settings->roi_transfer.ref_mod_roi.width;
            std::uint16_t height = light_system->settings->roi_transfer.ref_mod_roi.height;
            std::uint16_t roi_center_x = left_x + width/2;
            std::uint16_t roi_center_y = top_y + height/2;
            ltaf::ModuleName ref_mod = static_cast<ltaf::ModuleName>(ref_mod_idx-1);
            ltaf::ModuleName src_mod = static_cast<ltaf::ModuleName>(module_to_idx(pcam->info.module)-1);
            // Compute mirror hallcode
            std::uint16_t mirror_hc = 0;
            if (pcam->optical->mirr != nullptr) {
                actuator_t *mirr = static_cast<actuator_t *>(pcam->optical->mirr);
                actuator_read_position(mirr, &mirror_hc);
            }
            ltaf::Vec2f src_roi_center = ltaf::transferROI(ref_mod, src_mod,
                    calib_data_manager::instance().geometric_calib(), ltaf::Vec2f(roi_center_x, roi_center_y),
                    light_system->settings->roi_transfer.distance_mm, mirror_hc);

            SLOGF(SLOG_DEBUG, "ROI transfer to %X (mirror hc: %d): %f %f", (int)pcam->info.module,
                    (int)mirror_hc, src_roi_center.x(), src_roi_center.y());

            constexpr float calib_img_width{4160.0f};
            constexpr float calib_img_height{3120.0f};
            float src_left_x = src_roi_center.x() - width/2;
            if (src_left_x < 0.0f)
                src_left_x = 0.0f;
            else if (src_left_x + width > calib_img_width)
                src_left_x = calib_img_width - width;
            float src_top_y = src_roi_center.y() - height/2;
            if (src_top_y < 0.0f)
                src_top_y = 0.0f;
            else if (src_top_y + height > calib_img_height)
                src_top_y = calib_img_height - height;
            // Save ROI center in calibration coordinates ratio for later storing in light header
            rectangle src_roi(src_left_x, src_top_y, width, height);
            pcam->cam_common_data.roi_center_x = (src_roi.left_x() + src_roi.width()/2) / calib_img_width;
            pcam->cam_common_data.roi_center_y = (src_roi.top_y() + src_roi.height()/2) / calib_img_height;
            // Save ROI in sensor coordinates for contrast refinement
            rectangle abs_src_roi = calib_rectangle_to_abs(src_roi, pcam->image->flip);
            pcam->cam_common_data.roi_rectangle.left_x = abs_src_roi.left_x();
            pcam->cam_common_data.roi_rectangle.top_y = abs_src_roi.top_y();
            pcam->cam_common_data.roi_rectangle.width = abs_src_roi.width();
            pcam->cam_common_data.roi_rectangle.height = abs_src_roi.height();
            SLOGF(SLOG_DEBUG, "Abs ROI of %X: %u %u %u %u", (int)pcam->info.module,
                    pcam->cam_common_data.roi_rectangle.left_x, pcam->cam_common_data.roi_rectangle.top_y,
                    pcam->cam_common_data.roi_rectangle.width, pcam->cam_common_data.roi_rectangle.height);
        }
    }
    contrast_sweep_manager csm;
    csm.init_refinement(cam_channel_bitmask, light_system->settings->roi_transfer.distance_mm);
    return csm.execute_sweep();
}
