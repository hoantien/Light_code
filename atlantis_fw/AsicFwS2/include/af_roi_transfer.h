/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    af_roi_transfer.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-22-2016
 * @brief   Implements ROI transfer
 *
 ******************************************************************************/

#ifndef __AF_ROI_TRANSFER_H__
#define __AF_ROI_TRANSFER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Runs ROI transfer on given camera modules
 * @param cam_channel_bitmask Bitmask of camera module channels on which to run ROI transfer
 * @return Camera module bitmask used to update command status
 */
uint32_t af_roi_transfer_execute(uint32_t cam_channel_bitmask);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstdint>

class af_roi_transfer {
public:
    std::uint32_t run(std::uint32_t cam_channel_bitmask);

private:
    static constexpr std::size_t MAX_NUM_CAM_CH = 6;
};

#endif
#endif /* __AF_ROI_TRANSFER_H__ */
