/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    calib_data_manager.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-8-2016
 * @brief   Calibration data manager singleton
 *
 ******************************************************************************/

#ifndef __CALIB_DATA_MANAGER_H__
#define __CALIB_DATA_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

void initialize_calib_data_manager(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <memory>
#include "lightheader/include/lightheader.pb-c.h"
#include "calibclasses.h"
#include "opticalzoom.h"

class calib_data_manager {
public:
    static calib_data_manager& instance();

    static void initialize_instance();

    const ltaf::CalibDataManager& geometric_calib() const {
        return calib_data_mgr_;
    }

    const ltaf::OpticalZoomASIC& optical_zoom() const {
        return optical_zoom_;
    }

private:
    static std::unique_ptr<calib_data_manager> instance_;

    ltaf::CalibDataManager calib_data_mgr_;
    ltaf::OpticalZoomASIC optical_zoom_;
};

#endif
#endif /* __CALIB_DATA_MANAGER_H__ */
