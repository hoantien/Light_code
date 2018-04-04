/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    af_ae.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Sep-22-2016
 * @brief   Implements auto-exposure and auto-focus
 *
 ******************************************************************************/

#ifndef __AF_AE_H__
#define __AF_AE_H__

#include "os.h"

#ifdef __cplusplus
extern "C" {
#endif

void execute_af_ae(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class af_ae {
public:
    af_ae();

    ~af_ae();

    void run(const rectangle& pri_roi);

    void pri_mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq);

    void aux_mipi2axi_cb(uint8_t iidx, uint32_t signal_irq, uint32_t error_irq);

private:
    SemaphoreHandle_t pri_capture_sem_;
    SemaphoreHandle_t aux_capture_sem_;
};

#endif
#endif /* __AF_AE_H__ */
