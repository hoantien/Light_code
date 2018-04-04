/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    calib_data_manager.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-8-2016
 * @brief   Calibration data manager singleton
 *
 ******************************************************************************/

#include "log.h"
#include "operator_new.h"
#include "light_system.h"
#include "ddr_heap.h"
#include "calib_data_manager.h"

#define SLOGF_ID                SLOG_ID_AF


extern "C" void initialize_calib_data_manager(void) {
    calib_data_manager::initialize_instance();
}


std::unique_ptr<calib_data_manager> calib_data_manager::instance_{};

calib_data_manager& calib_data_manager::instance() {
    return *instance_;
}

namespace {
    void* alloc_pbuf(void *allocator_data, size_t size) {
        return gen_pvPortMalloc(static_cast<heap_attribute_t*>(allocator_data), size);
    }

    void free_pbuf(void *allocator_data, void *pointer) {
        gen_vPortFree(static_cast<heap_attribute_t*>(allocator_data), pointer);
    }

    ProtobufCAllocator pbuf_alloc{alloc_pbuf, free_pbuf, &ddr_heap_attribute};
}

void calib_data_manager::initialize_instance() {
    instance_.reset(new calib_data_manager{});
    std::size_t calib_data_size = get_calib_size();
    SLOGF(SLOG_INFO, "Calibration data size: %u", (unsigned int) calib_data_size);
    const std::size_t lri_header_size{32};
    if ((calib_data_size <= lri_header_size) || (calib_data_size == 0xffffffff)) {
        SLOGF(SLOG_ERROR, "Calibration not present in flash");
        return;
    }
    std::size_t light_header_size{calib_data_size - lri_header_size};
    std::unique_ptr<unsigned char[]> calib_data{new(Heap::DDR) unsigned char[light_header_size]};
    copy_calib_data(lri_header_size, calib_data.get(), light_header_size);
    SLOGF(SLOG_DEBUG, "Calibration data[0..]: %02x %02x %02x %02x", calib_data.get()[0], calib_data.get()[1], calib_data.get()[2], calib_data.get()[3]);
    // Unpack the message using protobuf-c.
    Ltpb__LightHeader* lh{ltpb__light_header__unpack(&pbuf_alloc, light_header_size, calib_data.get())};
    if (lh == nullptr) {
        SLOGF(SLOG_ERROR, "Failed to unpack light_header protobuf");
    } else {
        instance_->calib_data_mgr_ = ltaf::CalibDataManager{*lh};
        instance_->optical_zoom_.loadZoomCalibFromProtoBuf(*lh);
        ltpb__light_header__free_unpacked(lh, &pbuf_alloc);
    }
}
