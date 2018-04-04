#ifndef __DISPARITYFOCUS_H__
#define __DISPARITYFOCUS_H__

#ifdef __cplusplus

#include <string>
#include "calibclasses.h"
#include "rectangle.h"


namespace ltaf {

/// this class is used to for fast disparity and depth computation on-device.
/// there is a JNI wrapper around this class in the engineering-demo-app
class DisparityFocus
{

public:

    DisparityFocus() = delete;

    // use default calib data for A1 and A5 from p2-6.
    DisparityFocus( const CalibDataManager & manager ):
        _A1_calib(static_cast<const CalibData *>(manager.getCamParams(ModuleName::A1))),
        _A5_calib(static_cast<const CalibData *>(manager.getCamParams(ModuleName::A5))),
        _B4_calib(static_cast<const CalibData *>(manager.getCamParams(ModuleName::B4)))
    {
        setup(); 
    }


    DisparityFocus( const CalibData * calib_A1,
                    const CalibData * calib_A5,
                    const CalibData * calib_B4 ): //keep everything full res.
        _A1_calib(calib_A1),
        _A5_calib(calib_A5),
        _B4_calib(calib_B4)
    {
        setup();
    }

    // given current ROI rectangle in primary view, compute the rectangle we need to capture (search band) in aux view.
    // pri_roi is the exact size of the query roi, size and offset is in full res reference frame
    // capture mode 0 = A1 A5 (AB mode), 1 = B4 A5 (BC mode)
    rectangle computeAuxRectangle(const rectangle & pri_roi_fullres, std::uint16_t capture_mode) const;

    // assume aux capture rect is big enough.
    // scale is the same for both sensor captures.
    // all rect are full res reference frame.
    float computeDepth(const void * img_pri_ptr, const rectangle & pri_capture_rect, const rectangle & pri_roi_rect,
                       const void * img_aux_ptr, const rectangle & aux_capture_rect, 
                        std::uint16_t capture_mode, std::uint16_t scale, rectangle* aux_roi) const;

private:

    Vec3f projectPriToAux(const Vec3f& pri_coord, std::uint16_t capture_mode) const;

    // compute the center of the roi given the roi rectangle
    void getROICenter(const rectangle & roi_fullres, std::uint16_t & center_x, std::uint16_t & center_y) const;

    void setup();

    // upon loading only read the relevant camera params (we only care about A1 A5 and B4 for now)
    const CalibData * _A1_calib = nullptr; ///< stores all the calib data we care about, assume A1 A5 B4 are all we need
    const CalibData * _A5_calib = nullptr;
    const CalibData * _B4_calib = nullptr;

    // precompute epf and wpf for A1-A5 mode, B4-A5 mode
    EpiField _epf_A1A5;
    WarpField _wpf_A1A5;
    EpiField _epf_B4A5;
    WarpField _wpf_B4A5;
};

} // end namespace ltaf

#endif

#endif // DISPARITYFOCUS_H
