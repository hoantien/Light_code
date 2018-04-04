#include <limits>
#include <memory>
#include <algorithm>
#include "log.h"
#include "operator_new.h"
#include "disparityfocus.h"
#include "image_raw10.h"
#include "basic_image.h"

#define SLOGF_ID                SLOG_ID_AF


namespace ltaf {

constexpr static float kMinFocusingDepth = 200.0f; // in mm, how close we can focus
// How far we focus (60m) this will bound how much we search in aux
constexpr static float kMaxFocusingDepth = 60000.0f;
constexpr static float kBFoverAF = 70.0f / 28.0f;

constexpr static std::uint16_t kCollapseFactor = 2; // how much to downsample from the image data (to save space) for integral image computation. 2 means collapse every 2 lines (GRBG block)
constexpr static std::uint16_t kCellHeight = 8; // size of the cell for computing the features. the patch will be divided into nx x ny cells.
constexpr static std::uint16_t kCellWidth = 8;
constexpr static std::uint16_t kNumFeaturesPerCell = 5;

std::uint16_t computeFeatureLength(std::uint16_t n_cells_x, std::uint16_t n_cells_y) {
    return n_cells_x * n_cells_y * kNumFeaturesPerCell;
}



void DisparityFocus::setup()
{
    // Set up epifield and warpfield
    _epf_A1A5 = EpiField( *_A1_calib, *_A5_calib );
    _epf_B4A5 = EpiField( *_B4_calib, *_A5_calib );

    _wpf_A1A5 = WarpField( *_A1_calib, *_A5_calib );
    _wpf_B4A5 = WarpField( *_B4_calib, *_A5_calib );
}


// assume aux capture rect is big enough.
// scale is the same for both sensor captures.
// all rect are full res reference frame.
float DisparityFocus::computeDepth( const void * img_pri_ptr, const rectangle & pri_capture_rect, const rectangle & pri_roi_rect,
                                    const void * img_aux_ptr, const rectangle & aux_capture_rect, 
                                    std::uint16_t capture_mode, std::uint16_t scale, rectangle* aux_roi ) const {
    assert(capture_mode <= 1); // 0 is AB capture, 1 is BC capture

    // scale reflects decimation on the sensor, if scale == 1, then we get GRGRGR... or BGBGBG... in full
    // if scale == 2, we get GR(xx)GR(xx)... or BG(xx)BG(xx)... where (xx) are decimated. and calling next_value will skip (xx)
    // if scale == 4, we get GR(xx)(xx)(xx)GR(xx)(xx)(xx)... or BG(xx)(xx)(xx)BG(xx)(xx)(xx)... where (xx) are decimated. and calling next_value will skip (xx)

    // figure out target img size
    std::uint16_t total_downsampling = scale * kCollapseFactor; // how much total downsampling, compared to full res.
    std::uint16_t pri_patch_width = pri_roi_rect.width() / total_downsampling;
    std::uint16_t pri_patch_height = pri_roi_rect.height() / total_downsampling;
    std::uint16_t aux_patch_width = aux_capture_rect.width() / total_downsampling;
    std::uint16_t aux_patch_height = aux_capture_rect.height() / total_downsampling;

    // process pri image roi/patch and compute features
    std::uint16_t npix_per_line = pri_capture_rect.width() / scale;
    image_raw10 img(img_pri_ptr, npix_per_line); // account for potential sensor decimation

    std::uint16_t y_offset = pri_roi_rect.top_y() - pri_capture_rect.top_y();
    std::uint16_t x_offset = pri_roi_rect.left_x() - pri_capture_rect.left_x();

    ImageU16 pri_patch(pri_patch_width, pri_patch_height); // create collapsed image for pri patch (downscaled: sensor decimation, and collapsed sampling/averaging)

    // todo: check offsets are correct given scale...
    for (std::uint16_t y = 0; y < pri_roi_rect.height(); y+=scale) {
        // assumption: we are going through every line from image data (after sensor decimation)
        std::uint16_t line_number = y / scale; // the line number in the image data
        image_pixel_iter line_start = img.get_iter( (y + y_offset) / scale,  x_offset / scale ); // make sure we are getting the right line
        collapseImage(pri_patch, line_number, line_start, npix_per_line, kCollapseFactor);
    }

    // for BC capture, we will have to downsample this patch
    if (capture_mode == 1) {
        ImageU16 pri_downsampled;
        downsample(pri_patch, pri_downsampled, kBFoverAF);
        std::swap(pri_patch, pri_downsampled); // now pri_patch is the downsampled patch
        // update patch size
        pri_patch_width = pri_patch.width();
        pri_patch_height = pri_patch.height();
    }

    // now compute features integral image from pri patch..
    computeIntegralImageInPlace(pri_patch);

    // feature will ignore the last few cols and rows if not divisible
    std::uint16_t n_cells_x = pri_patch_width / kCellWidth;
    std::uint16_t n_cells_y = pri_patch_height / kCellHeight;
    std::uint16_t feature_length =  computeFeatureLength(n_cells_x, n_cells_y);

    // todo: this sits in DDR or BTCM?
    std::unique_ptr<float[]> features(new(Heap::DDR) float[feature_length]);

    // compute features from pri patch integral image
    computeFeaturesGeneral( pri_patch, 0, 0, pri_patch_width, pri_patch_height,  // size of the patch over which to compute feature
                            kCellWidth, kCellHeight,    // size of cell, must not be more than 64 pixels, try 8x8 usually
                            features.get());

    //---------------------------------------
    // todo:
    // now we can free pri_patch memory
    //---------------------------------------
    pri_patch.emptify();

    // process aux img rectangle
    npix_per_line = aux_capture_rect.width() / scale;
    img = image_raw10(img_aux_ptr, npix_per_line);
    ImageU16 aux_patch(aux_patch_width, aux_patch_height); // create collapsed image for aux patch (downscaled: sensor decimation, and collapsed sampling/averaging)
    // todo: check offsets are correct given scale...
    for (std::uint16_t y = 0; y < aux_capture_rect.height(); y+=scale) {
        // assumption: we are going through every line from image data (after sensor decimation)
        std::uint16_t line_number = y / scale; // the line number in the image data
        image_pixel_iter line_start = img.get_iter( line_number,  0 ); // make sure we are getting the right line
        collapseImage(aux_patch, line_number, line_start, npix_per_line, kCollapseFactor);
    }
    // now compute features integral image from aux patch..
    computeIntegralImageInPlace(aux_patch);

    // -------------- compute epipolar line -------------
    std::uint16_t roi_center_x, roi_center_y;
    getROICenter(pri_roi_rect, roi_center_x, roi_center_y);

    Line2f epiline; // full res scale, same as calibraion
    if (capture_mode == 0) {
        epiline = _epf_A1A5(roi_center_x, roi_center_y); // will upcast to float
    } else {
        epiline = _epf_B4A5(roi_center_x, roi_center_y);
    }

    std::uint16_t pri_patch_half_width = pri_patch_width / 2; // this is after downsampling.
    std::uint16_t pri_patch_half_height = pri_patch_height / 2;

    // loop through and find best offset
    // assume A1 A5, no need to downsample by 2.5 to account for scale difference.
    // work on B4 A5 later.

    float best_cost = std::numeric_limits<float>::max();
    std::uint16_t best_offset_x;
    std::uint16_t best_offset_y;
    float best_x_calib = 0.0f;
    float best_y_calib = 0.0f;

    // Patch center coord in image space.
    float cxmin = projectPriToAux(Vec3f(roi_center_x, roi_center_y, kMinFocusingDepth), capture_mode).x();
    float cxmax = projectPriToAux(Vec3f(roi_center_x, roi_center_y, kMaxFocusingDepth), capture_mode).x();
    if (cxmin > cxmax)
        std::swap(cxmin, cxmax);
    cxmin = std::max(0.0f, cxmin);
    cxmax = std::max(0.0f, cxmax);
    std::uint16_t center_x_calib_min = cxmin;
    std::uint16_t center_x_calib_max = cxmax;

    for (std::uint16_t curr_x_calib = center_x_calib_min; curr_x_calib < center_x_calib_max; curr_x_calib += total_downsampling) {
        // Compute y coord in calibration space
        float curr_y_calib = epiline.yGivenX(curr_x_calib);
        
        // Compute x and y coord in image space
        float curr_x_img = ( curr_x_calib - aux_capture_rect.left_x() ) / total_downsampling;

        float start_x_f = std::round(curr_x_img - pri_patch_half_width);
        // Check to make sure always within bounds
        if (start_x_f < 0.0f || start_x_f + pri_patch_width > aux_patch_width)
            continue;

        float curr_y_img_e = ( curr_y_calib - aux_capture_rect.top_y() ) / total_downsampling;
        for (int i = -2; i < 3; ++i) {
            float curr_y_img = curr_y_img_e + i * 1.0f;

            float start_y_f = std::round(curr_y_img - pri_patch_half_height);
            if (start_y_f < 0.0f || start_y_f + pri_patch_height > aux_patch_height)
                continue;

            std::uint16_t start_x = start_x_f;
            std::uint16_t start_y = start_y_f;

            bool updated = computeFeaturesGeneralAndUpdate(aux_patch, start_x, start_y, // where the starting point is to compute
                                            pri_patch_width, pri_patch_height,  // size of the patch over which to compute feature
                                            kCellWidth, kCellHeight,    // size of cell, must not be more than 64 pixels, try 8x8 usually
                                            features.get(),               // target feature to compare against
                                            best_offset_x, best_offset_y, best_cost );

            if (updated) { // keep track of the best
                best_x_calib = curr_x_calib;
                best_y_calib = curr_y_calib;
            }
        }
    } // end looping over patches in aux

    SLOGF(SLOG_DEBUG, "Optimal aux ROI center: %d, %d", (int)best_x_calib, (int)best_y_calib);
    SLOGF(SLOG_DEBUG, "Optimal aux ROI left/top offset (subsampled): %d, %d", (int)best_offset_x, (int)best_offset_y);
    float mode_scale = (capture_mode == 0) ? 1.0f : (1.0f / kBFoverAF);
    aux_roi->set(best_x_calib - mode_scale * pri_roi_rect.width()/2, best_y_calib - mode_scale * pri_roi_rect.height()/2,
                 mode_scale * pri_roi_rect.width(), mode_scale * pri_roi_rect.height());

    // Compute depth
    const EpiField& epf = (capture_mode == 0) ? _epf_A1A5 : _epf_B4A5;
    const CalibData * pri_calib = (capture_mode == 0) ? _A1_calib : _B4_calib;
    const CalibData * aux_calib = (capture_mode == 0) ? _A5_calib : _A5_calib;
    float depth = CalibUtils::depthGivenCorrespondence(epf, *pri_calib, *aux_calib,
                                                roi_center_x, roi_center_y, // pri roi center point
                                                best_x_calib, best_y_calib);// aux best match center point
    return depth;
}


// compute the center of the roi given the roi rectangle
void DisparityFocus::getROICenter(const rectangle & roi_fullres, std::uint16_t & center_x, std::uint16_t & center_y) const {
    std::uint16_t roi_center_x = roi_fullres.left_x() + roi_fullres.width()/2;
    std::uint16_t roi_center_y = roi_fullres.top_y() + roi_fullres.height()/2;

    center_x = roi_center_x; // roi reference frame is the same as calibration
    center_y = roi_center_y;
}


Vec3f DisparityFocus::projectPriToAux(const Vec3f& pri_coord, std::uint16_t capture_mode) const
{
    const WarpField& wf = (capture_mode == 0) ? _wpf_A1A5 : _wpf_B4A5;
    return wf.project3D(pri_coord);
}


// given current ROI rectangle in primary view, compute the rectangle we need to capture (search band) in aux view.
// pri_roi is the exact size of the query roi, size and offset is in full res reference frame
// capture mode 0 = A1 A5 (AB mode), 1 = B4 A5 (BC mode)
rectangle DisparityFocus::computeAuxRectangle(const rectangle & pri_roi_fullres, std::uint16_t capture_mode) const
{
    assert(capture_mode == 0 || capture_mode == 1);

    // First compute the roi center (full res scale)
    std::uint16_t center_x, center_y;
    getROICenter(pri_roi_fullres, center_x, center_y);

    std::uint16_t aux_roi_half_width = pri_roi_fullres.width() / 2;
    std::uint16_t aux_roi_half_height = pri_roi_fullres.height() / 2;

    if (capture_mode == 1){
        // A5 is at a smaller scale than B4. need to downsample B4 later.
        aux_roi_half_width = std::floor( pri_roi_fullres.width() / kBFoverAF ) / 2;
        aux_roi_half_height = std::floor( pri_roi_fullres.height() / kBFoverAF ) / 2;
    }

    Vec3f min_depth_bound_calib{projectPriToAux(Vec3f(center_x, center_y, kMinFocusingDepth), capture_mode)};
    Vec3f max_depth_bound_calib{projectPriToAux(Vec3f(center_x, center_y, kMaxFocusingDepth), capture_mode)};
    
    // Compute boundaries but these might be out of bounds.
    // These can be negative
    int left_x = std::min(min_depth_bound_calib.x(), max_depth_bound_calib.x()) - aux_roi_half_width;
    int top_y = std::min(min_depth_bound_calib.y(), max_depth_bound_calib.y()) - aux_roi_half_height;
    int right_x = std::max(min_depth_bound_calib.x(), max_depth_bound_calib.x()) + aux_roi_half_width;
    int bot_y = std::max(min_depth_bound_calib.y(), max_depth_bound_calib.y()) + aux_roi_half_height;

    // Clip the top left corner
    left_x = std::max(0, left_x);
    top_y = std::max(0, top_y);
    assert(right_x > left_x);
    assert(bot_y > top_y);

    return rectangle( std::uint16_t(left_x), std::uint16_t(top_y), std::uint16_t(right_x - left_x), std::uint16_t(bot_y - top_y));

}

} // end namespace ltaf
