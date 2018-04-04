#ifndef OPTICALZOOMASIC_H
#define OPTICALZOOMASIC_H


#ifdef __cplusplus

#include <array>
#include <bitset>

#define USE_PROTOBUF
#define USE_ANGLE_HALL_MAP


#ifdef USE_PROTOBUF
    #include "lightheader/include/lightheader.pb-c.h"
#endif

namespace ltaf
{

const int kNumMirrorModules = 8;
const int kNumCoverRegions  = 5;

struct Pnt2f
{
    Pnt2f( float xcoord, float ycoord)
        : x( xcoord )
        , y( ycoord )
    {}

    Pnt2f()
        : Pnt2f(0.0f, 0.0f)
    {}

    float x;
    float y;
};


enum MirrorModule
{
    M_B1 = 0,
    M_B2 = 1,
    M_B3 = 2,
    M_B5 = 3,
    M_C1 = 4,
    M_C2 = 5,
    M_C3 = 6,
    M_C4 = 7
};


enum CoverRegion
{
    TopLeft = 0,
    TopRight = 1,
    BotLeft = 2,
    BotRight = 3,
    Center = 4
};


struct AnglePosMapCoefsASIC
{
    float _pos_start_x, _pos_start_y;    // (pos 0)
    float _pos_end_x, _pos_end_y;      // (pos 1)
    float _angle_offset;    // (in rad)
    float _pos_scale;
    float _pos_offset;
};


#ifdef USE_ANGLE_HALL_MAP
struct AngleHallMapCoefsASIC
{
    float _map_coefs[6];
    float _min_mirror_angle = 34.0;   // in degree
    float _max_mirror_angle = 46.0;   // in degree
    /// TODO: currently we hard coded hall code range
    /// We should use the calibration information when the hall code range is more reliable
    float _min_hall_code = 0.0;
    float _max_hall_code = 1023.0;
    float _hall_offset;
    float _hall_scale;
    float _mirror_angle_offset;
    float _mirror_angle_scale;
};
#endif


class OpticalZoomASIC
{
public:
    enum class CaptureMode
    {
        AB,
        BC,
        C
    };

    OpticalZoomASIC()
    {
        initDefaultParams();
    }

#ifdef USE_PROTOBUF
    OpticalZoomASIC( const Ltpb__LightHeader & header )
    {
        loadZoomCalibFromProtoBuf( header );
    }

    void loadZoomCalibFromProtoBuf( const Ltpb__LightHeader & header );
#endif

    void getMirrorAnglesByZoomLevel( float zoom_level, CaptureMode capture_mode,
                                     float * angle_array ) const;

    void getMirrorAnglesByFocalLength( float focal_length,
                                       float * angle_array ) const;

#ifdef USE_ANGLE_HALL_MAP
    /// get the hall codes given focal length
    void getHallCodesByFocalLength( float focal_length,
                                    std::int16_t * hall_code_array ) const;

    /// get the hall codes given the zoom level and capture mode
    void getHallCodesByZoomLevel( float zoom_level, CaptureMode capture_mode,
                                  std::int16_t * hall_code_array ) const;
#endif

private:
    /// initialize default parameters
    void initDefaultParams();

#ifdef USE_PROTOBUF
    void loadModuleZoomCalibFromProtoBuf( MirrorModule mirror_module,
                                          const Ltpb__GeometricCalibration & geom_calib_pb );
#endif

    void getImgCenterInRef( float zoom_level,
                            CaptureMode capture_mode,
                            Pnt2f * img_center_in_ref_array ) const;

    void getCoverRegionCenterInRefAB( float zoom_level,
                                      Pnt2f * cover_region_center_in_ref_array ) const;

    void getCoverRegionCenterInRefBC( float zoom_level,
                                      Pnt2f * cover_region_center_in_ref_array ) const;


    float getMirrorAngleGivenImgCenterInRef( const MirrorModule mirror_module,
                                             const Pnt2f & img_center_in_ref ) const;

#ifdef USE_ANGLE_HALL_MAP
    std::int16_t getHallCodeGivenImgCenterInRef( const MirrorModule mirror_module,
                                                 const Pnt2f & img_center_in_ref ) const;

    std::int16_t getHallCodeGivenMirrorAngle( const MirrorModule mirror_module,
                                              const float mirror_angle ) const;

    std::array<AngleHallMapCoefsASIC, kNumMirrorModules> _angle_hall_map_array;
#endif

    std::array<AnglePosMapCoefsASIC, kNumMirrorModules> _angle_pos_map_array;
    std::array<CoverRegion, kNumMirrorModules> _module_coverage_array;
    std::bitset<kNumMirrorModules> _is_zoom_calib_available;
    float _focal_prime35 = 28.0f;
    float _focal_prime70 = 70.0f;
    float _focal_prime150 = 150.0f;
    float _img_size_x = 4160.0f;
    float _img_size_y = 3120.0f;
    float _min_zoom_level_AB_mode = 1.0f;
    float _max_zoom_level_AB_mode = 2.0f;
    float _min_zoom_level_BC_mode = 1.0f;
    float _max_zoom_level_BC_mode = 2.0f;
};
}

#endif
#endif // OPTICALZOOMASIC_H
