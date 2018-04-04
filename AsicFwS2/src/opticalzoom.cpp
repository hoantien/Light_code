
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "log.h"
#include "opticalzoom.h"

#define SLOGF_ID                SLOG_ID_AF


using namespace ltaf;

/// convert radian to degree
namespace
{
    const float kPi = 3.14159265358979323846f;

    float rad2deg( float rad ) { return rad * ( 180.0f / kPi ); }

    const std::int16_t kInvalidHall = -1;
    const float kDefaultMirrorAngle = 45.0f;
}

#ifdef USE_PROTOBUF
void OpticalZoomASIC::loadModuleZoomCalibFromProtoBuf( MirrorModule mirror_module,
                                                       const Ltpb__GeometricCalibration & geom_calib_pb )
{
    bool has_optical_zoom = false;
    if( geom_calib_pb.angle_optical_center_mapping!=nullptr &&
        geom_calib_pb.mirror_type == LTPB__GEOMETRIC_CALIBRATION__MIRROR_TYPE__MOVABLE )
    {
        // set angle optical center mapping
        const Ltpb__GeometricCalibration__AngleOpticalCenterMapping & angle_optcenter_map =
            *(geom_calib_pb.angle_optical_center_mapping);

        _angle_pos_map_array[mirror_module]._angle_offset = angle_optcenter_map.angle_offset;
        _angle_pos_map_array[mirror_module]._pos_start_x  = angle_optcenter_map.center_start->x;
        _angle_pos_map_array[mirror_module]._pos_start_y  = angle_optcenter_map.center_start->y;
        _angle_pos_map_array[mirror_module]._pos_end_x    = angle_optcenter_map.center_end->x;
        _angle_pos_map_array[mirror_module]._pos_end_y    = angle_optcenter_map.center_end->y;
        _angle_pos_map_array[mirror_module]._pos_offset   = angle_optcenter_map.t_offset;
        _angle_pos_map_array[mirror_module]._pos_scale    = angle_optcenter_map.t_scale;

#ifdef USE_ANGLE_HALL_MAP
        for( uint i = 0; i < geom_calib_pb.n_per_focus_calibration; ++i )
        {
            const Ltpb__GeometricCalibration__CalibrationFocusBundle & focus_bundle = *(geom_calib_pb.per_focus_calibration[i]);
            if( focus_bundle.extrinsics!=nullptr )
            {
                const Ltpb__GeometricCalibration__Extrinsics & extrinsics = *(focus_bundle.extrinsics);
                if( extrinsics.moveable_mirror!=nullptr )
                {
                    if( extrinsics.moveable_mirror->mirror_actuator_mapping!=nullptr )
                    {
                        const Ltpb__MirrorActuatorMapping & mirror_mapping = *(extrinsics.moveable_mirror->mirror_actuator_mapping);
                        _angle_hall_map_array[mirror_module]._hall_offset = mirror_mapping.actuator_length_offset;
                        _angle_hall_map_array[mirror_module]._hall_scale  = mirror_mapping.actuator_length_scale;
                        // TODO: use hall code range from protobuf, however, we need to make sure it is set properly
                        _angle_hall_map_array[mirror_module]._min_hall_code = 0;
                        _angle_hall_map_array[mirror_module]._max_hall_code = 1023;
                        _angle_hall_map_array[mirror_module]._mirror_angle_offset = mirror_mapping.mirror_angle_offset;
                        _angle_hall_map_array[mirror_module]._mirror_angle_scale  = mirror_mapping.mirror_angle_scale;
                        if( extrinsics.moveable_mirror->mirror_system!=nullptr)
                        {
                            const Ltpb__MirrorSystem & msys = *(extrinsics.moveable_mirror->mirror_system);
                            _angle_hall_map_array[mirror_module]._min_mirror_angle = msys.mirror_angle_range->min_val;
                            _angle_hall_map_array[mirror_module]._max_mirror_angle = msys.mirror_angle_range->max_val;
                        }

                        if( mirror_mapping.quadratic_model!=nullptr )
                        {
                            const Ltpb__MirrorActuatorMapping__QuadraticModel & quadratic_model = *(mirror_mapping.quadratic_model);
                            for( int j = 0; j < 6; ++j )
                            { _angle_hall_map_array[mirror_module]._map_coefs[j] = quadratic_model.model_coeffs[j]; }

                            has_optical_zoom = true;
                            break;
                        }

                    }
                }
            }
        }
#else
        has_optical_zoom = true;
#endif
    }

    _is_zoom_calib_available[mirror_module] = has_optical_zoom;
}



void OpticalZoomASIC::loadZoomCalibFromProtoBuf( const Ltpb__LightHeader & header )
{
    initDefaultParams();
    for( uint i = 0; i < header.n_module_calibration; ++i )
    {
        const Ltpb__FactoryModuleCalibration & module_calib = *(header.module_calibration[i]);
        switch( module_calib.camera_id )
        {
            case LTPB__CAMERA_ID__B1:
                loadModuleZoomCalibFromProtoBuf( M_B1, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__B2:
                loadModuleZoomCalibFromProtoBuf( M_B2, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__B3:
                loadModuleZoomCalibFromProtoBuf( M_B3, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__B5:
                loadModuleZoomCalibFromProtoBuf( M_B5, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__C1:
                loadModuleZoomCalibFromProtoBuf( M_C1, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__C2:
                loadModuleZoomCalibFromProtoBuf( M_C2, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__C3:
                loadModuleZoomCalibFromProtoBuf( M_C3, *module_calib.geometry );
                break;
            case LTPB__CAMERA_ID__C4:
                loadModuleZoomCalibFromProtoBuf( M_C4, *module_calib.geometry );
                break;
            default:
                break;
        }
    }
}
#endif



void OpticalZoomASIC::getMirrorAnglesByFocalLength( float focal_length, float * angle_array ) const
{
    SLOGF(SLOG_DEBUG, "Computing mirror angles for focal_length %f", focal_length);
    if( focal_length < _focal_prime70 )
    {
        getMirrorAnglesByZoomLevel( focal_length / _focal_prime35, CaptureMode::AB, angle_array );
    }
    else
    {
        getMirrorAnglesByZoomLevel( focal_length / _focal_prime70, CaptureMode::BC, angle_array );
    }
}






void OpticalZoomASIC::getMirrorAnglesByZoomLevel( float zoom_level,
                                                  OpticalZoomASIC::CaptureMode capture_mode,
                                                  float * angle_array ) const
{
    Pnt2f img_center_in_ref[kNumMirrorModules];
    getImgCenterInRef( zoom_level, capture_mode, img_center_in_ref );

    for( int i = 0; i < kNumMirrorModules; ++i )
    {
        angle_array[i] = getMirrorAngleGivenImgCenterInRef( MirrorModule( i ), img_center_in_ref[i] );
        SLOGF(SLOG_DEBUG, "Computed mirror angle for module %d: %f", i, angle_array[i]);
    }
}



void OpticalZoomASIC::initDefaultParams()
{
    _focal_prime35 = 28.0f;
    _focal_prime70 = 70.0f;
    _focal_prime150 = 150.0f;
    _img_size_x = 4160.0;
    _img_size_y = 3120.0;

    _min_zoom_level_AB_mode = 1.25;
    _max_zoom_level_AB_mode = _focal_prime70 / _focal_prime35;
    _min_zoom_level_BC_mode = 1.05f;
    _max_zoom_level_BC_mode = _focal_prime150 / _focal_prime70;

    for( int i = 0; i < kNumMirrorModules; ++i )
    { _is_zoom_calib_available[i] = false; }

    _module_coverage_array[M_B1] = BotLeft;
    _module_coverage_array[M_B2] = BotRight;
    _module_coverage_array[M_B3] = TopRight;
    _module_coverage_array[M_B5] = TopLeft;
    _module_coverage_array[M_C1] = TopLeft;
    _module_coverage_array[M_C2] = BotRight;
    _module_coverage_array[M_C3] = BotLeft;
    _module_coverage_array[M_C4] = TopRight;
}



namespace
{
/// Helper function that gets image center positions in reference image
void getHiresImgCenterPosInRef( const Pnt2f & ref_center,
                                const Pnt2f & hires_fov_in_ref,
                                const Pnt2f & desired_fov_in_ref,
                                Pnt2f * hires_img_center_in_ref_array )
{
    float ref_center_x = ref_center.x;
    float ref_center_y = ref_center.y;

    float half_hires_fov_x = hires_fov_in_ref.x * 0.5f;
    float half_hires_fov_y = hires_fov_in_ref.y * 0.5f;

    float half_desired_fov_x = desired_fov_in_ref.x * 0.5f;
    float half_desired_fov_y = desired_fov_in_ref.y * 0.5f;


    float tr_corner_x = ref_center_x + half_desired_fov_x;
    float tr_corner_y = ref_center_y - half_desired_fov_y;

    float tl_corner_x = ref_center_x - half_desired_fov_x;
    float tl_corner_y = ref_center_y - half_desired_fov_y;

    float bl_corner_x = ref_center_x - half_desired_fov_x;
    float bl_corner_y = ref_center_y + half_desired_fov_y;

    float br_corner_x = ref_center_x + half_desired_fov_x;
    float br_corner_y = ref_center_y + half_desired_fov_y;

    // assign the image center position
    hires_img_center_in_ref_array[TopRight] = Pnt2f( tr_corner_x - half_hires_fov_x,
                                                     tr_corner_y + half_hires_fov_y );

    hires_img_center_in_ref_array[TopLeft] = Pnt2f( tl_corner_x + half_hires_fov_x,
                                                    tl_corner_y + half_hires_fov_y );

    hires_img_center_in_ref_array[BotLeft] = Pnt2f( bl_corner_x + half_hires_fov_x,
                                                    bl_corner_y - half_hires_fov_y );

    hires_img_center_in_ref_array[BotRight] = Pnt2f( br_corner_x - half_hires_fov_x,
                                                     br_corner_y - half_hires_fov_y );

    hires_img_center_in_ref_array[Center]   = Pnt2f( ref_center_x, ref_center_y );
}
}


void OpticalZoomASIC::getImgCenterInRef( float zoom_level ,
                                         CaptureMode capture_mode,
                                         Pnt2f * img_center_in_ref_array ) const
{
    Pnt2f cover_region_center_Bs_in_ref[kNumCoverRegions];
    Pnt2f cover_region_center_Cs_in_ref[kNumCoverRegions];

    if( capture_mode == CaptureMode::AB )
    {
        getCoverRegionCenterInRefAB( zoom_level, cover_region_center_Bs_in_ref );
        getCoverRegionCenterInRefBC( _min_zoom_level_BC_mode, cover_region_center_Cs_in_ref );
    }
    else
    {
        getCoverRegionCenterInRefAB( _max_zoom_level_AB_mode, cover_region_center_Bs_in_ref );
        getCoverRegionCenterInRefBC( zoom_level, cover_region_center_Cs_in_ref );
    }

    img_center_in_ref_array[M_B1] = cover_region_center_Bs_in_ref[_module_coverage_array[M_B1]];
    img_center_in_ref_array[M_B2] = cover_region_center_Bs_in_ref[_module_coverage_array[M_B2]];
    img_center_in_ref_array[M_B3] = cover_region_center_Bs_in_ref[_module_coverage_array[M_B3]];
    img_center_in_ref_array[M_B5] = cover_region_center_Bs_in_ref[_module_coverage_array[M_B5]];

    img_center_in_ref_array[M_C1] = cover_region_center_Cs_in_ref[_module_coverage_array[M_C1]];
    img_center_in_ref_array[M_C2] = cover_region_center_Cs_in_ref[_module_coverage_array[M_C2]];
    img_center_in_ref_array[M_C3] = cover_region_center_Cs_in_ref[_module_coverage_array[M_C3]];
    img_center_in_ref_array[M_C4] = cover_region_center_Cs_in_ref[_module_coverage_array[M_C4]];
}



void OpticalZoomASIC::getCoverRegionCenterInRefAB( float zoom_level, Pnt2f * cover_region_center_in_ref_array ) const
{
    zoom_level = std::max( std::min( zoom_level, _max_zoom_level_AB_mode ), _min_zoom_level_AB_mode );
    Pnt2f ref_center_in_A1( _img_size_x * 0.5f, _img_size_y * 0.5f );
    Pnt2f img_size_Bs_in_A1( _img_size_x * ( _focal_prime35 / _focal_prime70 ),
                             _img_size_y * ( _focal_prime35 / _focal_prime70 ) );
    Pnt2f desired_fov_in_ref( _img_size_x / zoom_level,
                              _img_size_y / zoom_level );

    getHiresImgCenterPosInRef( ref_center_in_A1,
                               img_size_Bs_in_A1,
                               desired_fov_in_ref,
                               cover_region_center_in_ref_array );
}



void OpticalZoomASIC::getCoverRegionCenterInRefBC( float zoom_level, Pnt2f * cover_region_center_in_ref_array ) const
{
    zoom_level = std::max( std::min( zoom_level, _max_zoom_level_BC_mode ), _min_zoom_level_BC_mode );

    Pnt2f ref_center_in_B4( _img_size_x * 0.5f,
                            _img_size_y * 0.5f );
    Pnt2f img_size_Cs_in_B4( _img_size_x * ( _focal_prime70 / _focal_prime150 ),
                             _img_size_y * ( _focal_prime70 / _focal_prime150 ) );
    Pnt2f desired_fov_in_ref( _img_size_x / zoom_level,
                              _img_size_y / zoom_level );

    getHiresImgCenterPosInRef( ref_center_in_B4,
                               img_size_Cs_in_B4,
                               desired_fov_in_ref,
                               cover_region_center_in_ref_array );
}

namespace
{
float mapPosToAngle( const AnglePosMapCoefsASIC & angle_pos_mapping, const Pnt2f & pos )
{
    // get the 1D param for the pos
    float vec_start_to_pos_x = pos.x - angle_pos_mapping._pos_start_x;
    float vec_start_to_pos_y = pos.y - angle_pos_mapping._pos_start_y;
    float vec_start_to_end_x = angle_pos_mapping._pos_end_x - angle_pos_mapping._pos_start_x;
    float vec_start_to_end_y = angle_pos_mapping._pos_end_y - angle_pos_mapping._pos_start_y;

    float r = ( vec_start_to_pos_x * vec_start_to_end_x + vec_start_to_pos_y * vec_start_to_end_y )
              / ( vec_start_to_end_x * vec_start_to_end_x + vec_start_to_end_y * vec_start_to_end_y );

    float angle = ( atan( ( r - angle_pos_mapping._pos_offset ) / angle_pos_mapping._pos_scale ) -
                    angle_pos_mapping._angle_offset ) * 0.5f;

    return rad2deg( angle );
}
}

float OpticalZoomASIC::getMirrorAngleGivenImgCenterInRef( const MirrorModule mirror_module,
                                                          const Pnt2f & img_center_in_ref ) const
{
    if (!_is_zoom_calib_available[mirror_module] )
    {
#if 0
        printf( "Mapping information for mirror module %d does not exist,"
                "return %f mirror angle instead!\n", mirror_module, kDefaultMirrorAngle );
#endif
        return kDefaultMirrorAngle;
    }

    return mapPosToAngle( _angle_pos_map_array[mirror_module], img_center_in_ref );
}


#ifdef USE_ANGLE_HALL_MAP
namespace
{
float mapAngleToHall( const AngleHallMapCoefsASIC & angle_hall_mapping, float mirror_angle )
{
    auto mirror_func = [&]( float angle )
    {return ( angle - angle_hall_mapping._mirror_angle_offset ) / angle_hall_mapping._mirror_angle_scale;};


    auto hall_func = [&]( float hall_param )
    {return hall_param * angle_hall_mapping._hall_scale + angle_hall_mapping._hall_offset;};


    float f_x     =  mirror_func( mirror_angle );
    float f_x_sqr = f_x * f_x;

    float f_y = - ( angle_hall_mapping._map_coefs[3] * f_x_sqr +
                    angle_hall_mapping._map_coefs[4] * f_x +
                    angle_hall_mapping._map_coefs[5] ) /
                ( angle_hall_mapping._map_coefs[0] * f_x_sqr +
                  angle_hall_mapping._map_coefs[1] * f_x +
                  angle_hall_mapping._map_coefs[2] );

    return hall_func( f_y );
}
}

std::int16_t OpticalZoomASIC::getHallCodeGivenImgCenterInRef( const MirrorModule mirror_module,
                                                              const Pnt2f & img_center_in_ref ) const
{
    return getHallCodeGivenMirrorAngle( mirror_module,
                                        getMirrorAngleGivenImgCenterInRef( mirror_module, img_center_in_ref ) );
}



int16_t OpticalZoomASIC::getHallCodeGivenMirrorAngle( const MirrorModule mirror_module,
                                                      const float mirror_angle ) const
{
    if (!_is_zoom_calib_available[mirror_module] )
    {
#if 0
        printf( "Mapping information for mirror module %d does not exist,"
                "return %d hall code instead!\n", mirror_module, kInvalidHall );
#endif
        return kInvalidHall;
    }

    const AngleHallMapCoefsASIC & angle_hall_mapping = _angle_hall_map_array[mirror_module];
    float hall_code = mapAngleToHall( angle_hall_mapping, mirror_angle );
    // clamp the hall code
    hall_code = std::max( std::min( hall_code, angle_hall_mapping._max_hall_code ),
                          angle_hall_mapping._min_hall_code );
    return std::int16_t( hall_code );
}



void OpticalZoomASIC::getHallCodesByFocalLength( float focal_length,
                                                 int16_t * hall_code_array ) const
{
    float mirror_angle_array[kNumMirrorModules];
    getMirrorAnglesByFocalLength( focal_length, mirror_angle_array );
    for( int i = 0; i < kNumMirrorModules; ++i )
    {
        hall_code_array[i] = getHallCodeGivenMirrorAngle( MirrorModule( i ), mirror_angle_array[i] );
        SLOGF(SLOG_DEBUG, "Computed mirror hallcode for module %d: %d", i, hall_code_array[i]);
    }
}



void OpticalZoomASIC::getHallCodesByZoomLevel( float zoom_level ,
                                               CaptureMode capture_mode,
                                               int16_t * hall_code_array ) const
{
    float mirror_angle_array[kNumMirrorModules];
    getMirrorAnglesByZoomLevel( zoom_level, capture_mode, mirror_angle_array );
    for( int i = 0; i < kNumMirrorModules; ++i )
    { hall_code_array[i] = getHallCodeGivenMirrorAngle( MirrorModule( i ), mirror_angle_array[i] );}
}
#endif
