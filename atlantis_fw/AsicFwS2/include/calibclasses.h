// this file contains CI side calibration related code
// that we require for disparity based focus to work
#ifndef __CALIBCLASSES_H__
#define __CALIBCLASSES_H__

#ifdef __cplusplus
#include <memory>
#include <cassert>
#include "matrix.h"
#include "line2.h"
#include "mirrorsystems.h"
#include "lightheader/include/lightheader.pb-c.h"

namespace ltaf {

constexpr short kNumModules = 16;
//constexpr short kNumAModules = 5;
//constexpr short kNonMoveableModules = 5 + 1 + 1; // B4, C5
//constexpr short kMoveableModules = 9;

enum ModuleName {
    A1 = 0,
    A2,
    A3,
    A4,
    A5,
    B1,
    B2,
    B3,
    B4,
    B5,
    C1,
    C2,
    C3,
    C4,
    C5,
    C6  // 15
};

enum MirrorType
{
    NotMoveable = 0,           ///< Not moveable
    Moveable        ///< Mirrors that can move. All B and C modules that are not glued.
};

struct MirrorActuatorMapping {
    float actuator_length_offset;
    float actuator_length_scale;
    float mirror_angle_offset;
    float mirror_angle_scale;
    float a; // param 3, discard first 3. ( 001)
    float b;
    float c;
    bool branch_left;
    // //default:
    // MirrorActuatorMapping():
    //     actuator_length_offset(0),
    //     actuator_length_scale(0),
    //     mirror_angle_offset(0),
    //     mirror_angle_scale(0),
    //     a(0),
    //     b(0),
    //     c(0),
    //     branch_left(true)
    // {}
};

// -------------------------------------------------------
//			calibdata
// -------------------------------------------------------
class CalibDataBase{
public:
    CalibDataBase():
        _K(Matrix3x3f::Identity()){}

    CalibDataBase(const Matrix3x3f & k):
        _K(k) {}

    virtual ~CalibDataBase() {}

    const Matrix3x3f & K() const { return _K;}
protected:
    Matrix3x3f _K;
};

class CalibData : public CalibDataBase
{
public:
    CalibData()
        : CalibDataBase()
        , _T( {0.0, 0.0, 0.0} )
    	, _R( Matrix3x3f::Identity() ) {}

    CalibData(const Matrix3x3f & k,
    		  const Matrix3x3f & r,
    		  const Vec3f	   & t)
        : CalibDataBase( k )
        , _T( t )
    	, _R( r ) {}

//    const Matrix3x3f & K() const { return _K;}

    const Vec3f & translation() const {return _T;}

    const Matrix3x3f & R() const {return _R;}

    void setCameraParameters( const Matrix3x3f & K,
                              const Matrix3x3f & R,
                              const Vec3f    &   translation );

    /// Scale the camera (e.g. reduce the pixel size) by
    /// adjusting the corresponding intrinsic matrix
    void scale( float s );

    /// Shift the camera parallel to the sensor plane.
    void shift( int delta_x, int delta_y );

protected:
    //Matrix3x3f  _K;                  /// Intrinsic matrix and its precomputed inversion.
    Vec3f       _T;                  /// Translation
    Matrix3x3f  _R;                  /// Rotation matrix
};

class CalibDataWMirror : public CalibDataBase 
{
public:
    // default constructor
    CalibDataWMirror(){}

    CalibDataWMirror(const Matrix3x3f & k,
                     const MirrorSysParam & msp,
                     const MirrorActuatorMapping & map):
        CalibDataBase(k),
        _mirror_sys_param(msp),
        _mirror_act_mapping(map)
    {}

    const MirrorSysParam & mirror_sys_param() const             {return _mirror_sys_param;}
    const MirrorActuatorMapping & mirror_act_mapping() const    {return _mirror_act_mapping;}

private:
    MirrorSysParam _mirror_sys_param;
    MirrorActuatorMapping _mirror_act_mapping;
};


// ----------------------------------------------
// epifield
// ----------------------------------------------
class EpiField
{
public:
    EpiField(): _F( Matrix3x3f::Identity() ) {}

    EpiField( const CalibData & ref, const CalibData & src );

    void setup( const CalibData & ref, const CalibData & src );

    inline Line2f operator()( float x, float y ) const { return Line2f( _F * Vec3f( x, y, 1.0f ) );}

    Matrix3x3f getF() const {return _F;}

private:
    Matrix3x3f _F; // fundamental matrix;
};

// ----------------------------------------------
// warpfield
// ----------------------------------------------
class WarpField
{

public:
    WarpField( void ) : _mat( Matrix4x4f::Identity() ) {}

    WarpField( const CalibData & ref,
               const CalibData & src );

    /// 3D projection from (x,y,z) to (x',y',z')
    Vec3f project3D( float x, float y, float depth ) const ;

    Vec3f project3D( const Vec3f& in ) const
    {
        return project3D(in.x(), in.y(), in.z());
    }

private:
    Matrix4x4f _mat;
};

class CalibUtils{

// compute depth given point to point correspondence
public:
    static float depthGivenCorrespondence( const EpiField & epf,
                                       const CalibData & ref, const CalibData & src,
                                       float x, float y,
                                       float x_src, float y_src );
    // compute depth given point to point correspondence
    static float depthGivenCorrespondence( const CalibData & ref,
                                           const CalibData & src,
                                           float x, float y,
                                           float x_src, float y_src );

};


// stores and manages calib data. do not do any computation or funcationality
class CalibDataManager {

public:
    CalibDataManager() {
        setDefaultMirrorTypes(); // B4 C5 glued. set up default mirror types, and calib data defaults to identity
        hardCodeCalib();    // hardcode A1 A5 calibration data for now.
    }

    CalibDataManager(const Ltpb__LightHeader& lh);

    bool isModuleCalibrated(ModuleName m) const; // check if module has been calibrated, present in calib data
    bool isModuleMirrorMoveable(ModuleName m) const; // check if mirror is moveable
    void setMirrorTypes(ModuleName m, MirrorType t);
    const CalibDataBase * getCamParams(ModuleName m) const ;

private:

    void setDefaultMirrorTypes();

    void hardCodeCalib(); // for debugging only

    std::unique_ptr<CalibDataBase> _cam_params[kNumModules]; // store pointers to calib data (base class)

    MirrorType _cam_mirror_types[kNumModules]; // store mirror types for each module  

    bool parseGeometricCalibData(const Ltpb__LightHeader& lh);

    bool parseGeometricCalibData(unsigned char * mem_address, size_t nbytes);

    void parseMirrorSystem(const Ltpb__MirrorSystem *mirror_system, MirrorSysParam & msp);

    void parseMirrorActuatorMapping(const Ltpb__MirrorActuatorMapping * m, MirrorActuatorMapping & mam);

};


} // end namespace ltaf

#endif

#endif
