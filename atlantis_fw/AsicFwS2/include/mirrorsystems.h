#ifndef __MIRRORSYSTEMS_H__
#define __MIRRORSYSTEMS_H__

#ifdef __cplusplus

// #include "mirroractuatormapping.h"
// #include "logdefines.h"
// #include "utilities/domfile.h"
// #include "rotation3d.h"

#include "matrix.h"

# define M_PI       3.14159265358979323846  /* pi */

namespace ltaf {

/// Normalize axis
template <typename T>
void normalizeAxis( T iter )
{
    double norm2 = 0.0f;
    norm2 += iter[0] * iter[0]; norm2 += iter[1] * iter[1]; norm2 += iter[2] * iter[2];
    double inv_norm2 = 1.0 / std::sqrt( norm2 );
    iter[0] *= inv_norm2; iter[1] *= inv_norm2; iter[2] *= inv_norm2;
}


/// Data structure that stores the fixed parameters of a mirror system.
/// Considers mirror and camera as a fixed pair system.
/// Includes parameters for both.
struct MirrorSysParam
{
    Matrix3x3f  actual_cam_orient;      ///< Actual camera orientation.
    Vec3f       actual_cam_location;  ///< Actual camera location.
    Vec3f       rot_axis_pnt;         ///< A point on the rotation axis that parameterizes axis.
    Vec3f       rot_axis_vec;         ///< Direction of the rotation axis.
    Vec3f       start_normal;         ///< Mirror normal at the starting position (rotation angle is 0).
    float       dist_pnt_to_plane;// { 0.0f }; ///< Distance between rot_axis_pnt and the mirror plane.
    bool        flip_img_around_x;// =        false; ///< The system needs to flip the image around x axis (true) or y axis (false).
    Vec2f       mirror_angle_range;//{0.0f, 0.0f}; ///< The mirror angle range

    // add a default constructor
    // MirrorSysParam():
    //     actual_cam_orient(Matrix3x3f::Identity()),
    //     actual_cam_location({0,0,0}),
    //     rot_axis_pnt({0,0,0}),
    //     rot_axis_vec({0,0,0}),
    //     start_normal({0,0,0}),
    //     dist_pnt_to_plane(0),
    //     flip_img_around_x(false),
    //     mirror_angle_range({0,0}) {}

    // Returns angle between rotation axis vector and start normal (in degrees).
    float computeAxisNormalAngle() const
    {
        float dot_prod = start_normal.dot( rot_axis_vec ) / length(start_normal) / length(rot_axis_vec);
        return std::abs( std::acos( dot_prod ) ) * 180 / M_PI ;
    }

    Vec3f getPointOnMirror( const Vec3f & mirror_normal ) const
    {
        return rot_axis_pnt + mirror_normal * dist_pnt_to_plane;
    }

    Vec3f getPointOnStartMirrorPlane( ) const
    {
        return getPointOnMirror( start_normal );
    }

    // update mirror rot_axis_pnt and dist_pnt_to_plane so that rot_axis_pnt is as close as possible to actual_cam_location
    // wihtout actually changing the line
    void updateMirrorAxisAndPnt()
    {
       start_normal.normalize();
       rot_axis_vec.normalize();

        Vec3f pnt_on_mirror = getPointOnStartMirrorPlane();
        Vec3f rot_axis_pnt_new = rot_axis_pnt + rot_axis_vec * ( ( actual_cam_location - rot_axis_pnt ).dot( rot_axis_vec ) );
        float dist_pnt_to_plane_new = ( pnt_on_mirror - rot_axis_pnt_new ).dot( start_normal );
        rot_axis_pnt = rot_axis_pnt_new;
        dist_pnt_to_plane = dist_pnt_to_plane_new;
    }

    ///< Compute virtual camera location and orientation using the mirror rotation angle.
    void compVirtualCamUsingRotAngle( Vec3f    &    loc,              ///< Virtual camera location
                                      Matrix3x3f & orient,     ///< Virtual camera orientation
                                      float angle ) const;                ///< Rotation angle

    ///< Compute Extrinsics Rotation matrix and translation vector using the mirror rotation angle
    void compRandTUsingRotAngle( Vec3f & trans_vec,
                                 Matrix3x3f & rot_mat,
                                 float angle ) const;
private:
    ///< Compute the mirror normal given the mirror rotation angle
    Vec3f   compMirrorNormal( float angle )  const;   ///< Rotation angle (in degrees )
    Vec3f   computeVirtualCamLocation( const Vec3f & mirror_normal ) const;

};

}

#endif

#endif // MIRRORSYSTEMS_H
