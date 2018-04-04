#include "mirrorsystems.h"

namespace ltaf {

Matrix3x3f RotFromAngleAndAxis( float angle_deg, Vec3f axis )
{
    axis = normalize( axis );
    float ux = axis.x(), uy = axis.y(), uz = axis.z();

    float rad = M_PI * angle_deg / 180.0f;
    float c = std::cos( rad ), s = std::sin( rad );
    float _c = 1 - c;

    return Matrix3x3f ( {c + ux * ux * _c,       ux * uy * _c - uz * s,  ux * uz * _c + uy * s,
                        uy * ux * _c + uz * s,  c + uy * uy * _c,       uy * uz * _c - ux * s,
                        uz * ux * _c - uy * s,  uz * uy * _c + ux * s,  c + uz * uz * _c });
}

Matrix3x3f outerProduct(const Vec3f & a, const Vec3f & b){
    return Matrix3x3f({ a.x() * b.x(), a.x() * b.y(), a.x() * b.z(),
                        a.y() * b.x(), a.y() * b.y(), a.y() * b.z(),
                        a.z() * b.x(), a.z() * b.y(), a.z() * b.z()});
}


Vec3f MirrorSysParam::computeVirtualCamLocation( const Vec3f & mirror_normal ) const
{
    // a point on the mirror plane
    Vec3f mirror_pnt = getPointOnMirror( mirror_normal );

    // distance from the actual camera location to the mirror plane
    float dist_cam_to_mirror = ( mirror_pnt - actual_cam_location ).dot( mirror_normal );
    Vec3f loc = actual_cam_location + mirror_normal * ( 2.0f * dist_cam_to_mirror );

    return loc;
}



/// First compute the mirror normal given the rotation angle.
/// Then reflect the actual camera orientation and location given mirror normal.
/// This gives us the virtual camera orientation and location.
/// Loc and orientation are in right-handed system.
void MirrorSysParam::compVirtualCamUsingRotAngle( Vec3f & loc, Matrix3x3f & orient, float angle ) const
{
    // Compute the virtual camera orientation

    // Compute mirror normal
    auto mirror_normal = compMirrorNormal( angle );

    //printf("mirror normal = (%f, %f, %f)\n", mirror_normal.x(), mirror_normal.y(), mirror_normal.z());

    // Compute the mirror transformation matrix ( I - 2nn^T)
    Matrix3x3f mirror_xform_mat({1,0,0,0,1,0,0,0,1});
    mirror_xform_mat -= outerProduct( mirror_normal, mirror_normal ) * 2.0;

    // printf("mirror xform mat:\n");
    //     printf("%f, %f, %f\n", mirror_xform_mat(0,0), mirror_xform_mat(0,1), mirror_xform_mat(0,2));
    //     printf("%f, %f, %f\n", mirror_xform_mat(1,0), mirror_xform_mat(1,1), mirror_xform_mat(1,2));
    //     printf("%f, %f, %f\n", mirror_xform_mat(2,0), mirror_xform_mat(2,1), mirror_xform_mat(2,2));

    // Reflect camera orientation. It becomes left-handed.
    Matrix3x3f mirrored_orient = mirror_xform_mat * actual_cam_orient ;

    // Convert left-handed camera orientation to right-handed.
    if( flip_img_around_x )
    {
        orient = Matrix3x3f( {      mirrored_orient(0, 0), -mirrored_orient(0, 1), mirrored_orient(0, 2),
                                    mirrored_orient(1, 0), -mirrored_orient(1, 1), mirrored_orient(1, 2),
                                    mirrored_orient(2, 0), -mirrored_orient(2, 1), mirrored_orient(2, 2) } );
    }
    else
    {
        orient = Matrix3x3f( {      -mirrored_orient(0, 0), mirrored_orient(0, 1), mirrored_orient(0, 2),
                                    -mirrored_orient(1, 0), mirrored_orient(1, 1), mirrored_orient(1, 2),
                                    -mirrored_orient(2, 0), mirrored_orient(2, 1), mirrored_orient(2, 2)  });
    }

    // Compute the virtual camera location
    loc = computeVirtualCamLocation( mirror_normal );
}



void MirrorSysParam::compRandTUsingRotAngle(    Vec3f & trans_vec,
                                                Matrix3x3f & rot_mat,
                                                float angle ) const
{
    compVirtualCamUsingRotAngle( trans_vec, rot_mat, angle );

    rot_mat = transpose(rot_mat);

    trans_vec = ( rot_mat * trans_vec ) * -1.0f ;
}



Vec3f MirrorSysParam::compMirrorNormal( float angle ) const
{
    return Vec3f( RotFromAngleAndAxis(angle, rot_axis_vec) * start_normal );
}



}