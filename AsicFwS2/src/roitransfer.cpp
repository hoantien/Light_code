
#include "roitransfer.h"
#include "log.h"


#define SLOGF_ID                SLOG_ID_AF

// check these ranges
constexpr int kMaxMirrorHallCode = 1000;
constexpr int kMinMirrorHallCode = 0;

namespace ltaf{ 

using Rotation3Dd = Matrix3x3d;
using Rotation3Df = Matrix3x3f;


//template<typename T>
// always return 2 solutions.
// if no solutions (det < 0), return closet point to x-axis
void SolveQuadEq( float a, float b, float c , 
                                float & solution1,  float & solution2 )
{
    float delta = b * b - 4 * a * c;
    if( std::abs( delta ) < 1e-5 ) { delta = 0.0; }
    if( delta < 0 )
    {
        // in case the user always wants a solution, we return the best solution we can get
        solution1 = -b / ( 2.0f * a );
        solution2 = solution1;
    }
    delta = std::sqrt( delta );
    solution1 = ( -b + delta ) / ( 2.0f * a );
    solution2 = ( -b - delta ) / ( 2.0f * a );
}
//template std::vector<float> SolveQuadEq( float a, float b, float c , bool solution_guranteed );



Matrix3x3f computeHomography( 	  const Matrix3x3f & ref_kmat,
                                  const Matrix3x3f & src_kmat,
                                  const Rotation3Df & ref_rot,
                                  const Rotation3Df & src_rot,
                                  const Vec3f & ref_trans,
                                  const Vec3f & src_trans,
                                  const float ref_depth )
{
    Matrix3x3f R_rel = src_rot * transpose( ref_rot );
    Vec3f T_rel = ( R_rel * ref_trans - src_trans );
    R_rel( 0, 2 ) -= T_rel.x() / ref_depth;
    R_rel( 1, 2 ) -= T_rel.y() / ref_depth;
    R_rel( 2, 2 ) -= T_rel.z() / ref_depth;

    return src_kmat * R_rel * inverse( ref_kmat );
}



float computeMirrorAngleGivenHallCode(std::uint16_t hall_code, 
                                        const MirrorActuatorMapping & m)
{
    // normalize hallcode
    float y = hall_code;
    y = (y - m.actuator_length_offset)/m.actuator_length_scale;

    // computes f(x)
    float root1, root2;
    SolveQuadEq( m.a, m.b, m.c + y, root1, root2 );

    if (root1 == root2){
        return root1;
    }
    else{
        //multi scale + offset back to angle space
        if (m.branch_left)  // double check, are we returning the correct root
            return root1 * m.mirror_angle_scale + m.mirror_angle_offset;
        else
            return root2 * m.mirror_angle_scale + m.mirror_angle_offset;
    }
}


// return (-1 -1) if fails
Vec2f transferROI(ModuleName ref_module_name,       // Reference module name
                  ModuleName src_module_name,       // Source module name
                  const CalibDataManager& manager,  // Calib data manager
			      const Vec2f&   roi_center,        // Reference roi center coord in pixels
			      float depth_in_mm,                // Depth in mm. If -1, then we use infinity
			      std::uint16_t mirror_hall_code )  // Mirror hall code. If its -ve, we ignore the hall code.
{
    if (!manager.isModuleCalibrated(ref_module_name) || !manager.isModuleCalibrated(src_module_name)){
        SLOGF(SLOG_ERROR, "transferROI(): modules not calibrated %i, %i", ref_module_name, src_module_name);
        return Vec2f(-1, -1);
    }

    if (manager.isModuleMirrorMoveable(ref_module_name)){
        SLOGF(SLOG_ERROR, "transferROI(): ref module (%i) with moveable mirror not supported", ref_module_name);
        return Vec2f(-1, -1);
    }

    const CalibData * ref_calib_ptr = static_cast<const CalibData*>(manager.getCamParams(ref_module_name));

	Matrix3x3f h;

    // dealing with mirror modules
	if (manager.isModuleMirrorMoveable(src_module_name)) {

		if ( mirror_hall_code > kMaxMirrorHallCode || mirror_hall_code < kMinMirrorHallCode ) {
            SLOGF(SLOG_ERROR, "transferROI(): mirror_hall_code out of range %i, %i", ref_module_name, src_module_name);
            return Vec2f(-1, -1);
		}

        const CalibDataWMirror * src_calib_ptr = static_cast<const CalibDataWMirror*>(manager.getCamParams(src_module_name));

        const MirrorActuatorMapping & m = src_calib_ptr->mirror_act_mapping();

        float mirror_angle = computeMirrorAngleGivenHallCode(mirror_hall_code, m);

        SLOGF(SLOG_INFO, "Mirror angle: %f", mirror_angle);

        Vec3f src_trans_vec;        // updated t
        Matrix3x3f src_rot_mat;     // updated R
        (src_calib_ptr->mirror_sys_param()).compRandTUsingRotAngle( src_trans_vec, src_rot_mat, mirror_angle );

        // printf("src_rot_mat:");
        //     printf("%f, %f, %f\n", src_rot_mat(0,0), src_rot_mat(0,1), src_rot_mat(0,2));
        //     printf("%f, %f, %f\n", src_rot_mat(1,0), src_rot_mat(1,1), src_rot_mat(1,2));
        //     printf("%f, %f, %f\n", src_rot_mat(2,0), src_rot_mat(2,1), src_rot_mat(2,2));
        // printf("src_trans_vec: %f %f %f\n", src_trans_vec.x(), src_trans_vec.y(), src_trans_vec.z());

		h = computeHomography(	ref_calib_ptr->K(),
								src_calib_ptr->K(),
								ref_calib_ptr->R(),
								src_rot_mat,
								ref_calib_ptr->translation(),
								src_trans_vec,
								depth_in_mm);
	} else {
	    // dealing with non moveable modules
        const CalibData * src_calib_ptr = static_cast<const CalibData*>(manager.getCamParams(src_module_name));
		h = computeHomography(	ref_calib_ptr->K(),
								src_calib_ptr->K(),
								ref_calib_ptr->R(),
								src_calib_ptr->R(),
								ref_calib_ptr->translation(),
								src_calib_ptr->translation(),
								depth_in_mm);
	}

    // 	// transfer the point
    // printf("\nfinal homog:");
    //     printf("%f, %f, %f\n", h(0,0), h(0,1), h(0,2));
    //     printf("%f, %f, %f\n", h(1,0), h(1,1), h(1,2));
    //     printf("%f, %f, %f\n", h(2,0), h(2,1), h(2,2));

    // printf("\nroi_center = %f, %f\n", roi_center.x(), roi_center.y());
    return h * roi_center;
}

}
