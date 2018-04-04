#ifndef __ROITRANSFER_H__
#define __ROITRANSFER_H__

#ifdef __cplusplus

#include <memory>
#include "calibclasses.h"
#include "mirrorsystems.h"

namespace ltaf {

	/// interface for roi transfer functions
    Vec2f transferROI( ModuleName ref_module_name,          // Reference module name
                       ModuleName src_module_name,          // Source module name
                       const CalibDataManager& manager,	    // Calib data manager
                       const Vec2f&   roi_center,           // Reference roi center coord (in pixels), assume full res reference frame
                       float depth_in_mm,                   // Depth in mm. If -1, then we use infinity
                       std::uint16_t mirror_hall_code );    // Mirror hall code. If its -ve, we ignore the hall code.

	float computeMirrorAngleGivenHallCode(std::uint16_t hall_code, const MirrorActuatorMapping & mirror_act_mapping);

} // end namespace ltaf

#endif
#endif
