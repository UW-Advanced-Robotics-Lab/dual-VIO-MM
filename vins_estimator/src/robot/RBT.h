#ifndef RBT_H
#define RBT_H

#include "Lie.h"

namespace RBT { 
// ----------------------------------------------------------------
// : Twist Coordinates :
// ----------------------------------------------------------------
    Lie::R6 twist_from_axis_point(const Lie::R3& w, const Lie::R3& q);
    Lie::R6 twist_from_axis_point_pitch(const Lie::R3& w, const Lie::R3& q, const Lie::R1 h);

// ----------------------------------------------------------------
// : Screw Motion :
// ----------------------------------------------------------------
    Lie::SE3 screw_motion_from_twist_angle(const Lie::Twist_Angle_t& vwt);
    Lie::SE3 screw_motion_from_pure_translation(const Lie::R3& v, const Lie::R1 t);

};


#endif // RBT_H