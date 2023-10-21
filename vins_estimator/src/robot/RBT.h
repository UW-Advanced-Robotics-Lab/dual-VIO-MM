#ifndef RBT_H
#define RBT_H

#include "Lie.h"

using namespace Lie;
namespace RBT { 
// ----------------------------------------------------------------
// : Twist Coordinates :
// ----------------------------------------------------------------
    R6 twist_from_axis_point(const R3& w, const R3& q)
    {
        R6 xi;
        //- rotation about a line
        R3 v = - HAT_R3_to_so3(w) * q; //% (Murray 2.26)
        xi << v, w;
        return xi;
    }

    R6 twist_from_axis_point_pitch(const R3& w, const R3& q, const R1 h)
    {
        R6 xi;
        //- rotation about a line + translation along the line
        R3 v = - HAT_R3_to_so3(w) * q + h * w; //% (Murray 2.26)
        xi << v, w;
        return xi;
    }
// ----------------------------------------------------------------
// : Screw Motion :
// ----------------------------------------------------------------
    SE3 screw_motion_from_twist_angle(const Twist_Angle_t& vwt)
    {
        // - compute screw motion, pure rotation
        return Lie::Exp_SE3_from_TwistAngle(vwt);
    }
    SE3 screw_motion_from_pure_translation(const R3& v, const R1 t)
    {
        SE3 mat = SE3::Identity();
        mat.block<3,1>(0,3) = v * t;
        return mat;
    }
};


#endif // RBT_H