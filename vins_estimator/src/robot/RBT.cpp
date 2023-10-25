#include "RBT.h"

namespace RBT { 
// ----------------------------------------------------------------
// : Twist Coordinates :
// ----------------------------------------------------------------
    Lie::R6 twist_from_axis_point(const Lie::R3& w, const Lie::R3& q)
    {
        Lie::R6 xi;
        //- rotation about a line
        Lie::R3 v = - Lie::HAT_R3_to_so3(w) * q; //% (Murray 2.26)
        xi << v, w;
        return xi;
    }

    Lie::R6 twist_from_axis_point_pitch(const Lie::R3& w, const Lie::R3& q, const Lie::R1 h)
    {
        Lie::R6 xi;
        //- rotation about a line + translation along the line
        Lie::R3 v = - Lie::HAT_R3_to_so3(w) * q + h * w; //% (Murray 2.26)
        xi << v, w;
        return xi;
    }
// ----------------------------------------------------------------
// : Screw Motion :
// ----------------------------------------------------------------
    Lie::SE3 screw_motion_from_twist_angle(const Lie::Twist_Angle_t& vwt)
    {
        // - compute screw motion, pure rotation
        return Lie::Exp_SE3_from_TwistAngle(vwt);
    }
    Lie::SE3 screw_motion_from_pure_translation(const Lie::R3& v, const Lie::R1 t)
    {
        Lie::SE3 mat = Lie::SE3::Identity();
        mat.block<3,1>(0,3) = v * t;
        return mat;
    }
};
