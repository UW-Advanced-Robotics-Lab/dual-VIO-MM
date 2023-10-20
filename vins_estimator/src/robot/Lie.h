/*
* @Credits:
    - some inspiration from https://github.com/Le0nX/ModernRoboticsCpp/blob/master/src/modern_robotics.cpp
    - based on Murrays, Intro to Robotics Manipulation
    - based on Park, ModernRobotics
*/
#ifndef LIE_H
#define LIE_H

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#define NEAR_ZERO_TOL   (double)(1e-6)
#define NEAR_ZERO(x)    (std::abs(x) < NEAR_ZERO_TOL)
class Lie { 
public:
// ----------------------------------------------------------------
// : Definitions :
// ----------------------------------------------------------------
    typedef Eigen::Matrix<double, 4, 4> Mat4x4d;
    typedef Eigen::Matrix<double, 6, 6> Mat6x6d;
    typedef Eigen::Matrix<double, 3, 3> Mat3x3d;
    typedef Eigen::Matrix<double, 3, 1> Vec3d;
    typedef Eigen::Matrix<double, 6, 1> Vec6d;

    // shortcuts:
    typedef double R1;
    typedef Vec3d R3;
    typedef Vec6d R6;
    typedef Mat4x4d SE3;
    typedef Mat4x4d se3;
    typedef Mat3x3d SO3;
    typedef Mat3x3d so3;

// ----------------------------------------------------------------
// : Axis Angle :
// ----------------------------------------------------------------
    // data struct:
    typedef struct{
        Vec3d   w;
        double  theta;
    } Axis_Angle_t;
    typedef struct{
        Vec6d   vw;
        double  theta;
    } Twist_Angle_t;

    Axis_Angle_t AxisAngle_From_R3(const R3& w){
        Axis_Angle_t wt;
        wt.theta = w.norm();
        if (NEAR_ZERO(wt.theta))
            wt.w = R3::Zero();
        else
            wt.w = w/wt.theta;
        return wt;
    }
    Twist_Angle_t TwistAngle_From_R6(const R3& xi){
        Twist_Angle_t vwt;
        vwt.theta = xi.norm();
        if (NEAR_ZERO(vwt.theta))
            vwt.vw = R6::Zero();
        else
            vwt.vw = xi/vwt.theta;
        return vwt;
    }
    inline R6 TwistAngle_To_R6(const Twist_Angle_t vwt){
        R6 xi = vwt.theta * vwt.vw;
        return xi;
    }
    inline R3 AxisAngle_To_R3(const Axis_Angle_t wt){
        R3 w = wt.theta * wt.w;
        return w;
    }
    
// ----------------------------------------------------------------
// : Tangent Space :
// ----------------------------------------------------------------
    // Lie Algebra <-> vector 
    Vec6d VEE_se3_to_R6(const Mat4x4d& T) {
		Vec6d m_ret(6);
		m_ret << T(0, 3), T(1, 3), T(2, 3), T(2, 1), T(0, 2), T(1, 0);
		return m_ret;
	}
    Vec3d VEE_so3_to_R3(const Mat3x3d& T) {
		Vec3d m_ret(3);
		m_ret << T(0, 3), T(1, 3), T(2, 3);
		return m_ret;
	}
    Mat3x3d HAT_R3_to_so3(const Vec3d& w) {
		Mat3x3d m_ret;
		m_ret << 0, -w(2), w(1),
                w(2), 0, -w(0),
                -w(1), w(0), 0;
		return m_ret;
	}
    Mat4x4d HAT_R3_to_so3(const Vec6d& vw) {
		Mat4x4d m_ret;
        Vec3d v = vw.head(3);
        Vec3d w = vw.tail<3>();
		m_ret << Lie::HAT_R3_to_so3(w), v, 
                 0, 0, 0, 0;
		return m_ret;
	}
// ----------------------------------------------------------------
// : Manifold :
// ----------------------------------------------------------------
    inline void SO3xR3_from_SE3(SO3& R, R3& p, const SE3& T) {
        R = T.block<3, 3>(0, 0);
        p = T.block<3, 1>(0, 3);
    }
    inline void SE3_from_SO3xR3(SE3& T, const SO3& R, const R3& p) {
        T << R, p, 0, 0, 0, 1;
    }

    SE3 inverse_SO3xR3(const SO3& R, const R3& p){
        SE3 m_ret;
        const SO3 RT = R.transpose();
        m_ret << RT, - RT * p, 
                0, 0, 0, 1;
		return m_ret;
    }

    SE3 inverse_SE3(const SE3& T){
        SO3 R; R3 p;
        Lie::SO3xR3_from_SE3(R, p, T);
        return Lie::inverse_SO3xR3(R, p);
    }

    // Adjoint Transformation --------------------------------
    /* Ad: A linear Transformations for a particular T\in SE3, that can be applied to the lie algebra g\in se3-R6
    * For each group element g in G with a linear transformation Ad(g) on the lie algebra g
    */
    Mat6x6d Ad(const SO3& R, const R3& p)
    {
        Mat6x6d m_ret;
        m_ret << R, Lie::HAT_R3_to_so3(p),
                 Mat3x3d::Zero(), R;
        return m_ret;
    }

    Mat6x6d Ad(const SE3& T){
        SO3 R; R3 p;
        Lie::SO3xR3_from_SE3(R, p, T);
        return Lie::Ad(R, p);
    }

    Mat6x6d invAd(const SO3& R, const R3& p){
        return Lie::Ad(R.transpose(), -p);
    }
    Mat6x6d invAd(const SE3& T){
        SO3 R; R3 p;
        Lie::SO3xR3_from_SE3(R, p, T);
        return Lie::invAd(R, p);
    }

    // Rodrigue exp/Exp Map --------------------------------
    /* Lift from Lie Algebra to Manifold
    * exp: so3 --exp--> SO3
    * Exp: R3/R3xR  --Exp--> SO3
    * Exp: R6/R6xR  --Exp--> SE3
    */
    SO3 exp_SO3_from_so3(const so3& hat_wt)
    {
        SO3 exp = SO3::Identity();
        const R3 w_R3 = Lie::VEE_so3_to_R3(hat_wt);
        const R1 t = w_R3.norm();
        if (NEAR_ZERO(t) == false)
        {
            const so3 hat_w = hat_wt / t;
            exp = exp + std::sin(t) * hat_w + ((1 - std::cos(t)) * (hat_w * hat_w));
        } 
        return exp;
    }
    SO3 Exp_SO3_from_AxisAngle(const R3& w, const R1 theta)
    {
        SO3 exp = SO3::Identity();
        so3 hat_w = Lie::HAT_R3_to_so3(w);
        exp = exp + std::sin(theta) * hat_w + ((1 - std::cos(theta)) * (hat_w * hat_w));
        return exp;
    }
    SO3 Exp_SO3_from_AxisAngle(const Axis_Angle_t& wt)
    {
        return Exp_SO3_from_AxisAngle(wt.w, wt.theta);
    }
    SO3 Exp_SO3_from_R3(const R3& w)
    {
        Axis_Angle_t wt = AxisAngle_From_R3(w);
        SO3 exp = SO3::Identity();
        if (NEAR_ZERO(wt.theta) == false)
        {
            exp = Lie::Exp_SO3_from_AxisAngle(wt);
        }
        return exp;
    }
    SE3 Exp_SE3_from_TwistAngle(const Twist_Angle_t& vwt)
    {
        SE3 exp = SE3::Identity();
        
        const Vec3d v = vwt.vw.head(3);
        const Vec3d w = vwt.vw.tail<3>();
        const double t = vwt.theta;
        
        if (NEAR_ZERO(w.norm()))
        {
            // That means it is a linear representation 
            // % (Murray 2.32)
            exp.block<3,1>(0,3) = v * t;
        }
        else
        {
            // % [Murray 2.38]
            SO3 R = Lie::Exp_SO3_from_AxisAngle(w, t);
            so3 hat_w = Lie::HAT_R3_to_so3(w);
            // % --> A_mat(.) non-singular for all t \in (0,2*pi)
            SE3 A_mat = (SO3::Identity() - R) * hat_w + w * w.transpose() * t;
            // fill:
            exp.block<3,3>(0,0) = R;
            exp.block<3,1>(0,3) = A_mat * v;
        }
        return exp;
    }
    SE3 Exp_SE3_from_R6(const R6& vw)
    {
        Twist_Angle_t vwt = TwistAngle_From_R6(vw);
        SE3 exp = SE3::Identity();
        if (NEAR_ZERO(vwt.theta) == false)
        {
            exp = Lie::Exp_SE3_from_TwistAngle(vwt);
        }
        return exp;
    }
    // Log Map --------------------------------
    /* Lift->solve->retract
    * Log Map is used to retract from the manifold to the tangent space
    * NOTE: The below formulation is not a smooth map of R, for a smooth map, 
    *   please consider cubic fitting formulation in Geometric Robotics Book
    */
    so3 log_so3_from_SO3(const SO3& R)
    {
        // Modern Robotics:
        double acosinput = 0.5 * (R.trace() - 1); // 1/2 = 0.5
        so3 hat_w = so3::Zero();
        if (acosinput >= 1)
        {
            // Do nothing
        }
        else if (acosinput <= -1)
        {
            R3 w;
            if (! NEAR_ZERO(1 + R(2,2)))
            {
                w = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
            }
			else if (! NEAR_ZERO(1 + R(1, 1)))
            {
				w = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
            }
			else
            {
				w = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
            }
            w *= M_PI;
			hat_w = Lie::HAT_R3_to_so3(w);
        }
        else
        {
            double theta = std::acos(acosinput);
            hat_w = 0.5 * theta / std::sin(theta) * (R - R.transpose());
        }
        return hat_w;
    }
    R3 Log_R3_from_SO3(const SO3& R)
    {
        // Modern Robotics:
        double acosinput = 0.5 * (R.trace() - 1); // 1/2 = 0.5
        R3 w = R3::Zero();
        if (acosinput >= 1)
        {
            // Do nothing
        }
        else if (acosinput <= -1)
        {
            if (! NEAR_ZERO(1 + R(2,2)))
            {
                w = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
            }
			else if (! NEAR_ZERO(1 + R(1, 1)))
            {
				w = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
            }
			else
            {
				w = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
            }
            w *= M_PI;
        }
        else
        {
            double theta = std::acos(acosinput);
            SO3 dR = (R - R.transpose());
            w = Lie::VEE_so3_to_R3(dR);
            w *= 0.5 * theta / std::sin(theta);
        }
        return w;
    }
    se3 log_se3_from_SE3(const SE3& T)
    {
        se3 hat_xi = se3::Zero();
        SO3 R; R3 p;
        Lie::SO3xR3_from_SE3(R, p, T);
        const so3 hat_w = Lie::log_so3_from_SO3(R);
        if (NEAR_ZERO(hat_w.norm()))
        {
            hat_xi.block<3,1>(0,3) = p;
        }
        else
        {
            // compute:
            const double t = 0.5 * std::acos(R.trace() - 1);
            const double inv_t = 1/t;
            const so3 mat_a = so3::Identity() - 0.5 * hat_w + (inv_t - 0.5/std::tan(0.5 * t)) * hat_w * hat_w * inv_t;
            // fill-in:
            hat_xi.block<3,3>(0,0) = hat_w;
            hat_xi.block<3,1>(0,3) = mat_a * p;

        }
        return hat_xi;
    }
};


#endif // LIE_H