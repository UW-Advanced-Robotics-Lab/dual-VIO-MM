/* [ An Ultra-light-weight Lie Algebra library based on Eigen ]
 * @Notation:
 *   The notation is based on Murrays, aka: (v,w) instead of (w,v) and uppoer triangle matrices
 * @Credits:
 *   - some inspiration from https://github.com/Le0nX/ModernRoboticsCpp/blob/master/src/modern_robotics.cpp
 *   - based on Murrays, Intro to Robotics Manipulation
 *   - based on Park, ModernRobotics
 * @author: 
 *   - Jack Xu @ projectbyjx@gmail.com
 * ----------------------------------------------------------------
*/
#include "Lie.h"
#include <sstream>

namespace Lie {
// ----------------------------------------------------------------
// : Axis Angle :
// ----------------------------------------------------------------
    Axis_Angle_t AxisAngle_From_R3(const R3& w){
        Axis_Angle_t wt;
        wt.theta = w.norm();
        if (NEAR_ZERO(wt.theta))
            wt.w = R3::Zero();
        else
            wt.w = w/wt.theta;
        return wt;
    }
    Twist_Angle_t TwistAngle_From_R6(const R6& xi){
        Twist_Angle_t vwt;
        vwt.theta = xi.norm();
        if (NEAR_ZERO(vwt.theta))
            vwt.vw = R6::Zero();
        else
            vwt.vw = xi/vwt.theta;
        return vwt;
    }
    R6 TwistAngle_To_R6(const Twist_Angle_t vwt){
        R6 xi = vwt.theta * vwt.vw;
        return xi;
    }
    R3 AxisAngle_To_R3(const Axis_Angle_t wt){
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
    void SO3xR3_from_SE3(SO3& R, R3& p, const SE3& T) {
        R = T.block<3, 3>(0, 0);
        p = T.block<3, 1>(0, 3);
    }
    
    void SE3_from_SO3xR3(SE3& T, const SO3& R, const R3& p) {
        T << R, p, 0, 0, 0, 1;
    }

    SE3 SE3_from_SO3xR3(const SO3& R, const R3& p) {
        SE3 T;
        T << R, p, 0, 0, 0, 1;
        return T;
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
 
    SE3 invert_SO3xR3(const SO3& R, const R3& p){
        SE3 m_ret;
        m_ret << R.transpose(), - p, 
                0, 0, 0, 1;
		return m_ret;
    }
    
    SE3 invert_SE3(const SE3& T){
        SO3 R; R3 p;
        Lie::SO3xR3_from_SE3(R, p, T);
        return Lie::invert_SO3xR3(R, p);
    }

    // SE3 prod_chain_forward(const SE3 mat[], const size_t N){
    //     SE3 m_ret = mat[0];
    //     for(size_t i = 1; i < N; i ++)
    //     {
    //         m_ret = m_ret * mat[i];
    //     }
    // }
    // SE3 prod_chain_reverse(const SE3 mat[], const size_t N){
    //     SE3 m_ret = mat[0];
    //     for(size_t i = 1; i < N; i ++)
    //     {
    //         m_ret = mat[i] * m_ret;
    //     }
    // }
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

    Mat6x6d ad(const R3& v, const R3& w)
    {
        so3 hat_w = Lie::HAT_R3_to_so3(w);
        so3 hat_v = Lie::HAT_R3_to_so3(v);
        Mat6x6d m_ret;
        m_ret << hat_w, hat_v, so3::Zero(), hat_w;
        return m_ret;
    }

    // Rodrigue exp/Exp Map --------------------------------
    /* Retract from Lie Algebra to Manifold
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
            SO3 A_mat = (SO3::Identity() - R) * hat_w + w * w.transpose() * t;
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
    * Log Map is used to lift from the manifold to the tangent space (local)
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


    // Projection Map --------------------------------
    SO3 project_to_SO3(const Mat3x3d& M)
    {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		SO3 R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0)
        {
            // In this case the result may be far from M; reverse sign of 3rd column
			R.col(2) *= -1;
        }
        return R;
    }

    SE3 project_to_SE3(const Mat4x4d& M)
    {
        SO3 R; R3 p; SE3 T;
        Lie::SO3xR3_from_SE3(R,p,M);
        Lie::SE3_from_SO3xR3(T,Lie::project_to_SO3(R),p);
        return T;
    }

    double distance_to_SO3(const Mat3x3d& M)
    {
        double dist = AT_NEAR_INFINITY;
        if (M.determinant() > NEAR_ZERO_TOL)
        {
            // Ideally it should be close to identity by definition:
            dist = (M.transpose() * M - SO3::Identity()).norm();
        }
        return dist;
    }

    double distance_to_SE3(const Mat4x4d& M)
    {
        double dist = AT_NEAR_INFINITY;
        SO3 R = M.block<3,3>(0,0);
        SE3 T;
        if (R.determinant() > NEAR_ZERO_TOL)
        {
            T << R.transpose() * R, R3::Zero(), M.row(3);
            T -= SE3::Identity();
            dist = T.norm();
        }
        return dist;
    }
	
	bool if_SO3(const Mat3x3d& M) {
		return NEAR_ZERO(Lie::distance_to_SO3(M));
	}

	bool TestIfSE3(const Eigen::Matrix4d& T) {
		return NEAR_ZERO(Lie::distance_to_SE3(T));
    }

    std::string to_string(const Eigen::MatrixXd& M)
    {
        std::stringstream ss;
        ss << M;
        return ss.str();
    }
    std::string to_string(const Eigen::Quaterniond& q)
    {
        std::stringstream ss;
        ss << "Quaternion (x,y,z,w): "  << q.x() << ", " << q.y() << ", " << q.z()  << ", " << q.w() << std::endl;
        return ss.str();
    }
} /* namespace Lie */

