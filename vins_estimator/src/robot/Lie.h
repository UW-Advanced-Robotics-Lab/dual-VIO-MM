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
#ifndef LIE_H
#define LIE_H

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#define NEAR_ZERO_TOL       (double)(1e-6)
#define NEAR_INFINITY_TOL   (double)(1e9)
#define AT_NEAR_ZERO           (double)(NEAR_ZERO_TOL * 0.1)
#define AT_NEAR_INFINITY       (double)(NEAR_INFINITY_TOL * 10)

#define NEAR_ZERO(x)        (std::abs(x) < NEAR_ZERO_TOL)

namespace Lie {
// ----------------------------------------------------------------
// : Definitions :
// ----------------------------------------------------------------
    typedef Eigen::Matrix<double, 4, 4> Mat4x4d;
    typedef Eigen::Matrix<double, 6, 6> Mat6x6d;
    typedef Eigen::Matrix<double, 3, 3> Mat3x3d;
    typedef Eigen::Matrix<double, 3, 1> Vec3d;
    typedef Eigen::Matrix<double, 4, 1> Vec4d;
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

    Axis_Angle_t AxisAngle_From_R3(const R3& w);
    Twist_Angle_t TwistAngle_From_R6(const R6& xi);
    R6 TwistAngle_To_R6(const Twist_Angle_t vwt);
    R3 AxisAngle_To_R3(const Axis_Angle_t wt);
    
// ----------------------------------------------------------------
// : Tangent Space :
// ----------------------------------------------------------------
    // Lie Algebra <-> vector 
    Vec6d VEE_se3_to_R6(const Mat4x4d& T);
    Vec3d VEE_so3_to_R3(const Mat3x3d& T);
    Mat3x3d HAT_R3_to_so3(const Vec3d& w);
    Mat4x4d HAT_R3_to_so3(const Vec6d& vw);
// ----------------------------------------------------------------
// : Manifold :
// ----------------------------------------------------------------
    void SO3xR3_from_SE3(SO3& R, R3& p, const SE3& T);
    void SE3_from_SO3xR3(SE3& T, const SO3& R, const R3& p);
    SE3 SE3_from_SO3xR3(const SO3& R, const R3& p);
    SE3 inverse_SO3xR3(const SO3& R, const R3& p);
    SE3 inverse_SE3(const SE3& T);
    // SE3 prod_chain_forward(const SE3 mat[], const size_t N);
    // SE3 prod_chain_reverse(const SE3 mat[], const size_t N);
    SE3 invert_SO3xR3(const SO3& R, const R3& p);
    SE3 invert_SE3(const SE3& T);
    
// Adjoint Transformation --------------------------------
    /* Ad: A linear Transformations for a particular T\in SE3, that can be applied to the lie algebra g\in se3-R6
    * For each group element g in G with a linear transformation Ad(g) on the lie algebra g
    */
    Mat6x6d Ad(const SO3& R, const R3& p);
    Mat6x6d Ad(const SE3& T);
    Mat6x6d invAd(const SO3& R, const R3& p);
    Mat6x6d invAd(const SE3& T);
    Mat6x6d ad(const R3& v, const R3& w);

// Rodrigue exp/Exp Map --------------------------------
    /* Retract from Lie Algebra to Manifold
    * exp: so3 --exp--> SO3
    * Exp: R3/R3xR  --Exp--> SO3
    * Exp: R6/R6xR  --Exp--> SE3
    */
    SO3 exp_SO3_from_so3(const so3& hat_wt);
    SO3 Exp_SO3_from_AxisAngle(const R3& w, const R1 theta);
    SO3 Exp_SO3_from_AxisAngle(const Axis_Angle_t& wt);
    SO3 Exp_SO3_from_R3(const R3& w);
    SE3 Exp_SE3_from_TwistAngle(const Twist_Angle_t& vwt);
    SE3 Exp_SE3_from_R6(const R6& vw);

// Log Map --------------------------------
    /* Lift->solve->retract
    * Log Map is used to lift from the manifold to the tangent space (local)
    * NOTE: The below formulation is not a smooth map of R, for a smooth map, 
    *   please consider cubic fitting formulation in Geometric Robotics Book
    */
    so3 log_so3_from_SO3(const SO3& R);
    R3 Log_R3_from_SO3(const SO3& R);
    se3 log_se3_from_SE3(const SE3& T);
 
// Projection Map --------------------------------
    SO3 project_to_SO3(const Mat3x3d& M);
    SE3 project_to_SE3(const Mat4x4d& M);
    double distance_to_SO3(const Mat3x3d& M);
    double distance_to_SE3(const Mat4x4d& M);
	bool if_SO3(const Mat3x3d& M);
	bool TestIfSE3(const Eigen::Matrix4d& T);

// Debug Utils --------------------------------
    std::string to_string(const Eigen::MatrixXd& M);
    std::string to_string(const Eigen::Quaterniond& q);
} /* namespace Lie */


#endif  // LIE_H