/*******************************************************
 * Based on HKST 
 * Formulation is equivalent to SE3 Absolute Pose Factor
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include "../robot/Lie.h"
#include <ceres/ceres.h>

#define USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_COVARIANCE (0U) // Minimal Jacob - ref: MCVIO - PoseError.cpp 
#define USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_COMPACT        (1U) // Left-Conjugate - ref: MCVIO - SE3AbsolutatePoseFactor 
#define USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR             (0U) // Right-Inverse  - ref: GVINS-2021 - PoseAnchorFactor

#if (USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_COVARIANCE)
class ARMFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    /* Params */
    Lie::R3 P_arm; 
    Lie::Qd Q_arm;
    Eigen::Matrix<double, 6, 6> sqrt_info; // TODO: add non-uniform scale

    ARMFactor() = delete;
    ARMFactor(Lie::SO3 _R, Lie::R3 _p, double sqrt_info_scale = 120)
    {
        P_arm = _p;
        Q_arm = Lie::Qd(_R);
    	sqrt_info = sqrt_info_scale * Eigen::Matrix<double, 6, 6>::Identity();
    }
    
    ARMFactor(Lie::SO3 _R, Lie::R3 _p, double translation_variance = 0.01, double rotation_variance = 0.01)
    {
        P_arm = _p;
        Q_arm = Lie::Qd(_R);
        sqrt_info.setZero();
        // cov = info.inverse();
    	sqrt_info.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0 / translation_variance;
        sqrt_info.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0 / rotation_variance;
        // perform the Cholesky decomposition on order to obtain the correct error weighting
        Eigen::LLT<information_t> lltOfInformation(sqrt_info);
        sqrt_info = lltOfInformation.matrixL().transpose();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    { // TODO: we should check if the formulation is right
    	Lie::R3 P(parameters[0][0], parameters[0][1], parameters[0][2]);
    	Lie::Qd Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        // P - P_arm:
        residual.head<3>() = P - P_arm;
        // Q_arm^-1 * Q: (error quaternion)
        // (OR) NOTE: left multiply by Q_arm^-1:
        residual.tail<3>() = 2.0 * (Q_arm.conjugate() * Q).vec(); // scalar * 1vec
        residual.applyOnTheLeft(sqrt_info);

        if (jacobians && jacobians[0])
        {
            // TODO: verify the jacobian formulation
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.topLeftCorner<3, 3>().setIdentity();
            jacobian_pose.block<3, 3>(3, 3) = (Utility::Qleft(Q_arm.conjugate() * Q)).bottomRightCorner<3, 3>();
            // jacobian_pose = 2.0 * sqrt_info * jacobian_pose;
            jacobian_pose.applyOnTheLeft(2.0 * sqrt_info);
        }

        return true;
    }
};

#elif (USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_COMPACT)
class ARMFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    /* Params */
    Lie::R3 P_arm; 
    Lie::Qd Q_arm;
    Eigen::Matrix<double, 6, 6> sqrt_info; // TODO: add non-uniform scale

    ARMFactor() = delete;
    ARMFactor(Lie::SO3 _R, Lie::R3 _p, double sqrt_info_scale = 120)
    {
        P_arm = _p;
        Q_arm = Lie::Qd(_R);
    	sqrt_info = sqrt_info_scale * Eigen::Matrix<double, 6, 6>::Identity();
    }
    
    ARMFactor(Lie::SO3 _R, Lie::R3 _p, double W_x, double W_y, double W_z, double W_rot)
    {
        P_arm = _p;
        Q_arm = Lie::Qd(_R);
        Eigen::DiagonalMatrix<double, 6> sqrt_mat;
        sqrt_mat.diagonal() << W_x,W_y,W_z,W_rot,W_rot,W_rot;
    	sqrt_info = sqrt_mat * Eigen::Matrix<double, 6, 6>::Identity();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    { // TODO: we should check if the formulation is right
    	Lie::R3 P(parameters[0][0], parameters[0][1], parameters[0][2]);
    	Lie::Qd Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        // P - P_arm:
        residual.head<3>() = P - P_arm;
        // Q_arm^-1 * Q: (error quaternion)
    #if (USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR)
        residual.tail<3>() = 2.0 * (Q * Q_arm.inverse()).vec();
        residual = sqrt_info * residual;
    #else
        // (OR) NOTE: left multiply by Q_arm^-1:
        residual.tail<3>() = 2.0 * (Q_arm.conjugate() * Q).vec(); // scalar * 1vec
        residual.applyOnTheLeft(sqrt_info);
    #endif //(USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR)

        if (jacobians && jacobians[0])
        {
            // TODO: verify the jacobian formulation
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.topLeftCorner<3, 3>().setIdentity();
    #if (USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR)
            Eigen::Quaterniond Q_arm_inv = Q_arm.inverse();
            Eigen::Matrix3d J_q;
            J_q <<   Q_arm_inv.w(),  Q_arm_inv.z(), -Q_arm_inv.y(),
                    -Q_arm_inv.z(),  Q_arm_inv.w(),  Q_arm_inv.x(),
                     Q_arm_inv.y(), -Q_arm_inv.x(),  Q_arm_inv.w();
            jacobian_pose.block<3, 3>(3, 3) = J_q;
            jacobian_pose = 2.0 * sqrt_info * jacobian_pose;
    #else
            
            jacobian_pose.block<3, 3>(3, 3) = (Utility::Qleft(Q_arm.conjugate() * Q)).bottomRightCorner<3, 3>();
            jacobian_pose.applyOnTheLeft(2.0 * sqrt_info);
    #endif //(USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR)
        }

        return true;
    }
};

#elif (USE_ARM_FACTOR_SE3_ABSOLUTE_POSE_ANCHOR)
class ARMFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    /* Params */
    Lie::R3 P_arm; 
    Lie::Qd Q_arm;
    Eigen::Matrix<double, 6, 6> sqrt_info; // TODO: add non-uniform scale

    ARMFactor() = delete;
    ARMFactor(Lie::SO3 _R, Lie::R3 _p, double sqrt_info_scale = 120)
    {
        P_arm = _p;
        Q_arm = Lie::Qd(_R);
    	sqrt_info = sqrt_info_scale * Eigen::Matrix<double, 6, 6>::Identity();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    { // TODO: we should check if the formulation is right
    	Lie::R3 P(parameters[0][0], parameters[0][1], parameters[0][2]);
    	Lie::Qd Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        // P - P_arm:
        residual.head<3>() = P - P_arm;
        // Q_arm^-1 * Q: (error quaternion)
        residual.tail<3>() = 2.0 * (Q * Q_arm.inverse()).vec();
        residual = sqrt_info * residual;

        if (jacobians && jacobians[0])
        {
            // TODO: verify the jacobian formulation
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.topLeftCorner<3, 3>().setIdentity();

            Eigen::Quaterniond Q_arm_inv = Q_arm.inverse();
            Eigen::Matrix3d J_q;
            J_q <<   Q_arm_inv.w(),  Q_arm_inv.z(), -Q_arm_inv.y(),
                    -Q_arm_inv.z(),  Q_arm_inv.w(),  Q_arm_inv.x(),
                     Q_arm_inv.y(), -Q_arm_inv.x(),  Q_arm_inv.w();
            jacobian_pose.block<3, 3>(3, 3) = J_q;
            jacobian_pose = 2.0 * sqrt_info * jacobian_pose;
        }

        return true;
    }
};
#else // (GVINS-2021) modified
class ARMFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    /* Params */
    Eigen::Matrix<double, 7, 1> arm_pose;
    constexpr static double sqrt_info = 120; // TODO: add covariance matrix from kinematic config?

    ARMFactor() = delete;
    ARMFactor(Lie::SO3 _R, Lie::R3 _p)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(_R);
        arm_pose(0) = _p.x();
        arm_pose(1) = _p.y();
        arm_pose(2) = _p.z();
        arm_pose(3) = q.x();
        arm_pose(4) = q.y();
        arm_pose(5) = q.z();
        arm_pose(6) = q.w();
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    { // TODO: we should check if the formulation is right
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> pose(parameters[0]);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> res(residuals);
        res.head<3>() = pose.head<3>() - arm_pose.head<3>();
        const Eigen::Quaterniond curr_q(pose.tail<4>());
        const Eigen::Quaterniond anchor_q(arm_pose.tail<4>());
        res.tail<3>() = 2.0 * (curr_q*anchor_q.inverse()).vec();
        res *= sqrt_info;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.topLeftCorner<3, 3>().setIdentity();

            Eigen::Quaterniond anchor_q_inv = anchor_q.inverse();
            Eigen::Matrix3d J_q;
            J_q <<   anchor_q_inv.w(),  anchor_q_inv.z(), -anchor_q_inv.y(),
                    -anchor_q_inv.z(),  anchor_q_inv.w(),  anchor_q_inv.x(),
                     anchor_q_inv.y(), -anchor_q_inv.x(),  anchor_q_inv.w();
            J.block<3, 3>(3, 3) = J_q;
            J *= 2.0*sqrt_info;
        }

        return true;
    }
};
#endif