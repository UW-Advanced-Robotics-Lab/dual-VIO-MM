/*******************************************************
 * Based on HKST 
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
        // Q_arm^-1 * Q:
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
            J_q <<  Q_arm_inv.w(),  Q_arm_inv.z(), -Q_arm_inv.y(),
                    -Q_arm_inv.z(),  Q_arm_inv.w(),  Q_arm_inv.x(),
                     Q_arm_inv.y(), -Q_arm_inv.x(),  Q_arm_inv.w();
            jacobian_pose.block<3, 3>(3, 3) = J_q;
            jacobian_pose = 2.0 * sqrt_info * jacobian_pose;
        }

        return true;
    }
};

