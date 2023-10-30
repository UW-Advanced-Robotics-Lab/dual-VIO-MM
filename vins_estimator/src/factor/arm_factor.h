/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
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
    {
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

