#ifndef OPEN_CHAIN_H
#define OPEN_CHAIN_H

#include "Lie.h"
#include "RBT.h"

namespace OpenChain { 
// ----------------------------------------------------------------
// : Twist Angle to Screw Motion :
// ----------------------------------------------------------------
   std::vector<Lie::SE3> compute_screw_motion(const std::vector<Lie::Twist_Angle_t>& xit_);

// ----------------------------------------------------------------
// : Inertia Matrix :
// ----------------------------------------------------------------
    Lie::Mat6x6d init_link_generalized_inertia_matrix(const Lie::R1 mass_i, const Lie::Mat3x3d& I_mc_i);
 
    /* init_link_generalized_inertia_matrix_from_CoM:
        % -* inertia at the center-of-mass, aligned with output frame axis:
        % -> translating inertia matrix from center-of-mass to the output frame:
    */
    Lie::Mat6x6d init_link_generalized_inertia_matrix_from_CoM(const Lie::R1 mass_i, const Lie::R3 q_mc_i, const Lie::Mat3x3d& I_mc_i);

// ----------------------------------------------------------------
// : Spatial :
// ----------------------------------------------------------------
    /* compute_spatial_FK
     * @exp_xi_theta_i: computed from OpenChain::compute_screw_motion
     * @g0_st: zero configuration of the tool frame: g_st(0)
     * @formula:
     *      g_st(t) = exp_xi_1(t_1) * exp_xi_2(t_2) * ... * exp_xi_n(t_n) * g_st(0)
    */
    Lie::SE3 compute_spatial_FK(const std::vector<Lie::SE3>& exp_xi_theta_, const Lie::SE3 & g0_st);
    Lie::SE3 compute_spatial_FK(const Lie::SE3 exp_xi_theta_[], const Lie::SE3 & g0_st, const int N_jnts);
    std::vector<Lie::SE3> compute_spatial_FK_for_all_joints(const std::vector<Lie::SE3>& exp_xi_theta_, const std::vector<Lie::SE3> g0_st_);
};


#endif // OPEN_CHAIN_H