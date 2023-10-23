#ifndef OPEN_CHAIN_H
#define OPEN_CHAIN_H

#include "Lie.h"
#include "RBT.h"

// using namespace Lie;
// using namespace RBT;
namespace OpenChain { 
// ----------------------------------------------------------------
// : Twist Angle to Screw Motion :
// ----------------------------------------------------------------
   std::vector<Lie::SE3> compute_screw_motion(const std::vector<Lie::Twist_Angle_t>& xit_)
   {
        const int N_jnts = xit_.size();
        std::vector<Lie::SE3> g_st;
        for (std::size_t i = 0; i < N_jnts; i++)
        {
            g_st.push_back(RBT::screw_motion_from_twist_angle(xit_[i]));
        }
        return g_st;
   }
// ----------------------------------------------------------------
// : Inertia Matrix :
// ----------------------------------------------------------------
    Lie::Mat6x6d init_link_generalized_inertia_matrix(const Lie::R1 mass_i, const Lie::Mat3x3d& I_mc_i)
    {
        Lie::Mat6x6d M_i = Lie::Mat6x6d::Zero();
        //-* inertia at the output frame axis:
        M_i.block<3,3>(0,0) = mass_i * Lie::Mat3x3d::Identity();
        M_i.block<3,3>(3,3) = I_mc_i;
        return M_i;
    }
    
    /* init_link_generalized_inertia_matrix_from_CoM:
        % -* inertia at the center-of-mass, aligned with output frame axis:
        % -> translating inertia matrix from center-of-mass to the output frame:
    */
    Lie::Mat6x6d init_link_generalized_inertia_matrix_from_CoM(const Lie::R1 mass_i, const Lie::R3 q_mc_i, const Lie::Mat3x3d& I_mc_i)
    {
        Lie::Mat6x6d M_i;
        Lie::so3 hat_q_mc_i = Lie::HAT_R3_to_so3(q_mc_i);
        //[pg 288]
        M_i.block<3,3>(0,0) = mass_i * Lie::Mat3x3d::Identity();
        M_i.block<3,3>(0,3) = - mass_i * hat_q_mc_i;
        M_i.block<3,3>(3,0) = mass_i * hat_q_mc_i;
        M_i.block<3,3>(3,3) = I_mc_i - mass_i * (hat_q_mc_i * hat_q_mc_i);
        return M_i;
    }
// ----------------------------------------------------------------
// : Spatial :
// ----------------------------------------------------------------
    /* compute_spatial_FK
     * @exp_xi_theta_i: computed from OpenChain::compute_screw_motion
     * @g0_st: zero configuration of the tool frame: g_st(0)
     * @formula:
     *      g_st(t) = exp_xi_1(t_1) * exp_xi_2(t_2) * ... * exp_xi_n(t_n) * g_st(0)
    */
    Lie::SE3 compute_spatial_FK(const std::vector<Lie::SE3>& exp_xi_theta_, const Lie::SE3 g0_st)
    {
        const int N_jnts = exp_xi_theta_.size();
        Lie::SE3 s_G_st = Lie::SE3::Zero();
        for (std::size_t i = 0; i < N_jnts; i++)
        {
            s_G_st = s_G_st * exp_xi_theta_[i];
        }
        s_G_st = s_G_st * g0_st;
        return s_G_st;
    }

    std::vector<Lie::SE3> compute_spatial_FK_for_all_joints(const std::vector<Lie::SE3>& exp_xi_theta_, const std::vector<Lie::SE3> g0_st_)
    {
        const int N_jnts = exp_xi_theta_.size();
        std::vector<Lie::SE3> s_G_st_; 
        Lie::SE3 s_G_st = Lie::SE3::Zero();
        for (std::size_t i = 0; i < N_jnts; i++)
        {
            s_G_st = s_G_st * exp_xi_theta_[i];
            s_G_st_.push_back(s_G_st * g0_st_[i]);
        }
        return s_G_st_;
    }
    
};


#endif // OPEN_CHAIN_H