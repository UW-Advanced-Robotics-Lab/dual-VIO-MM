#include "../estimator/parameters.h"
#include "ArmModel.h"


typedef struct{
    std::string     name;
    // double          mass;
    // Lie::R3         mass_ceneer;
    // Lie::Mat3x3d    MoI;
    Lie::R3         tip_frame_Tq;// point
    Lie::R3         tip_frame_Tw;// axis
    Lie::R1         tip_frame_Tt;// angle
    double          tip_frame_lb;
    double          tip_frame_ub;
} link_t;

const link_t ARM_LINKS[ARM_NUM_LINKS] = {
    { // F0-->F1
        .name = "base_B3350",
        .tip_frame_Tq = Lie::R3(0,0,ARM_MODEL_CONFIG_L_SHOULDER),
        .tip_frame_Tw = Lie::R3(0,0,1),
        .tip_frame_Tt = 0,
        .tip_frame_lb = -2.6,
        .tip_frame_ub = 2.6,
    },
    { // F1-->F2
        .name = "shoulder_B3351",
        .tip_frame_Tq = Lie::R3(0,0,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = -M_PI_2, // -pi/2
        .tip_frame_lb = -2.0,
        .tip_frame_ub = 2.0,
    },
    { // F2-->F3
        .name = "arm_base_B3352",
        .tip_frame_Tq = Lie::R3(0,0,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = M_PI_2, // pi/2
        .tip_frame_lb = -2.8,
        .tip_frame_ub = 2.8,
    },
    { // F3-->F4
        .name = "arm_B3353",
        .tip_frame_Tq = Lie::R3(ARM_MODEL_CONFIG_W_ELBOW_JNT,0,ARM_MODEL_CONFIG_L_ARM),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = -M_PI_2, // -pi/2
        .tip_frame_lb = -0.9,
        .tip_frame_ub = 3.1,
    },
    { // F4-->F5
        .name = "elbow_+_wrist_base_B3355",
        .tip_frame_Tq = Lie::R3(-ARM_MODEL_CONFIG_W_ELBOW_JNT,-ARM_MODEL_CONFIG_L_ARM,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = M_PI_2, // pi/2
        .tip_frame_lb = -4.8,
        .tip_frame_ub = 1.3,
    },
    { // F5-->F6
        .name = "wrist_yaw_B3345",
        .tip_frame_Tq = Lie::R3(0,0,0),
        .tip_frame_Tw = Lie::R3(0,0,1),
        .tip_frame_Tt = -M_PI_2, // -pi/2
        .tip_frame_lb = -1.6,
        .tip_frame_ub = 1.6,
    },
    { // F6-->F7
        .name = "wrist_pitch",
        .tip_frame_Tq = Lie::R3(0,0,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = M_PI_2, // pi/2
        .tip_frame_lb = -2.2,
        .tip_frame_ub = 2.2,
    },
    { // F7-->F8
        .name = "wrist_palm",
        .tip_frame_Tq = Lie::R3(0,0,0.06),
        .tip_frame_Tw = Lie::R3(0,0,1),
        .tip_frame_Tt = 0, //
        .tip_frame_lb = 0,
        .tip_frame_ub = 0,
    },
};


ArmModel::ArmModel()
{
    // model initialization:
    m_arm.guard.lock();
    this->_model_initialization_unsafe();
    m_arm.guard.unlock();
}

ArmModel::~ArmModel()
{
    
}

const Lie::SE3 T0_summit = Lie::SE3::Identity();
const Lie::R3 summit_dP_wam = Lie::R3(0.14,0,0.405);
const Lie::R3 summit_dP_cam_base = Lie::R3(0.346,0,0.397);
const Lie::R3 wam_dP_cam_ee = Lie::R3(0,-0.11,0.018);

void ArmModel::_model_initialization_unsafe()
{
    // - summit pose:
    const Lie::SE3 spatial_G_summit = Lie::SE3::Identity() * T0_summit;
    // - summit ->> WAM base frame:
    const Lie::Axis_Angle_t wt_summit = Lie::Axis_Angle_t{.w=Lie::R3(0,0,1),.theta=1};
    const Lie::SO3 R_summit = Lie::Exp_SO3_from_AxisAngle(wt_summit);
    Lie::SE3 summit_G_wam = Lie::SE3_from_SO3xR3(R_summit, summit_dP_wam);
    // - define base spatial frame for WAM base frame:
    Lie::SE3 spatial_G_wam = spatial_G_summit * summit_G_wam;
    // ---- [ i-->i+1: i's Links with i's output frame]:
    for (size_t i = 0; i < ARM_NUM_LINKS; i++) 
    {
        PRINT_WARN("> Indexing Links @ %d-->%d", i, i+1)
        // 1. obtain relative params for current link (i)-->(i+1)
        const link_t * link = & ARM_LINKS[i];
        Lie::R3 q_i = link->tip_frame_Tq;
        Lie::R3 w_i = link->tip_frame_Tw;
        Lie::R1 t_i = link->tip_frame_Tt;
        // 2. output joint properties:
        // - joint limit:
        m_arm.jLimit_lb[i] = link->tip_frame_lb;
        m_arm.jLimit_ub[i] = link->tip_frame_ub;
        // - joint flag:
        if (NEAR_ZERO(m_arm.jLimit_lb[i] - m_arm.jLimit_ub[i]) == false)
            m_arm.jActive |= (1<<i); // active joint
        // 3. (axis,angle) to SO3:
        Lie::SO3 R_i = Lie::Exp_SO3_from_AxisAngle(w_i, t_i);
        Lie::SE3 G_i = Lie::SE3_from_SO3xR3(R_i, q_i); // relative RBT
        // 4. SE3 operation:
        // - propagate base frame TF (i-1) to tip frame (i) in current link:
        if (i)
            m_arm.sG0_[i] = m_arm.sG0_[i-1] * G_i; // <-- propagate fwd
        else
            m_arm.sG0_[i] = spatial_G_wam * G_i; // <-- initial based frame
        // - obtain global representation of the joint rotation axis:
        Lie::R3 w_ = m_arm.sG0_[i].block<3,3>(0,0) * ROTATION_AXIS_Z;
        // - twist:
        m_arm.sXi_[i].vw = RBT::twist_from_axis_point(w_, m_arm.sG0_[i].block<3,1>(0,3));
        m_arm.sXi_[i].theta = 0; // zero config
    }
    // - init base cam transformation:
    m_arm.sG_cam_base = Lie::SE3_from_SO3xR3(R_summit, summit_dP_cam_base);
    // - init tool cam transformation:
    m_arm.tG_cam_EE = Lie::SE3_from_SO3xR3(Lie::SO3::Identity(), wam_dP_cam_ee);
}

void ArmModel::acquireLock() {m_arm.guard.lock();}
void ArmModel::releaseLock() {m_arm.guard.unlock();}

void ArmModel::setAngles_unsafely(const double theta[], const size_t N)
{
    // m_arm.guard.lock();
    for (size_t i = 0; (i < N) && (i < ARM_NUM_DOF); i++)
    {
        m_arm.sXi_[i].theta = theta[i];
        m_arm.jUnProcessed |= (1<<i);
    }
    // m_arm.guard.unlock();
}
void ArmModel::setAngles_unsafely(const Vector7d_t theta)
{
    // m_arm.guard.lock();
    for (size_t i = 0; (i < 7) && (i < ARM_NUM_DOF); i++)
    {
        m_arm.sXi_[i].theta = theta(i);
        m_arm.jUnProcessed |= (1<<i);
    }
    // m_arm.guard.unlock();
}

void ArmModel::_compute_spatial_FK_unsafe()
{
    std::vector<Lie::SE3> s_G_st_; 
    Lie::SE3 s_G_st = Lie::SE3::Identity();
    for (std::size_t i = 0; i < ARM_NUM_LINKS; i++)
    {
        Lie::SE3 exp_xi_theta_i = Lie::Exp_SE3_from_TwistAngle(m_arm.sXi_[i]);
        s_G_st = s_G_st * exp_xi_theta_i;           // exp(xi * theta) = exp_0 * ... * exp_i;
        m_arm.sG_[i] = (s_G_st * m_arm.sG0_[i]);    // g_st(theta) = exp(xi * theta) * g_st(0);
    }
}

void ArmModel::processJntAngles_unsafely()
{
    // m_arm.guard.lock();
    if (m_arm.jUnProcessed == m_arm.jActive) // all joints are active and unprocessed
    {
        this->_compute_spatial_FK_unsafe();
        m_arm.jUnProcessed = 0x0000; // reset
    }
    // m_arm.guard.unlock();
}

bool ArmModel::getLinkPose_unsafely(Lie::SE3& T, const size_t index){
    bool success = (index < ARM_NUM_LINKS) && (m_arm.jUnProcessed == 0x0000);
    if (success)
    {
        // m_arm.guard.lock();
        T = m_arm.sG_[index];
        // m_arm.guard.unlock();
    }
    return success;
}
bool ArmModel::getAllLinkPoses_unsafely(Lie::SE3 T[]){
    bool success = (m_arm.jUnProcessed == 0x0000);
    if (success)
    {
        std::memcpy(&T, &m_arm.sG_, sizeof(Lie::SE3)*ARM_NUM_LINKS);
    }
    return success;
}
bool ArmModel::getEndEffectorPose_unsafely(Lie::SE3& T){
    bool success = (m_arm.jUnProcessed == 0x0000);
    if (success)
    {
        // m_arm.guard.lock();
        T = m_arm.sG_[ARM_NUM_LINKS - 1];
        // m_arm.guard.unlock();
    }
    return success;
}
Lie::SE3 ArmModel::getCamEE(){
    return m_arm.tG_cam_EE;
}
Lie::SE3 ArmModel::getCamBase(){
    return m_arm.sG_cam_base;
}