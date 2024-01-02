#include "../estimator/parameters.h"
#include "ArmModel.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

// Arm Model Configuration:
const link_t ARM_LINKS[ARM_NUM_LINKS] = {
    { // F0-->F1
        .name = "base_B3350",
        .tip_frame_Tq = Lie::R3(0,0,0),
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
        .tip_frame_Tq = Lie::R3(-ARM_MODEL_CONFIG_W_ELBOW_JNT,-ARM_MODEL_CONFIG_L_ELBOW,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
        .tip_frame_Tt = M_PI_2, // pi/2
        .tip_frame_lb = -4.8,
        .tip_frame_ub = 1.3,
    },
    { // F5-->F6
        .name = "wrist_yaw_B3345",
        .tip_frame_Tq = Lie::R3(0,0,0),
        .tip_frame_Tw = Lie::R3(1,0,0),
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
        .tip_frame_Tq = Lie::R3(0,0,ARM_MODEL_CONFIG_L_WRIST),
        .tip_frame_Tw = Lie::R3(0,0,1),
        .tip_frame_Tt = 0, //
        .tip_frame_lb = 0,
        .tip_frame_ub = 0,
    },
#if (FEATURE_ENABLE_CAMERA_TOOL_TIPS)
    { // F8-->F9
        .name = "camera_EE",
        // .tip_frame_Tq = Lie::R3(-0.185091,0,0.052259),
        .tip_frame_Tq = Lie::R3(-0.092647,0,0.006), // z - ARM_MODEL_CONFIG_L_WRIST
        .tip_frame_Tw = Lie::R3(0,0,1),
        .tip_frame_Tt = 0, //
        .tip_frame_lb = 0,
        .tip_frame_ub = 0,
    },
#endif //(FEATURE_ENABLE_CAMERA_TOOL_TIPS)
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

#if 1 // [Dec 21 2023, 1221]
// TODO: we may make the config file for the arm model
const Lie::SE3 T0_summit         = Lie::SE3::Identity();
// -> robot frame:
const Lie::R3 summit_dP_wam      = Lie::R3(0.      , 0.      , 0.748879);       // vicon : vins-research-pkg/research-project/analysis/camera_report.py
const Lie::R3 summit_dP_cam_base = Lie::R3(0.20182, 0.      , 0.392077);  // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#if (FEATURE_ENABLE_CAMERA_TOOL_TIPS)
const Lie::R3 wam_dP_cam_ee = Lie::R3(0,0,0);
#else
const Lie::R3 wam_dP_cam_ee = Lie::R3(-0.092647,0,0.006);   // vicon : vins-research-pkg/research-project/analysis/camera_report.py
// const Lie::R3 wam_dP_cam_ee = Lie::R3(-0.106302,0,0.014592);   // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#endif //(FEATURE_ENABLE_CAMERA_TOOL_TIPS)
#elif 1 // [Dec 13 2023, 1213]
// TODO: we may make the config file for the arm model
const Lie::SE3 T0_summit = Lie::SE3::Identity();
// -> robot frame:
const Lie::R3 summit_dP_wam = Lie::R3(0.139676,0,0.746286);       // vicon : vins-research-pkg/research-project/analysis/camera_report.py
const Lie::R3 summit_dP_cam_base = Lie::R3(0.330588,0.0,0.389403);  // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#if (FEATURE_ENABLE_CAMERA_TOOL_TIPS)
const Lie::R3 wam_dP_cam_ee = Lie::R3(0,0,0);
#else
const Lie::R3 wam_dP_cam_ee = Lie::R3(-0.105301,0,0.026472);   // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#endif //(FEATURE_ENABLE_CAMERA_TOOL_TIPS)
#elif 1 // [Dec 2023, 1207]
// TODO: we may make the config file for the arm model
const Lie::SE3 T0_summit = Lie::SE3::Identity();
// -> robot frame:
const Lie::R3 summit_dP_wam = Lie::R3(0.142425,0,0.745);       // vicon : vins-research-pkg/research-project/analysis/camera_report.py
const Lie::R3 summit_dP_cam_base = Lie::R3(0.327884,0.0,0.40062);  // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#if (FEATURE_ENABLE_CAMERA_TOOL_TIPS)
const Lie::R3 wam_dP_cam_ee = Lie::R3(0,0,0);
#else
const Lie::R3 wam_dP_cam_ee = Lie::R3(-0.103501,0,-0.01);    // z - ARM_MODEL_CONFIG_L_WRIST  // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#endif //(FEATURE_ENABLE_CAMERA_TOOL_TIPS)
#else // [Nov 2023, 1127]
// TODO: we may make the config file for the arm model
const Lie::SE3 T0_summit = Lie::SE3::Identity();
// -> robot frame:
const Lie::R3 summit_dP_wam = Lie::R3(0.100901,0,0.709702);       // vicon : vins-research-pkg/research-project/analysis/camera_report.py
const Lie::R3 summit_dP_cam_base = Lie::R3(0.297595,0.0,0.399599);  // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#if (FEATURE_ENABLE_CAMERA_TOOL_TIPS)
const Lie::R3 wam_dP_cam_ee = Lie::R3(0,0,0);
#else
const Lie::R3 wam_dP_cam_ee = Lie::R3(-0.185091,0,0.042259);      // vicon : vins-research-pkg/research-project/analysis/camera_report.py
#endif //(FEATURE_ENABLE_CAMERA_TOOL_TIPS)
#endif

void ArmModel::_model_initialization_unsafe()
{
    // - summit pose:
    const Lie::SE3 spatial_G_summit = Lie::SE3::Identity() * T0_summit;
    // - summit ->> WAM base frame:
    // const Lie::Axis_Angle_t wt_summit = Lie::Axis_Angle_t{.w=Lie::R3(0,0,1),.theta=0};
    // const Lie::SO3 R_summit = Lie::Exp_SO3_from_AxisAngle(wt_summit);
    const Lie::SO3 R_summit = Lie::SO3::Identity();
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

void ArmModel::setAngles_unsafely(const double theta[], const size_t N, const double t)
{
    m_arm.t = t;
    // m_arm.guard.lock();
    for (size_t i = 0; (i < N) && (i < ARM_NUM_DOF); i++)
    {
        if (m_arm.jActive & (1<<i)) // apply actuation fiter
        {
            m_arm.sXi_[i].theta = theta[i];
            m_arm.jUnProcessed |= (1<<i);
        }
    }
    // m_arm.guard.unlock();
}
void ArmModel::setAngles_unsafely(const Vector7d_t theta, const double t)
{
    m_arm.t = t;
    // m_arm.guard.lock();
    for (size_t i = 0; (i < theta.size()) && (i < ARM_NUM_DOF); i++)
    {
        if (m_arm.jActive & (1<<i)) // apply actuation fiter
        {
            m_arm.sXi_[i].theta = theta(i);
            m_arm.jUnProcessed |= (1<<i);
        }
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
        for (size_t i = 0; i < ARM_NUM_LINKS; i++)
        {
            T[i] = m_arm.sG_[i];
        }
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
    return m_arm.tG_cam_EE; // robot axis
}
Lie::SE3 ArmModel::getCamBase(){
    return m_arm.sG_cam_base; // robot axis
}

void ArmModel::storeTransformations_unsafely(Lie::SE3& T){
    m_arm.sG_t = T;
}

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
void ArmModel::pub_Marker(ros::Publisher &pub) {
    nav_msgs::Path   arm_path;
    if (m_arm.t > 0)
    {
        Lie::SE3 Links[ARM_NUM_LINKS];
        this->acquireLock();
        bool success = this->getAllLinkPoses_unsafely(Links);
        this->releaseLock();
        if (success)
        {
            Lie::SE3 sGt = m_arm.sG_t;
            Lie::SO3 R; Lie::R3 p;

            geometry_msgs::PoseStamped pose_stamped;

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(m_arm.t);

            for (int i = 0; i < ARM_NUM_LINKS; i++) {
                Lie::SO3xR3_from_SE3(R, p, sGt * Links[i]);
                Eigen::Quaterniond q = Eigen::Quaterniond(R);
                // apply transformation to pose:
                geometry_msgs::Pose pose;
                pose.position.x = p(0);
                pose.position.y = p(1);
                pose.position.z = p(2);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();
                
                pose_stamped.header = header;
                pose_stamped.pose = pose;
                arm_path.header = header;
                arm_path.header.frame_id = "world";
                arm_path.poses.push_back(pose_stamped);
            }

            pub.publish(arm_path);
        }
    }
}
#endif
