#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "Lie.h"
#include "RBT.h"
#include "OpenChain.h"

#include <ros/ros.h>

#include <mutex>

#define ROTATION_AXIS_Z     (Lie::R3(0,0,1))
#define ARM_NUM_DOF         (7U)
#define ARM_NUM_LINKS       (ARM_NUM_DOF + 1U)

// --- Configuration:
#define ARM_MODEL_CONFIG_L_SHOULDER          ((double)(0.352))
#define ARM_MODEL_CONFIG_L_ARM               ((double)(0.551))
#define ARM_MODEL_CONFIG_L_ELBOW             ((double)(0.3))
#define ARM_MODEL_CONFIG_L_WRIST             ((double)(0.072))
#define ARM_MODEL_CONFIG_W_ELBOW_JNT         ((double)(0.045))

class ArmModel { 
public:
    ArmModel();
    ~ArmModel();
    // lock:
    void acquireLock();
    // input:
    void setAngles_unsafely(const double theta[], const size_t N, const double t);
    void setAngles_unsafely(const Vector7d_t theta, const double t);
    // compute:
    void processJntAngles_unsafely();
    // output cached results:
    bool getLinkPose_unsafely(Lie::SE3& T, const size_t index);
    bool getAllLinkPoses_unsafely(Lie::SE3 T[]);
    bool getEndEffectorPose_unsafely(Lie::SE3& T);
    void storeTransformations_unsafely(Lie::SE3& T);
    // lock release:
    void releaseLock();
    // get:
    Lie::SE3 getCamEE();
    Lie::SE3 getCamBase();
    // viz:
#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
    void pub_Marker(ros::Publisher &pub);
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)



private:
    typedef struct{
        double      t;
        Lie::SE3    sG_t = Lie::SE3::Identity();
        // cam config:
        Lie::SE3 sG_cam_base;
        Lie::SE3 tG_cam_EE;
        // zero config:
        Lie::SE3 sG0_[ARM_NUM_LINKS];
        // joint related:
        uint16_t jActive = 0;
        double jLimit_lb[ARM_NUM_LINKS];
        double jLimit_ub[ARM_NUM_LINKS];
        // lock:
        std::mutex guard;
        // locked:
        Lie::Twist_Angle_t  sXi_[ARM_NUM_LINKS];
        uint16_t            jUnProcessed = 0;
        Lie::SE3            sG_[ARM_NUM_LINKS]; // result
    } data_t;
    
    data_t m_arm;

    void _model_initialization_unsafe();
    void _compute_spatial_FK_unsafe();
};


#endif // ARM_MODEL_H