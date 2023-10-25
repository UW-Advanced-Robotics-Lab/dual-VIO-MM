#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "Lie.h"
#include "RBT.h"
#include "OpenChain.h"

#include <mutex>

#define ROTATION_AXIS_Z     (Lie::R3(0,0,1))
#define ARM_NUM_DOF         (7U)
#define ARM_NUM_LINKS       (ARM_NUM_DOF + 1U)

// --- Configuration:
#define ARM_MODEL_CONFIG_L_SHOULDER          ((double)(0.346))
#define ARM_MODEL_CONFIG_L_ARM               ((double)(0.55))
#define ARM_MODEL_CONFIG_L_ELBOW             ((double)(0.3))
#define ARM_MODEL_CONFIG_L_WRIST             ((double)(0.06))
#define ARM_MODEL_CONFIG_W_ELBOW_JNT         ((double)(0.045))

class ArmModel { 
public:
    ArmModel();
    ~ArmModel();
    // lock:
    void acquireLock();
    // input:
    void setAngles_unsafely(const double theta[], const size_t N);
    void setAngles_unsafely(const Vector7d_t theta);
    // compute:
    void processJntAngles_unsafely();
    // output cached results:
    bool getLinkPose_unsafely(Lie::SE3& T, const size_t index);
    bool getAllLinkPoses_unsafely(Lie::SE3 T[]);
    bool getEndEffectorPose_unsafely(Lie::SE3& T);
    // lock release:
    void releaseLock();

private:
    typedef struct{
        Lie::SE3 sG0_[ARM_NUM_LINKS]; // zero config
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