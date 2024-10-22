#pragma once 

#ifndef ESTIMATOR_MANAGER_H
#define ESTIMATOR_MANAGER_H

#include <thread>
#include <mutex>
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "estimator.h"
#include "parameters.h"
#include "../utility/visualization.h"

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
#include "../robot/ArmModel.h"
#endif

class EstimatorManager
{
    public:
        EstimatorManager(std::shared_ptr<DeviceConfig_t> _pCfgs[]);
        ~EstimatorManager();
        // init:
        void registerPublishers(ros::NodeHandle &n, const int N_DEVICES);
        // reset:
        void restartManager();
        // interface:
        void inputIMU(const size_t DEV_ID, const double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
        void inputImage(double t_b, const cv::Mat &_img_b, double t_e, const cv::Mat &_img_e);
#if (FEATURE_ENABLE_VICON_SUPPORT)
        void inputVicon_safe(const size_t DEV_ID, const pair<double, Vector7d_t> &_T_pq);
#endif // (FEATURE_ENABLE_VICON_SUPPORT)
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
        void inputArmJnts_safe(const double t, const Vector7d_t &_jnt_msg);
        void emptyArmJnts_safe(const double t);
#endif

        // thread:
        void publishVisualization_thread();
        void processMeasurements_thread();

    private:
#if (FEATURE_ENABLE_VICON_SUPPORT)
typedef struct{
            // Compensator:
            Lie::SO3 dR0 = Lie::SO3::Identity(); 
            Lie::R3  dP0 = Lie::R3::Zero();
            // cache:
            Lie::SO3 prev_R0, R0;
            Lie::R3  prev_P0, P0;
            // protected:
            queue<pair<double, Vector7d_t>>             data; // R3 + Q4
            std::mutex                                  guard;
            bool                                        started = false;
        } vicon_buffer_t;
        vicon_buffer_t m_vicon[MAX_NUM_DEVICES];
        void _process_and_queueVicon_safe(const size_t DEV_ID, const pair<double, Vector7d_t> &_T_pq);
#endif //(FEATURE_ENABLE_VICON_SUPPORT)

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
        typedef struct{
            // protected:
            queue<pair<double, Vector7d_t>>             data;
            std::mutex                                  guard;
        } arm_buffer_t;
        typedef struct{
            // cache:
            double      t_header = -1; 
            Vector7d_t  arm_vec;
            Lie::SE3    arm_g_st;
            // output:
            double      arm_pose_header = -1; // [UNUSED]
            Lie::SE3    arm_pose_st;
            Lie::SE3    arm_pose_st_0;
            Lie::SE3    arm_pose_ts;
            Lie::SE3    arm_pose_ts_0;
            // correction:
            Lie::SE3    init_dT_t;
            Lie::SE3    init_dT_s;

            Lie::SE3    init_T_c1;
            Lie::SE3    init_T_c2;

            bool        init_T_c1_inited = false;  
            bool        init_T_c2_inited = false;  

            bool        arm_pose_st_ready = false;
            bool        arm_pose_ts_ready = false;
            bool        arm_pose_st_inited = false;
            bool        arm_pose_ts_inited = false;
            bool        init_dT_t_inited = false;
            bool        init_dT_s_inited = false;
        } arm_data_t;
        arm_buffer_t                    arm_buf;
        arm_data_t                      arm_prev_data;

        typedef struct{
            Lie::SO3 last_R[MAX_NUM_DEVICES];
            Lie::R3  last_P[MAX_NUM_DEVICES];
            Lie::SO3 R0[MAX_NUM_DEVICES];
            Lie::R3  P0[MAX_NUM_DEVICES];
            bool     last_RP_ready[MAX_NUM_DEVICES];
        } manager_data_t;
        manager_data_t                  m_data;

        std::shared_ptr<ArmModel>       pArm;
        
        bool _getJointVector_safe(pair<double, Vector7d_t> &jnt_buf, const double t, const bool if_pop);
        void _postProcessArmJnts_unsafe(const double t, const Vector7d_t& jnt_vec);
#endif

        std::shared_ptr<DeviceConfig_t> pCfgs[MAX_NUM_DEVICES];
        std::shared_ptr<Estimator>      pEsts[MAX_NUM_DEVICES];

        std::shared_ptr<std::thread>    pProcessThread = nullptr;
        std::shared_ptr<std::thread>    pPublishThread = nullptr;

};

#endif // !ESTIMATOR_MANAGER_H