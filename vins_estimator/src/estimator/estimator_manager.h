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
        void inputImage(double t, const cv::Mat &_img_b, const cv::Mat &_img_e);

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
        void inputArm(double t, const sensor_msgs::JointStateConstPtr &_jnt_msg_b);
#endif
        // callbacks:
        // thread:
        void publishVisualization_thread();
        void processMeasurements_thread();

    private:
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
        typedef struct{
            // protected:
            queue<pair<double, Vector7d_t>> data;
            std::mutex                      guard;
        } arm_buffer_t;

        arm_buffer_t                    arm_buf;
        Lie::SE3                        arm_pose_st;
        std::shared_ptr<ArmModel>       pArm;
        void processArm_unsafe(const double t, const Vector7d_t& jnt_vec);
#endif

        std::shared_ptr<DeviceConfig_t> pCfgs[MAX_NUM_DEVICES];
        std::shared_ptr<Estimator>      pEsts[MAX_NUM_DEVICES];

        std::shared_ptr<std::thread>    pProcessThread = nullptr;
        std::shared_ptr<std::thread>    pPublishThread = nullptr;

};

#if (FEATURE_ENABLE_VICON_SUPPORT)
void callback_viconOdometry(const geometry_msgs::TransformStampedConstPtr &transform_msg, const int device_id);
#endif

#endif // !ESTIMATOR_MANAGER_H