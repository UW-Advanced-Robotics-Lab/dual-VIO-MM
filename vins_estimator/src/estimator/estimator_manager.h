#pragma once 

#include <thread>
#include <mutex>
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

class EstimatorManager
{
    public:
        EstimatorManager(std::shared_ptr<DeviceConfig_t> _pCfgs[]);
        ~EstimatorManager();
        // reset:
        void restartManager();
        // interface:
        void inputIMU(const size_t DEV_ID, const double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
        void inputImage(double t, const cv::Mat &_img_b, const cv::Mat &_img_e, const sensor_msgs::JointStateConstPtr &jnt_msg=NULL);
    private:
        std::shared_ptr<DeviceConfig_t> pCfgs[MAX_NUM_DEVICES];
        std::shared_ptr<Estimator>      pEsts[MAX_NUM_DEVICES];

        // static void _process_JntVector_from_msg(const sensor_msgs::JointStateConstPtr &_jnt_msg, Vector7d_t &jnt_pos, Vector7d_t &jnt_vel, Vector7d_t &jnt_tau);
};