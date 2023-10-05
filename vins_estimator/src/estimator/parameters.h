/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

// ----------------------------------------------------------------
// : Parameters :
// ----------------------------------------------------------------
#define MAX_NUM_CAMERAS     (2U) // 2: stereo, 1: mono
#define MAX_NUM_DEVICES     (2U) // 2: dual, 1: standalone
#define BASE_DEV            (0U)
#define EE_DEV              (1U)

#define FOCAL_LENGTH        ((double)   (460.0))
#define WINDOW_SIZE         ((int)      (10))
#define NUM_OF_F            ((int)      (1000))
#define DEFAULT_GRAVITY     (Eigen::Vector3d(0.0, 0.0, 9.8)) // default G is assumed to be -ve z-axis


// ----------------------------------------------------------------
// : Feature Definitions :
// ----------------------------------------------------------------
#define ENABLED             (1U)
#define DISABLED            (0U)
#define NOT_IMPLEMENTED     (0U)
#define FOREVER             (1U)

#define FEATURE_ENABLE_STEREO_SUPPORT        (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_PT_CLOUD_SUPPORT      (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_8UC1_IMAGE_SUPPORT    (NOT_IMPLEMENTED) // allow 8UC1 image support per device [opt out for runtime]
#define FEATURE_ENABLE_STANDALONE_SUPPORT    (NOT_IMPLEMENTED) // allow standalone support for one device [opt out for runtime]

// ----------------------------------------------------------------
// : ROS TOPICS :
// ----------------------------------------------------------------
#define FUSE_TOPIC(BRANCH, LEAF)      ( BRANCH "\/" LEAF )// combine string
// publisher: ----------------------------------------------------
#define TOPIC_CAMERA_POSE_B           (FUSE_TOPIC("base" , "camera_pose"))
#define TOPIC_CAMERA_POSE_E           (FUSE_TOPIC("EE"   , "camera_pose"))
#define TOPIC_ODOMETRY_B              (FUSE_TOPIC("base" , "odometry"))
#define TOPIC_ODOMETRY_E              (FUSE_TOPIC("EE"   , "odometry"))
#define TOPIC_IMU_PROPAGATE_B         (FUSE_TOPIC("base" , "imu_propagate"))
#define TOPIC_IMU_PROPAGATE_E         (FUSE_TOPIC("EE"   , "imu_propagate"))
#define TOPIC_KEY_POSES_B             (FUSE_TOPIC("base" , "key_poses"))
#define TOPIC_KEY_POSES_E             (FUSE_TOPIC("EE"   , "key_poses"))
#define TOPIC_KEYFRAME_POINT_B        (FUSE_TOPIC("base" , "keyframe_point"))
#define TOPIC_KEYFRAME_POINT_E        (FUSE_TOPIC("EE"   , "keyframe_point"))
#define TOPIC_KEYFRAME_POSE_B         (FUSE_TOPIC("base" , "keyframe_pose"))
#define TOPIC_KEYFRAME_POSE_E         (FUSE_TOPIC("EE"   , "keyframe_pose"))
#define TOPIC_EXTRINSIC_B             (FUSE_TOPIC("base" , "extrinsic"))
#define TOPIC_EXTRINSIC_E             (FUSE_TOPIC("EE"   , "extrinsic"))
#define TOPIC_CAMERA_POSE_VISUAL_B    (FUSE_TOPIC("base" , "camera_pose_visual"))
#define TOPIC_CAMERA_POSE_VISUAL_E    (FUSE_TOPIC("EE"   , "camera_pose_visual"))
#define TOPIC_PATH_B                  (FUSE_TOPIC("base" , "path"))
#define TOPIC_PATH_E                  (FUSE_TOPIC("EE"   , "path"))
#define TOPIC_IMAGE_TRACK_B           (FUSE_TOPIC("base" , "image_track"))
#define TOPIC_IMAGE_TRACK_E           (FUSE_TOPIC("EE"   , "image_track"))
#define TOPIC_POINT_CLOUD_B           (FUSE_TOPIC("base" , "point_cloud"))
#define TOPIC_POINT_CLOUD_E           (FUSE_TOPIC("EE"   , "point_cloud"))
#define TOPIC_MARGIN_CLOUD_B          (FUSE_TOPIC("base" , "margin_cloud"))
#define TOPIC_MARGIN_CLOUD_E          (FUSE_TOPIC("EE"   , "margin_cloud"))
#define PUBLISHER_BUFFER_SIZE         (1000)

// ----------------------------------------------------------------
// : Public Functions :
// ----------------------------------------------------------------
void readParameters(std::string config_file);

// ----------------------------------------------------------------
// : Definitions :
// ----------------------------------------------------------------
enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

enum CalibVersion
{
    CALIB_NO_PRIOR    = (2),
    CALIB_INIT_GUESS  = (1),
    CALIB_EXACT       = (0),
};


typedef struct{
    int                 DEVICE_ID;
// [Imaging Cameras]:
    std::string         IMAGE_TOPICS[MAX_NUM_CAMERAS]; // ros topic
    // std::vector<std::string> CAM_NAMES;
    std::string         CAM_NAMES[MAX_NUM_CAMERAS]; // cameras intrinsic
    // - camera parameters:
    int                 NUM_OF_CAM;     // number of cameras
    int                 STEREO;         // auto-assign stereo: if n_cam = 2
    int                 ROW, COL;       // ?

// [IMU]:
    std::string         IMU_TOPIC;                      // 
    int                 USE_IMU;                        // if imu
    double              ACC_N, ACC_W;                   //
    double              GYR_N, GYR_W;                   //

// [Calibration]:
    double              TD;                             //
    int                 ESTIMATE_TD;                    //
    int                 ESTIMATE_EXTRINSIC;             //
    // std::vector<Eigen::Matrix3d> RIC;                   //
    // std::vector<Eigen::Vector3d> TIC;                   //
    Eigen::Matrix3d     RIC[MAX_NUM_CAMERAS];
    Eigen::Vector3d     TIC[MAX_NUM_CAMERAS];

// [Output]:
    std::string         EX_CALIB_RESULT_PATH;           //
    std::string         VINS_RESULT_PATH;               //
    std::string         OUTPUT_FOLDER;                  //
// [Front-end Algorithm]:
    int                 MULTIPLE_THREAD;    // threading enabled 1, 0 disables
    // - Feature Tracker:
    int                 MAX_CNT;            // 
    int                 MIN_DIST;           // 
    double              F_THRESHOLD;        // 
    int                 SHOW_TRACK;         // show tracking
    int                 FLOW_BACK;          // reverse optical-flow check in feature_tracker.cpp
    // - OP params:
    int                 NUM_ITERATIONS;     //
    double              MIN_PARALLAX;       //
    double              INIT_DEPTH;         //
    double              BIAS_GYR_THRESHOLD; //TODO: should we apply these to individual device config?
    double              BIAS_ACC_THRESHOLD; //TODO: should we apply these to individual device config?
    double              SOLVER_TIME;        //
    // - Const Params:
    Eigen::Vector3d     G;                  // gravity constant
} DeviceConfig_t;

// ----------------------------------------------------------------
// : Global Data Placeholder:
// ----------------------------------------------------------------
// extern DeviceConfig_t   DEV_CONFIG;
extern int              N_DEVICES;
extern DeviceConfig_t   DEV_CONFIGS[MAX_NUM_DEVICES];
extern map<int, Eigen::Vector3d> pts_gt;
