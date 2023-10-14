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
// ros:
#include <sensor_msgs/JointState.h>

// for print:
#include <stdio.h>

using namespace std;

// ----------------------------------------------------------------
// : Definitions :
// ----------------------------------------------------------------
#define ENABLED             (1U)
#define DISABLED            (0U)
#define NOT_IMPLEMENTED     (0U)
#define FOREVER             (1U)

#define NO_IMG              (cv::Mat())

// ----------------------------------------------------------------
// : Parameters :
// ----------------------------------------------------------------
#define MAX_NUM_CAMERAS     (2U) // 2: stereo, 1: mono
#define MAX_NUM_DEVICES     (2U) // 2: dual, 1: standalone
#define BASE_DEV            (0U)
#define EE_DEV              (1U)

// ----------------------------------------------------------------
// : Hyper-Parameters :
// ----------------------------------------------------------------
#define FOCAL_LENGTH        ((double)   (460.0))
#define WINDOW_SIZE         ((int)      (10))
#define NUM_OF_F            ((int)      (1000))
#define DEFAULT_GRAVITY     (Eigen::Vector3d(0.0, 0.0, 9.8)) // default G is assumed to be -ve z-axis

#define IMAGE_FPS                               (float)(30)     // [UNUSED]--> implies the frame difference between two cameras can be up to 0.015~0.03333 ms

/* Hyperparams:
*   @IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX:
*       - Ex: 0.03 sync tolerance: we throw out images if the time between two devices' image is greater than this value
*       - for 30Hz FPS two devices: timing gap is within 0.033 seconds
*       - range: [0.003, 0.03] // the original article for stereo images is 0.003, but the tol between devices can be greater than the same device stereo images
*   @IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE:
*       - Ex: 0.1 seconds tolerance: drop frames if we are processing the image 0.1s behind the schedule, wrt the latest image feed
*       - the smaller the stricter the schedule, 
*           - the lower latency between the resultant pose estimation wrt the latest image
*           - the higher frame drops on lower cpu/gpu power
*           - ALERT: a too small number will result unstable trajectory, a too large number will cumulates large latency overtime on the estimations
*           - Rule of thumb: a right threshold will result a converging frame drop rates, one can risk a smaller number for short demos
*           - a high rate of image stabilizatio is required for unstable device, like ee camera
*       - for 30Hz x2 devices, the rate can be minimum 0.033 seconds, lets give it a tolerance of 0.06 for a 15Hz processing rate
*       - range: [0.033, 0.1]: 0.1: about 10% drop rates, ee path is squiggly. 0.06: steady increasing to 20% for 40s rung
*/
//[Later] TODO: should we parameterize as config hyper parameter? depending on the hardware, the rate might be dfifferent.
#define IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX    (double)(0.03) 
#define IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE    (double)(0.06) 
#define IMAGE_ARM_SYNC_TIME_DELTA_MAX           (double)(0.002)  // 500 Hz --> 0.002 s --> tol: 0.002 s: allowance of 1-2 ticks

// ----------------------------------------------------------------
// : ROS TOPICS :
// ----------------------------------------------------------------
#define FUSE_TOPIC(BRANCH, LEAF)      ( BRANCH "/" LEAF )// combine string
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

#define TOPIC_VICON_PATH_B            (FUSE_TOPIC("base" , "vicon/path"))
#define TOPIC_VICON_PATH_E            (FUSE_TOPIC("EE"   , "vicon/path"))

// buffer size: -----------------------------
#define SUB_IMG_BUFFER_SIZE         (30U)
#define SUB_IMU_BUFFER_SIZE         (100U)
#define SUB_FEAT_BUFFER_SIZE        (100U)
#define SUB_ARM_BUFFER_SIZE         (100U)

// converter: --------------------------------
#define CV_YAML_TO_BOOL(X)          ((bool)(static_cast<int>(X)!=0))
#define FLOAT_IN_RANGE(X, BOUND)    (((X < BOUND) && (X > -BOUND)))
#define FLOAT_IN_BOUND(X, LB, UB)   (((X < UB) && (X > LB)))

// ----------------------------------------------------------------
// : Feature Definitions :
// ----------------------------------------------------------------
#define FEATURE_ENABLE_STEREO_SUPPORT        (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_PT_CLOUD_SUPPORT      (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_8UC1_IMAGE_SUPPORT    (NOT_IMPLEMENTED) // allow 8UC1 image support per device [opt out for runtime]
#define FEATURE_ENABLE_STANDALONE_SUPPORT    (NOT_IMPLEMENTED) // allow standalone support for one device [opt out for runtime]

// performance related feature support:
#define FEATURE_ENABLE_PERFORMANCE_EVAL                 ( ENABLED) // report performance
/* To enforce the real-time performance, we will drop frames if we are n seconds behind the schedule */
#define FEATURE_ENABLE_FRAME_DROP_FOR_REAL_TIME         ( ENABLED) // set via "IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE"

// vicon support:
#define FEATURE_ENABLE_VICON_SUPPORT                    ( ENABLED) // to feed vicon data and output as nav_msg::path for visualization
// arm odometry support:
#define FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT             ( ENABLED) // [WIP]

// debug only features:
#define FEATURE_CONSOLE_PRINTF                          ( ENABLED)
#define FEATURE_CONSOLE_DEBUG_PRINTF                    ( ENABLED)
#define FEATURE_VIZ_ROSOUT_ODOMETRY_SUPPORT             (DISABLED)

// debug only features (additional images):
#define FEATURE_DEBUG_IMAGE_AT_CONNECTIONS              (DISABLED)
// ----------------------------------------------------------------
// : Debug printfs :
// ----------------------------------------------------------------
#if (FEATURE_CONSOLE_PRINTF)
#   define PRINTF(...)       {printf(__VA_ARGS__); printf("\n"); fflush(stdout);}
#   define PRINT_INFO(...)   {printf(" \033[0;36m > [INFO ] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_WARN(...)   {printf(" \033[0;32m > [WARN ] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_ERROR(...)  {printf(" \033[0;35m > [ERROR] \033[0m "); PRINTF(__VA_ARGS__);}
#else
#   define PRINTF(...)       {}// Do Nothing
#   define PRINT_INFO(...)   {}// Do Nothing
#   define PRINT_WARN(...)   {}// Do Nothing
#   define PRINT_ERROR(...)  {}// Do Nothing
#endif
#if (FEATURE_CONSOLE_DEBUG_PRINTF)
#   define PRINT_DEBUG_FULL(...)  {printf("%s %s:%d", __TIME__, __FILE__, __LINE__); printf(" \033[0;33m > [DEBUG-FULL] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_DEBUG(...)  {printf(" \033[0;33m > [DEBUG-INFO] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_ARRAY(array, length) {printf(" \033[0;33m > [DEBUG-ARR] ["); for(int i = 0; i < length; i++){printf("%f\t", array[i]);} PRINTF("] \033[0m ");}
#   define PRINT_ARRAY_DIFF(array1, array2, length) {printf(" \033[0;33m > [DEBUG-ARR] ["); for(int i = 0; i < length; i++){printf("%f\t", (array1[i]-array2[i]));} PRINTF("] \033[0m ");}
#else
#   define PRINT_DEBUG_FULL(...)  {}// Do Nothing
#   define PRINT_DEBUG(...)  {}// Do Nothing
#   define PRINT_ARRAY(...)  {}// Do Nothing
#   define PRINT_ARRAY_DIFF(...)  {}// Do Nothing
#endif

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

typedef Eigen::Matrix<double, 7, 1> Vector7d_t;

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
    // - Optionals: GT vicon
#if (FEATURE_ENABLE_VICON_SUPPORT)
    // vicon:
    std::string          VICON_TOPIC;
#endif
} DeviceConfig_t;

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
typedef struct{
    // topics:
    std::string          JOINTS_TOPIC;
    std::string          POSE_TOPIC;
    // calibs:
    std::string          CALIBRATION_FILE_PATH;

    // UNUSED:
    Eigen::Matrix4d      T_BS; // Transformation from start to base camera position
    Eigen::Matrix4d      T_TE; // Transformation from end-effector camera to tool tip
    bool                 estimate_arm_extrinsics;
} ArmConfig_t;
#endif
// ----------------------------------------------------------------
// : Public Functions :
// ----------------------------------------------------------------
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    int readParameters(
        const std::string config_file, 
        DeviceConfig_t   DEV_CONFIGS[], 
        ArmConfig_t     &ARM_CONFIG); //--> N_DEVICES
#else
    int readParameters(
        const std::string config_file, 
        DeviceConfig_t   DEV_CONFIGS[]); //--> N_DEVICES
#endif
