/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// for print:
#include <stdio.h>

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
#define HYPERPARAMS_NEW_IMAGE_TIME_TOLERANCE_MIN            ((double) (1.0))
#define HYPERPARAMS_SKIP_FIRST_N_CNTS_IMAGES                (10) //?
// ----------------------------------------------------------------
// : ROS TOPICS :
// ----------------------------------------------------------------
#define FUSE_TOPIC(BRANCH, LEAF)      ( BRANCH "/" LEAF )// combine string
#define FUSE_SUB_TOPIC(NODE, BRANCH, LEAF)  ( "/" NODE "/" BRANCH "/" LEAF )// combine string

// mathcing estimator: --------------------------------------------
#define SUB_TOPIC_ODOMETRY_B              (FUSE_SUB_TOPIC("vins_estimator", "base" , "odometry"))
#define SUB_TOPIC_ODOMETRY_E              (FUSE_SUB_TOPIC("vins_estimator", "EE"   , "odometry"))
#define SUB_TOPIC_KEYFRAME_POSE_B         (FUSE_SUB_TOPIC("vins_estimator", "base" , "keyframe_pose"))
#define SUB_TOPIC_KEYFRAME_POSE_E         (FUSE_SUB_TOPIC("vins_estimator", "EE"   , "keyframe_pose"))
#define SUB_TOPIC_EXTRINSIC_B             (FUSE_SUB_TOPIC("vins_estimator", "base" , "extrinsic"))
#define SUB_TOPIC_EXTRINSIC_E             (FUSE_SUB_TOPIC("vins_estimator", "EE"   , "extrinsic"))
#define SUB_TOPIC_KEYFRAME_POINT_B        (FUSE_SUB_TOPIC("vins_estimator", "base" , "keyframe_point"))
#define SUB_TOPIC_KEYFRAME_POINT_E        (FUSE_SUB_TOPIC("vins_estimator", "EE"   , "keyframe_point"))
#define SUB_TOPIC_MARGIN_CLOUD_B          (FUSE_SUB_TOPIC("vins_estimator", "base" , "margin_cloud"))
#define SUB_TOPIC_MARGIN_CLOUD_E          (FUSE_SUB_TOPIC("vins_estimator", "EE"   , "margin_cloud"))
#define SUBSCRIPTION_BUFFER_SIZE          (2000)

// publisher: ----------------------------------------------------
#define TOPIC_MATCH_IMAGE_B               (FUSE_TOPIC("base" , "match_image"))
#define TOPIC_MATCH_IMAGE_E               (FUSE_TOPIC("EE"   , "match_image"))
#define TOPIC_CAMERA_POSE_VISUAL_B        (FUSE_TOPIC("base" , "camera_pose_visual"))
#define TOPIC_CAMERA_POSE_VISUAL_E        (FUSE_TOPIC("EE"   , "camera_pose_visual"))
#define TOPIC_POINT_CLOUD_LOOP_RECT_B     (FUSE_TOPIC("base" , "point_cloud_loop_rect"))
#define TOPIC_POINT_CLOUD_LOOP_RECT_E     (FUSE_TOPIC("EE"   , "point_cloud_loop_rect"))
#define TOPIC_MARGIN_CLOUD_LOOP_RECT_B    (FUSE_TOPIC("base" , "margin_cloud_loop_rect"))
#define TOPIC_MARGIN_CLOUD_LOOP_RECT_E    (FUSE_TOPIC("EE"   , "margin_cloud_loop_rect"))
#define TOPIC_ODOMETRY_RECT_B             (FUSE_TOPIC("base" , "odometry_rect"))
#define TOPIC_ODOMETRY_RECT_E             (FUSE_TOPIC("EE"   , "odometry_rect"))
#define PUBLISHER_BUFFER_SIZE             (1000)

// posegraph:
#define TOPIC_POSE_GRAPH_PATH_B           (FUSE_TOPIC("base" , "pose_graph_path"))
#define TOPIC_POSE_GRAPH_PATH_E           (FUSE_TOPIC("EE"   , "pose_graph_path"))
#define TOPIC_POSE_GRAPH_B                (FUSE_TOPIC("base" , "pose_graph"))
#define TOPIC_POSE_GRAPH_E                (FUSE_TOPIC("EE"   , "pose_graph"))
#define TOPIC_POSE_BASE_PATH_B            (FUSE_TOPIC("base" , "path_"))
#define TOPIC_POSE_BASE_PATH_E            (FUSE_TOPIC("EE"   , "path_"))

// converter: --------------------------------
#define CV_YAML_TO_BOOL(X)      ((bool)(static_cast<int>(X)!=0))

// ----------------------------------------------------------------
// : Feature Definitions :
// ----------------------------------------------------------------
#define ENABLED             (1U)
#define DISABLED            (0U)
#define NOT_IMPLEMENTED     (0U)
#define FOREVER             (1U)

// #define FEATURE_ENABLE_STEREO_SUPPORT        (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
// #define FEATURE_ENABLE_PT_CLOUD_SUPPORT      (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
// #define FEATURE_ENABLE_8UC1_IMAGE_SUPPORT    (NOT_IMPLEMENTED) // allow 8UC1 image support per device [opt out for runtime]
// #define FEATURE_ENABLE_STANDALONE_SUPPORT    (NOT_IMPLEMENTED) // allow standalone support for one device [opt out for runtime]
#define FEATURE_ENABLE_KEYBOARD_INTERFACE (DISABLED) 

// visual only features:
#define FEATURE_VISUAL_ONLY                             (ENABLED)
#define FEATURE_ENABLE_VISUAL_MARGIN_POINT_CLOUD_SUPPORT        ((ENABLED) && (FEATURE_VISUAL_ONLY))
#define FEATURE_ENABLE_VISUAL_POINT_CLOUD_SUPPORT               ((ENABLED) && (FEATURE_VISUAL_ONLY))
#define FEATURE_ENABLE_VISUAL_CAMERA_POSE_SUPPORT               ((ENABLED) && (FEATURE_VISUAL_ONLY))

// debug only features:
#define FEATURE_CONSOLE_PRINTF                          (ENABLED)
#define FEATURE_CONSOLE_DEBUG_PRINTF                    (ENABLED)
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
#   define PRINT_DEBUG_FULL(...)  {printf("%s %s:%d", __TIME__, __FILE__, __LINE__); printf(" \033[0;33m > [DEBUG] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_DEBUG(...)  {printf(" \033[0;33m > [DEBUG-INFO] \033[0m "); PRINTF(__VA_ARGS__);}
#else
#   define PRINT_DEBUG(...)  {}// Do Nothing
#endif
// ----------------------------------------------------------------
// : Definitions :
// ----------------------------------------------------------------
typedef struct{
    int                     DEVICE_ID;
    // config placeholders:
    int                     VISUALIZATION_SHIFT_X;
    int                     VISUALIZATION_SHIFT_Y;
    int                     SKIP_CNT;
    double                  SKIP_DIS;
    // config:
    int                     ROW;
    int                     COL;
    int                     DEBUG_IMAGE;
    bool                    USE_IMU;
    bool                    LOAD_PREVIOUS_POSE_GRAPH;

    // path configs:
    std::string             BRIEF_PATTERN_FILE;
    std::string             POSE_GRAPH_SAVE_PATH;
    std::string             OUTPUT_PATH;
    std::string             VINS_RESULT_PATH;
    std::string             CAM0_CALIB_PATH;
    std::string             VOCAB_FILE_PATH;
    std::string             IMAGE_TOPIC;
} DeviceConfig_t;

typedef struct{
    camodocal::CameraPtr    m_camera;
    Eigen::Vector3d         tic;
    Eigen::Matrix3d         qic;
    std::mutex              guard;
    // ros publisher
    ros::Publisher*         p_match_image_publisher;
} LoopDevice_t;
// ----------------------------------------------------------------
// : Public Functions :
// ----------------------------------------------------------------
int readParameters(
    const std::string config_file, 
    const std::string pkg_path, 
    DeviceConfig_t DEV_CONFIGS[], 
    LoopDevice_t LoopDevices[]); //-->N_DEVICES
