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
#define IMPLEMENTED         (1U)
#define FOREVER             (1U)
#define TODO                (0U)
#define HACK_ON             (1U)
#define HACK_OFF            (0U)

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
#define WINDOW_SIZE         ((int)      (10))
#define NUM_OF_F            ((int)      (1000))
#define DEFAULT_GRAVITY     (Eigen::Vector3d(0.0, 0.0, 9.8)) // default G is assumed to be -ve z-axis

#define IMAGE_FPS                               ((float)(30))      // ->implies the frame difference between two cameras can be up to 0.015~0.03333 ms
#define IMAGE_PROCESSING_FPS                    ((float)(15))      // [run-time,perf] set processing rate [15, 30], reduce if the frame drops are significant
#define IMAGE_PROCESSING_INTERVAL               ((float)(1.0/(IMAGE_PROCESSING_FPS)))

#define RUN_EST_MANAGER_AT_FIXED_RATE           (DISABLED) // Enable to run the estimator at a fixed rate, else run as fast as possible
#define     EST_MANAGER_PROCESSING_FPS              ((int)(20))
#define     EST_MANAGER_PROCESSING_INTERVAL_MS      ((int)(1000/EST_MANAGER_PROCESSING_FPS))
// #define     EST_MANAGER_PROCESSING_SLEEP_MS         ((int)(2)) //2MS fixed sleep , comment out for no sleep

#define TOPIC_PUBLISH_FPS                       ((int)(10))        // [run-time,visual] reduce if the frame drops are significant, visual only
#define TOPIC_PUBLISH_INTERVAL_MS               ((int)(1000/TOPIC_PUBLISH_FPS))

#define N_FRAMES_BEHIND_FOR_LOOP_FUSION         (2) 
#define NTH_FRAME                               (WINDOW_SIZE - N_FRAMES_BEHIND_FOR_LOOP_FUSION)
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
#define IMAGE_ARM_SYNC_TIME_DELTA_MAX           (double)(0.01)

// [imu]
#define NUMERICAL_UNSTABLE_COEF_THRESHOLD       (float)(1e8)
#define IF_NUMERICAL_UNSRABLE(jacob)            (bool)(((jacob).maxCoeff() > (NUMERICAL_UNSTABLE_COEF_THRESHOLD)) || ((jacob).minCoeff() < -(NUMERICAL_UNSTABLE_COEF_THRESHOLD)))

// [Arm] TODO: add to config file: TODO: Time Decay based 
#define ESTIMATOR_ARM_FACTOR_TO_EE              (double)(150.0) // 100.0
#define ESTIMATOR_ARM_FACTOR_TO_EE_Q            (double)(10.0) 
#define ESTIMATOR_ARM_FACTOR_TO_EE_Z            (double)(200.0) // 200.0
#define ESTIMATOR_ARM_FACTOR_TO_BASE            (double)(10.0)  //  20.0
#define ESTIMATOR_ARM_FACTOR_TO_BASE_Q          (double)(10.0) 
#define ESTIMATOR_ARM_FACTOR_TO_BASE_Z          (double)(20.0) // 200.0
// ----------------------------------------------------------------
// : Feature Definitions :
// ----------------------------------------------------------------
// MAJOR FEATURES:
#define FEATURE_MODE_RUNTIME                 (DISABLED) // enable: to disable unnecessary features
#define FEATURE_MODE_DEBUG_RUNTIME           ( ENABLED) // overrides below ...
#define FEATURE_MODE_DEBUG_KINEMATICS        ( ENABLED) 

// selection:
#define USER_PARAMS                 (IMPLEMENTED) // indicates as USER_PARAMS, do not disable
#define ZEROING_WRT_TO_BASE         ((USER_PARAMS) & (DISABLED)) // enable tp debug raw odometry offset from base
#define USER_VICON_DEBUG            ((USER_PARAMS) & (DISABLED)) // enable to debug arm config (TODO: the BASE->EE seems not working?)
#define FLAG_OURS                   ((USER_PARAMS) & ( ENABLED)) // To toggle between our tightly-coupled solution vs baseline

// ----------------------------------------------------------------
// FUTURE SUPPORTS:
#define FEATURE_ENABLE_STEREO_SUPPORT        (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_PT_CLOUD_SUPPORT      (NOT_IMPLEMENTED) // allow stereo support per device [TODO: let's study stereo later]
#define FEATURE_ENABLE_8UC1_IMAGE_SUPPORT    (NOT_IMPLEMENTED) // allow 8UC1 image support per device [opt out for runtime]
#define FEATURE_ENABLE_STANDALONE_SUPPORT    (NOT_IMPLEMENTED) // allow standalone support for one device [opt out for runtime]
#define FEATURE_ENABLE_CALIBRATION           (NOT_IMPLEMENTED) // allow extrinsic calibration
// ----------------------------------------------------------------
// Performance Support:
// performance related feature support:
#define FEATURE_ENABLE_PERFORMANCE_EVAL                 (( ENABLED) & (!FEATURE_MODE_RUNTIME)) // report performance
#    define FEATURE_PERFORMANCE_EVAL_THRESHOLD_MS           (30U) // TODO: [concise] report to console when above X ms, else ros debug
#    define FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE         (( ENABLED) & (FEATURE_ENABLE_PERFORMANCE_EVAL)) // report performance
#    define FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR       ((DISABLED) & (FEATURE_ENABLE_PERFORMANCE_EVAL)) // report performance
//TODO: we should have a performance logger to a text file

// other feature support more than originally supported:
#define FEATURE_ASSUME_INIT_IMU_TO_BE_ZEROED_ABSOLUTELY ( ENABLED)
//      - default: enabled, originally it is zeroed against the initial value, true to set to zero at the init.
//      - disabled: it may not be steady when system stopped
#define FEATURE_REINIT_ONLY_AT_NON_LINEAR_OPTIMIZATION  ( ENABLED)
//     - default: enabled
//     - disabled: reinit even when initialization failed, discard the slidewindow, restart
#define FEATURE_BROADCAST_ESTIMATOR_TF                  (DISABLED)
//     - original: enabled, but we are not using it

// ----------------------------------------------------------------
// MAJOR SUPPORTS:
#if (FEATURE_MODE_RUNTIME)
/* To enforce the real-time performance, we will drop frames if we are n seconds behind the schedule */
/* FEATURE_ENABLE_FRAME_DROP_FOR_REAL_TIME
*  @problem: it seems that we are running behind the schedule for the high res image feeds ??? 
*  @solution: 
*       Let's try to drop the frame if we are behind the schedule
*
*   FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT
*   @issue: idk why the original code drops frames if threading, inside the Estimator::inputImage
*   @solution: 
*       Let's try to drop the frame before the input image
*   @ solution to above: by isolating visual publisher to a separate thread, we are able to achieve no frame loss with two flags disabled
*/
    #define FEATURE_ENABLE_DYNAMIC_FRAME_DROP_FOR_RT        (DISABLED) // set via "IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE"
    #define FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT         ( ENABLED) // set via #define IMAGE_PROCESSING_FPS
    #define FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT_INSIDE_INPUT_IMG    (! FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT)
    // ... NOT ADDED YET
    
#elif (FEATURE_MODE_DEBUG_RUNTIME)
    // fps:
    #define FEATURE_ENABLE_DYNAMIC_FRAME_DROP_FOR_RT                        (DISABLED) // set via "IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE"
    #define FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT                         ( ENABLED) // set via #define IMAGE_PROCESSING_FPS
    #define FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT_INSIDE_INPUT_IMG        (! FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT)
    // other features:
    #define FEATURE_ENABLE_STATISTICS_LOGGING                               (DISABLED) // `printStatistics`
    #define FEATURE_NON_THREADING_SUPPORT                                   (DISABLED) // disable non-threading
    #define FEATURE_ESTIMATOR_THREADING_SUPPORT                             (DISABLED) // backward-compatible with threading in estimator
    #define FEATURE_PERMIT_WITHOUT_IMU_SUPPORT                              (NOT_IMPLEMENTED) // since there is no support from stereo, we will assume imu to be enabled all the time

    // vicon support:
    #define FEATURE_ENABLE_VICON_SUPPORT                                    (( ENABLED) || (USER_VICON_DEBUG)) // to feed vicon data and output as nav_msg::path for visualization
    #   define FEATURE_ENABLE_VICON_ZEROING_SUPPORT                             ( ENABLED) // zeroing vicons independently
    #       define FEATURE_ENABLE_VICON_ZEROING_WRT_BASE_SUPPORT                (ZEROING_WRT_TO_BASE) // zeroing vicons wrt base
    #       define FEATURE_ENABLE_VICON_ZEROING_ENFORCE_SO2                     ( ENABLED) // enforcing SO2 to R
    // initialize vicon after sfm:
    //     - disable in ZEROING_WRT_TO_BASE: as we are basing vicon wrt base only
    #   define FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_SFM                     ((USER_PARAMS) & ( DISABLED))
    #   define FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_BOTH_SFM                ((USER_PARAMS) & ( DISABLED)) 
    #       define FEATURE_ENABLE_VICON_ZEROING_ORIENTATION_WRT_BASE_SUPPORT    (FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_BOTH_SFM)
    // TODO: we should try to initialize when one is not inited/ stationary

    // arm odometry support:
    #define FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT                             ( ENABLED) // [WIP]
    #define     FEATURE_ENABLE_ARM_VICON_SUPPORT                            (USER_VICON_DEBUG) 
    //              - stubing vicon base data for arm odometry (to see arm kinematics accuracy)
    #define     FEATURE_ENABLE_ARM_ODOMETRY_VIZ                             (( ENABLED))
    #define         FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM                     (FEATURE_ENABLE_ARM_ODOMETRY_VIZ) // init first pose with arm odometry
    #define     FEATURE_ENABLE_ARM_ODOMETRY_FACTOR                          ((FLAG_OURS)) // [Our-Solution] Arm Marginalization
    //              - disabled [baseline]: baseline without arm odometry marginalization
    //              - enabled  [ours]: with arm odometry marginalization
    #define         FEATURE_ENABLE_ARM_ODOMETRY_WRT_TO_BASE                 (ZEROING_WRT_TO_BASE) // init first pose with arm odometry
    //              - enabled [default]: consider relative poses with respect to the first frame
    //              - disabled: to see direct comparison vs vicon
    #define         FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE                  (( ENABLED))
    //                  - apply arm odometry to estimate base from ee
    #define         FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE_ENFORCE_SE2      (( ENABLED) & (!USER_VICON_DEBUG))
    #define         FEATURE_ENABLE_ARM_ODOMETRY_POSE_ZERO_ENFORCE_SO2       ((DISABLED) & (!USER_VICON_DEBUG))
    #define         FEATURE_ENABLE_ARM_ODOMETRY_POSE_NO_ORIENTATION         (( ENABLED) & (!USER_VICON_DEBUG))
    //                  - enforcing SO2 projection for arm odometry to estimate base from ee
    #define         FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_TO_BASE              ((FLAG_OURS) & ( ENABLED))
    //                  - apply arm odometry factor to base
    #define         FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_NON_UNIFORM          ((FLAG_OURS) & ( ENABLED))
    #define     FEATURE_ENABLE_ARM_ODOMETRY_TO_POSE_INIT                    (TODO) //TODO: we should figureout a way to apply pose

#elif (FEATURE_MODE_DEBUG_KINEMATICS) // (Debugging kinematics)
    // other features:
    #define FEATURE_ENABLE_STATISTICS_LOGGING               (DISABLED) // `printStatistics`
    #define FEATURE_NON_THREADING_SUPPORT                   (DISABLED) // disable non-threading
    #define FEATURE_ESTIMATOR_THREADING_SUPPORT             (DISABLED) // backward-compatible with threading in estimator
    #define FEATURE_PERMIT_WITHOUT_IMU_SUPPORT              (FEATURE_ENABLE_STEREO_SUPPORT) // since there is no support from stereo, we will assume imu to be enabled all the time
    // vicon support:
    #define FEATURE_ENABLE_VICON_SUPPORT                    ( ENABLED) // to feed vicon data and output as nav_msg::path for visualization
    #   define FEATURE_ENABLE_VICON_ZEROING_SUPPORT             (( ENABLED) & (FEATURE_ENABLE_VICON_SUPPORT)) // zeroing vicons independently
    #       define FEATURE_ENABLE_VICON_ZEROING_WRT_BASE_SUPPORT    ((ENABLED) & (FEATURE_ENABLE_VICON_ZEROING_SUPPORT)) // zeroing vicons wrt base
    //              - disable [default]: to see direct comparison vs vicon
    //              - enable [debugging arm odom]: to see direct comparison vs arm odometry
    #   define FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_SFM         ((DISABLED) & (FEATURE_ENABLE_VICON_SUPPORT)) // initialize vicon after sfm
    //        - disable [default]: to see direct comparison at the end of first image processing (regardless whether system inited)
    //        - enable: to see direct comparison after system initialized from SfM

    // arm odometry support:
    #define FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT             ( ENABLED) // [WIP]
    #define     FEATURE_ENABLE_ARM_VICON_SUPPORT               (( ENABLED) & (FEATURE_ENABLE_VICON_SUPPORT)) // stubing vicon base data for arm odometry (to see arm kinematics accuracy)
    #define     FEATURE_ENABLE_ARM_ODOMETRY_VIZ                (( ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT))
    #define         FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM             (( ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)) // init first pose with arm odometry
    #define     FEATURE_ENABLE_ARM_ODOMETRY_FACTOR             ((DISABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)) // [Our-Solution] Arm Marginalization
    //              - disabled [baseline]: baseline without arm odometry marginalization
    //              - enabled  [ours]: with arm odometry marginalization
    #define         FEATURE_ENABLE_ARM_ODOMETRY_ZEROING             ((ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)) // init first pose with arm odometry
    //              - enabled [default]: consider relative poses with respect to the first frame
    // #define         FEATURE_ENABLE_ARM_ODOMETRY_BASE_TO_EE          (( ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_FACTOR)) 
    #define         FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE          (( ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT))
    //                  - apply arm odometry to estimate base from ee
    #define         FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE_ENFORCE_SO2     (( ENABLED))
    //                  - enforcing SO2 projection for arm odometry to estimate base from ee
    #define         FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_TO_BASE      (( ENABLED) & (FEATURE_ENABLE_ARM_ODOMETRY_FACTOR))
    //                  - apply arm odometry factor to base
#endif //(FEATURE_MODE_DEBUG_KINEMATICS)

// debug only features:
#define FEATURE_DEBUGGING                               (( ENABLED) & (!FEATURE_MODE_RUNTIME))
#   define FEATURE_CONSOLE_PRINTF                           (( ENABLED) & (FEATURE_DEBUGGING))
#   define FEATURE_CONSOLE_DEBUG_PRINTF                     (( ENABLED) & (FEATURE_DEBUGGING))
#   define FEATURE_VIZ_ROSOUT_ODOMETRY_SUPPORT              ((DISABLED) & (FEATURE_DEBUGGING))
#   define FEATURE_TRACKING_IMAGE_SUPPORT                   (( ENABLED) & (FEATURE_DEBUGGING)) 
#   define FEATURE_PERFORMANCE_DEBUG_PRINTF                 (( ENABLED) & (FEATURE_DEBUGGING)) 
#   define FEATURE_VIZ_PUBLISH                              (( ENABLED) & (FEATURE_DEBUGGING)) 
#   define FEATURE_ROS_PUBLISH_IMU_PROPAGATION              ((DISABLED) & (FEATURE_DEBUGGING)) 
// debug only features (additional images):
#   define FEATURE_DEBUG_IMAGE_AT_CONNECTIONS               ((DISABLED) & (FEATURE_DEBUGGING))
// ----------------------------------------------------------------
// : Hyperparams :
// ----------------------------------------------------------------
#define ESTIMATOR_CPP_IMU_EXCITATION_STRICT_MIN_VAR_THRESHOLD      (0.25f)
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
#   define PRINT_DEBUG_FULL(...)        {printf("%s %s:%d", __TIME__, __FILE__, __LINE__); printf(" \033[0;33m > [DEBUG-FULL] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_DEBUG(...)             {printf(" \033[0;33m > [DEBUG-INFO] \033[0m "); PRINTF(__VA_ARGS__);}
#   define PRINT_ROS_DEBUG(...)         {printf(" \033[0;33m > [DEBUG-INFO] \033[0m "); PRINTF(__VA_ARGS__); ROS_DEBUG(__VA_ARGS__);}
#   define PRINT_ARRAY(array, length)   {printf(" \033[0;33m > [DEBUG-ARR] ["); for(int i = 0; i < length; i++){printf("%f\t", array[i]);} PRINTF("] \033[0m ");}
#   define PRINT_ARRAY_DIFF(array1, array2, length) {printf(" \033[0;33m > [DEBUG-ARR] ["); for(int i = 0; i < length; i++){printf("%f\t", (array1[i]-array2[i]));} PRINTF("] \033[0m ");}
#else
#   define PRINT_DEBUG_FULL(...)    {}// Do Nothing
#   define PRINT_DEBUG(...)         {}// Do Nothing
#   define PRINT_ROS_DEBUG(...)     {ROS_DEBUG(__VA_ARGS__);}
#   define PRINT_ARRAY(...)         {}// Do Nothing
#   define PRINT_ARRAY_DIFF(...)    {}// Do Nothing
#endif

#if (FEATURE_ENABLE_PERFORMANCE_EVAL & FEATURE_CONSOLE_DEBUG_PRINTF)
#   define TIK(PM)              {PM.tic();}
#   define TOK(PM)              {if (PM.toc() > FEATURE_PERFORMANCE_EVAL_THRESHOLD_MS) { PRINT_DEBUG("%s Perf: %.2f ms", (#PM), (PM.dt())); }; }
#   define TOK_FORCE(PM)        { PRINT_DEBUG("%s Perf: %.2f ms", (#PM), (PM.dt())); }
#   define TOK_TAG(PM, TAG)     {if (PM.toc() > FEATURE_PERFORMANCE_EVAL_THRESHOLD_MS) { PRINT_DEBUG("%s(%d) Perf [%s]: %.2f ms", (#PM), (PM.n_tok()), (TAG), (PM.dt())); }; }
#   define TOK_IF(PM, MS)       {if (PM.toc() > MS) { PRINT_DEBUG("%s Perf: %.2f ms > %d ms", (#PM), (PM.dt()), (MS)); }; }
#else // precompile elimination for run-time performance
#   define TIK(PM)              {} // do nothing
#   define TOK(PM)              {} // do nothing
#   define TOK_FORCE(PM)        {} // do nothing
#   define TOK_TAG(PM, TAG)     {} // do nothing
#   define TOK_IF(PM, MS)       {} // do nothing
#endif


// ----------------------------------------------------------------
// : ROS TOPICS :
// ----------------------------------------------------------------
#define FUSE_TOPIC(BRANCH, LEAF)      ( BRANCH "/" LEAF )// combine string
// publisher: ----------------------------------------------------
// listened by loop fusion
//      - pub by processMeasurements_thread() everytime after successful processImage()
//      (buf->pose)
#define TOPIC_KEYFRAME_POSE_B         (FUSE_TOPIC("base" , "keyframe_pose")) // pubKeyframe_Odometry_and_Points_immediately [N Frames Behind]
#define TOPIC_KEYFRAME_POSE_E         (FUSE_TOPIC("EE"   , "keyframe_pose")) // pubKeyframe_Odometry_and_Points_immediately [N Frames Behind]
//      (buf->point)
#define TOPIC_KEYFRAME_POINT_B        (FUSE_TOPIC("base" , "keyframe_point"))// pubKeyframe_Odometry_and_Points_immediately [N Frames Behind]
#define TOPIC_KEYFRAME_POINT_E        (FUSE_TOPIC("EE"   , "keyframe_point"))// pubKeyframe_Odometry_and_Points_immediately [N Frames Behind]
//      (viz)
#define TOPIC_MARGIN_CLOUD_B          (FUSE_TOPIC("base" , "margin_cloud"))  // [VIZ] queue_PointCloud_unsafe -> pubPointClouds_safe 
#define TOPIC_MARGIN_CLOUD_E          (FUSE_TOPIC("EE"   , "margin_cloud"))  // [VIZ] queue_PointCloud_unsafe -> pubPointClouds_safe 
//      (camera_pose_visual, odometry_rect)
#define TOPIC_ODOMETRY_B              (FUSE_TOPIC("base" , "odometry"))      // [VIZ] queue_Odometry_unsafe -> pubOdometry_safe, pubOdometryPath_safe
#define TOPIC_ODOMETRY_E              (FUSE_TOPIC("EE"   , "odometry"))      // [VIZ] queue_Odometry_unsafe -> pubOdometry_safe, pubOdometryPath_safe
//      (dev->tic, dev->qic)
#define TOPIC_EXTRINSIC_B             (FUSE_TOPIC("base" , "extrinsic"))     // [VIZ] queue_TF_unsafe -> pubExtrinsic_TF_safe
#define TOPIC_EXTRINSIC_E             (FUSE_TOPIC("EE"   , "extrinsic"))     // [VIZ] queue_TF_unsafe -> pubExtrinsic_TF_safe
// not listened:
#define TOPIC_CAMERA_POSE_B           (FUSE_TOPIC("base" , "camera_pose"))
#define TOPIC_CAMERA_POSE_E           (FUSE_TOPIC("EE"   , "camera_pose"))
#if (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
#   define TOPIC_IMU_PROPAGATE_B      (FUSE_TOPIC("base" , "imu_propagate")) // UNUSED
#   define TOPIC_IMU_PROPAGATE_E      (FUSE_TOPIC("EE"   , "imu_propagate")) // UNUSED
#endif // (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
#define TOPIC_KEY_POSES_B             (FUSE_TOPIC("base" , "key_poses"))     // queue_KeyPoses_unsafe
#define TOPIC_KEY_POSES_E             (FUSE_TOPIC("EE"   , "key_poses"))     // queue_KeyPoses_unsafe
#define TOPIC_CAMERA_POSE_VISUAL_B    (FUSE_TOPIC("base" , "camera_pose_visual"))
#define TOPIC_CAMERA_POSE_VISUAL_E    (FUSE_TOPIC("EE"   , "camera_pose_visual"))
#define TOPIC_IMAGE_TRACK_B           (FUSE_TOPIC("base" , "image_track"))
#define TOPIC_IMAGE_TRACK_E           (FUSE_TOPIC("EE"   , "image_track"))
#define TOPIC_POINT_CLOUD_B           (FUSE_TOPIC("base" , "point_cloud"))
#define TOPIC_POINT_CLOUD_E           (FUSE_TOPIC("EE"   , "point_cloud"))
// Visualization and Evaluation:
#define TOPIC_PATH_B                  (FUSE_TOPIC("base" , "path"))
#define TOPIC_PATH_E                  (FUSE_TOPIC("EE"   , "path"))
#define PUBLISHER_BUFFER_SIZE         (1000)

// vicon Ground Truth:
#define TOPIC_VICON_PATH_B            (FUSE_TOPIC("base" , "vicon/path"))
#define TOPIC_VICON_PATH_E            (FUSE_TOPIC("EE"   , "vicon/path"))
// Arm Odometry Path (Debug/Reference):
#define TOPIC_ARM_PATH_B              (FUSE_TOPIC("base" , "arm/path"))
#define TOPIC_ARM_PATH_E              (FUSE_TOPIC("EE"   , "arm/path"))
// Arm Visualization:
#define TOPIC_ARM_PATH_VIZ            (FUSE_TOPIC("arm"   , "visual"))

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
    // std::vector<std::string> CAM_MODEL_PATH;
    std::string         CAM_MODEL_PATH[MAX_NUM_CAMERAS]; // cameras intrinsic
    // - camera parameters:
    int                 NUM_OF_CAM;     // number of cameras
    int                 STEREO;         // auto-assign stereo: if n_cam = 2
    int                 ROW, COL;       // ?
    double              FOCAL_LENGTH; // ((double)   (460.0)) //>>>???? why hardcoded
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
    // - SfM:
    double              MIN_PARALLAX_SFM;   //
    double              RANSAC_THRESHOLD_SFM;
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
        std::shared_ptr<DeviceConfig_t>     pCfgs[], 
        std::shared_ptr<ArmConfig_t>        pArmCfg); //--> N_DEVICES
#else
    int readParameters(
        const std::string config_file, 
        std::shared_ptr<DeviceConfig_t>     pCfgs[]); //--> N_DEVICES
#endif
