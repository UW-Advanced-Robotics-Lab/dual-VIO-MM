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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/parameters.h"
#include "estimator/estimator_manager.h"

// ----------------------------------------------------------------
// : Global Data Placeholder:
// ----------------------------------------------------------------
typedef struct{
    // buffer:
    queue<sensor_msgs::ImageConstPtr>       img0_buf;
    std::mutex                              img0_mutex;
    double                                  time_last_update;
#if (FEATURE_ENABLE_VICON_SUPPORT)
    queue<pair<double, Vector7d_t>>         vicon_buf;
    std::mutex                              vicon_mutex;
#endif
} NodeBuffer_t;
NodeBuffer_t   m_buffer[MAX_NUM_DEVICES];


// Performance:
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE)
typedef struct{
    int image_frame_dropped_tick = 0;
    int image_valid_counter = 0;
    double image_process_time = 0;
    double image_process_delta_time = 0;
} RosNodeTestPerf_t;
RosNodeTestPerf_t m_perf;
#endif
// typedef Matrix<double, 7, 1> Vector7d_t; // for storing <time, acc, gyr>

// configs:
std::shared_ptr<DeviceConfig_t> pCfgs[MAX_NUM_DEVICES] = {
    std::make_shared<DeviceConfig_t>(), std::make_shared<DeviceConfig_t>()
};

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
std::shared_ptr<ArmConfig_t> pArmCfg = std::make_shared<ArmConfig_t>();
#endif

// Estimators:
std::shared_ptr<EstimatorManager> pEstMnger = std::make_shared<EstimatorManager>(pCfgs);


//////////////////////////////////////////
///////   CALLBACK FUNCTION     /////////
////////////////////////////////////////

void buf_img_safely(NodeBuffer_t * const pB, const sensor_msgs::ImageConstPtr &img_msg)
{
    pB->img0_mutex.lock();
    pB->time_last_update = img_msg->header.stamp.toSec();
    pB->img0_buf.push(img_msg);
    pB->img0_mutex.unlock();
}
void d0_img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    buf_img_safely(& m_buffer[BASE_DEV], img_msg);
}
void d1_img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    buf_img_safely(& m_buffer[EE_DEV], img_msg);
}

#if (FEATURE_ENABLE_VICON_SUPPORT)
void buf_vicon_safely(NodeBuffer_t * const pB, const geometry_msgs::TransformStampedConstPtr &transform_msg)
{
    pB->vicon_mutex.lock();
    Vector7d_t pose;
    pose << transform_msg->transform.translation.x,
            transform_msg->transform.translation.y,
            transform_msg->transform.translation.z,
            transform_msg->transform.rotation.w,
            transform_msg->transform.rotation.x,
            transform_msg->transform.rotation.y,
            transform_msg->transform.rotation.z; // w,x,y,z
    pB->vicon_buf.push(make_pair(transform_msg->header.stamp.toSec(), pose));
    pB->vicon_mutex.unlock();
}
void callback_viconOdometry_EE_safe(const geometry_msgs::TransformStampedConstPtr &transform_msg)
{
    buf_vicon_safely(& m_buffer[EE_DEV], transform_msg);
}
void callback_viconOdometry_base_safe(const geometry_msgs::TransformStampedConstPtr &transform_msg)
{
    buf_vicon_safely(& m_buffer[BASE_DEV], transform_msg);
}
#endif // (FEATURE_ENABLE_VICON_SUPPORT)

cv::Mat process_IMG_from_msg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat img = ptr->image.clone();
    return img;
}

double process_IMU_from_msg(const sensor_msgs::ImuConstPtr &imu_msg, Vector3d &acc, Vector3d &gyr)
{
    double t, x, y, z;
    t = imu_msg->header.stamp.toSec();
    x = (double)(imu_msg->linear_acceleration.x);
    y = (double)(imu_msg->linear_acceleration.y);
    z = (double)(imu_msg->linear_acceleration.z);
    acc = Vector3d(x, y, z);
    x = (double)(imu_msg->angular_velocity.x);
    y = (double)(imu_msg->angular_velocity.y);
    z = (double)(imu_msg->angular_velocity.z);
    gyr = Vector3d(x, y, z);
    return t;
}
void d0_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    Vector3d acc, gyr;
    double t;
    t = process_IMU_from_msg(imu_msg, acc, gyr);
    pEstMnger->inputIMU(BASE_DEV, t, acc, gyr);
}
void d1_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    Vector3d acc, gyr;
    double t;
    t = process_IMU_from_msg(imu_msg, acc, gyr);
    pEstMnger->inputIMU(EE_DEV, t, acc, gyr);
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        pEstMnger->restartManager();
    }
    return;
}

void armJoints_callback(const sensor_msgs::JointStateConstPtr &jnts_msg)
{
    Vector7d_t jnt_buf;
    double t = jnts_msg->header.stamp.toSec();
    for (int i = 0; i < ARM_NUM_DOF; i++) {
        jnt_buf(i) = jnts_msg->position[i];
    }
    pEstMnger->inputArmJnts_safe(t, jnt_buf);
}

//////////////////////////////////////
///////   MAIN FUNCTIONS     /////////
//////////////////////////////////////

// extract images with same timestamp from two topics
void sync_process_IMG_thread()
{
    NodeBuffer_t * const pB0 = & m_buffer[BASE_DEV];
    NodeBuffer_t * const pB1 = & m_buffer[EE_DEV  ];
    
    double d0_time_last_submitted = 0;
    while(FOREVER)
    {
        // [Assumption]: cameras are not offline
        {
            // Dual Monocular Cameras: --> Requires image synchronization
            // placeholders:
            cv::Mat d0_img, d1_img;
            double d0_time, d1_time, d01_delta, d0_time_latest;
            
            bool if_image_synced = false;
            
            // cache data if the frame timing is within the diff threshold @ IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX
            pB0->img0_mutex.lock();
            pB1->img0_mutex.lock();
            if (!pB0->img0_buf.empty() && !pB1->img0_buf.empty())
            {
                d0_time_latest = pB0->time_last_update;
                d0_time = pB0->img0_buf.front()->header.stamp.toSec();
                d1_time = pB1->img0_buf.front()->header.stamp.toSec();
                // time delta = t_d0 - t_d1 = base - ee
                d01_delta = (d0_time - d1_time);
                // sync tolerance
                if(d01_delta < (- IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX)) // img0 is ahead of img1
                {
                    pB0->img0_buf.pop();
                    PRINT_WARN("throw img0 dt=%f ",d01_delta);
                }
                else if(d01_delta > (+ IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX)) // img1 is ahead of img0
                {
                    pB1->img0_buf.pop();
                    PRINT_WARN("throw img1");
                }
                else // in-bound sync:
                {
                    d0_img     = process_IMG_from_msg(pB0->img0_buf.front());
                    pB0->img0_buf.pop();
                    d1_img     = process_IMG_from_msg(pB1->img0_buf.front());
                    pB1->img0_buf.pop();
                    if_image_synced = true;
                    // printf("synced img0 and img1");
                }
            }
            pB0->img0_mutex.unlock();
            pB1->img0_mutex.unlock();

#if (FEATURE_ENABLE_VICON_SUPPORT)
            // Sampling Vicon Buffers:
            {
                // BASE:
                pB0->vicon_mutex.lock();
                while (!pB0->vicon_buf.empty())
                {
                    // NOTE: should be behind the estimator time (0.1s ish)
                    pEstMnger->inputVicon_safe(BASE_DEV, pB0->vicon_buf.front());
                    pB0->vicon_buf.pop();
                }
                pB0->vicon_mutex.unlock();
                // EE:
                pB1->vicon_mutex.lock();
                while (!pB1->vicon_buf.empty())
                {
                    // NOTE: should be behind the estimator time (0.1s ish)
                    pEstMnger->inputVicon_safe(EE_DEV, pB1->vicon_buf.front());
                    pB1->vicon_buf.pop();
                }
                pB1->vicon_mutex.unlock();
                
            }
#endif // (FEATURE_ENABLE_VICON_SUPPORT)
            
            // Placeholder for joints:
            sensor_msgs::JointStateConstPtr jnt_msg_E = NULL;
            sensor_msgs::JointStateConstPtr jnt_msg_b = NULL;

            if(if_image_synced)
            {
                // measure time difference between the latest image and the current processed image:
                const double delta_time = d0_time_latest - d0_time; 
                const double delta_time_input = d0_time - d0_time_last_submitted;

#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE)
                m_perf.image_valid_counter ++;
#endif

#if (FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT && FEATURE_ENABLE_DYNAMIC_FRAME_DROP_FOR_RT)
                if (    (delta_time_input > IMAGE_PROCESSING_INTERVAL)          // if we are within processing FPS
                    &&  (delta_time < IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE))    // and, if we are on-schedule, queue for processing
#elif (FEATURE_ENABLE_DYNAMIC_FRAME_DROP_FOR_RT)
                if (delta_time < IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE)
#elif (FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT)
                if (delta_time_input > IMAGE_PROCESSING_INTERVAL)
#endif // otherwise:
                {
                    pEstMnger->inputImage(d0_time, d0_img, d1_time, d1_img); 
                    d0_time_last_submitted = d0_time; // to compute run-time processing rate

#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE)
                    m_perf.image_process_delta_time =ros::Time::now().toSec() - m_perf.image_process_time;
                    m_perf.image_process_time =ros::Time::now().toSec();
#endif
                } // else, drop the images if we are behind the schedule:
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE && (FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT || FEATURE_ENABLE_DYNAMIC_FRAME_DROP_FOR_RT))
                else if (delta_time > IMAGE_BEHIND_SCHEDULE_TIME_TOLERANCE)
                {
                    // Performance DEBUG: lets see the time difference between current and latest image.
                    m_perf.image_frame_dropped_tick ++;
                    float drop_rate_pa = (m_perf.image_frame_dropped_tick * 100.0)/(m_perf.image_valid_counter);
                    float fps = 1.0/m_perf.image_process_delta_time;
                    PRINT_WARN(" Frame Dropped [delta time=%f], Cumulated Frame Drop Rate: %.2f | FPS: %.2f", delta_time, drop_rate_pa, fps);
                }
                else
                {
                    m_perf.image_frame_dropped_tick ++;
                }
#endif
            }
        }
        
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    // Initialization:
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //TODO: need some sort of verbosity flag

    if(argc != 2)
    {
        PRINT_ERROR("please intput: rosrun vins vins_node [config file] "
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml ");
        return 1;
    }

    string config_file = argv[1];
    PRINT_INFO("config_file: %s", argv[1]);
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    const int N_DEVICES = readParameters(config_file, pCfgs, pArmCfg);
#else
    const int N_DEVICES = readParameters(config_file, pCfgs);
#endif    
    // Register Visual Publisher:
    pEstMnger->registerPublishers(n, N_DEVICES);
    pEstMnger->restartManager();

#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ROSNODE)
    m_perf.image_frame_dropped_tick = 0;
    m_perf.image_valid_counter = 0;
#endif

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ////////// BEGIN HERE //////////////////////////////////
    ROS_INFO("waiting for image and imu...");
    ROS_WARN("waiting for image and imu...");
    ROS_DEBUG("waiting for image and imu...");

    // Subscribe: TODO: (minor) eventually, we should utilize the boost::bind cast instead of explicit function
    ros::Subscriber sub_d0_imu = n.subscribe(pCfgs[BASE_DEV]->IMU_TOPIC, SUB_IMU_BUFFER_SIZE, d0_imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_d1_imu = n.subscribe(pCfgs[EE_DEV  ]->IMU_TOPIC, SUB_IMU_BUFFER_SIZE, d1_imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_d0_img0 = n.subscribe(pCfgs[BASE_DEV]->IMAGE_TOPICS[0], SUB_IMG_BUFFER_SIZE, d0_img0_callback);
    ros::Subscriber sub_d1_img0 = n.subscribe(pCfgs[EE_DEV  ]->IMAGE_TOPICS[0], SUB_IMG_BUFFER_SIZE, d1_img0_callback);
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    ros::Subscriber sub_arm_jnts = n.subscribe(pArmCfg->JOINTS_TOPIC, SUB_ARM_BUFFER_SIZE, armJoints_callback); //, ros::TransportHints().tcpNoDelay());
#endif
#if (FEATURE_ENABLE_VICON_SUPPORT)
    ros::Subscriber sub_d0_vicon = n.subscribe< geometry_msgs::TransformStamped>(pCfgs[BASE_DEV]->VICON_TOPIC, SUB_ARM_BUFFER_SIZE, callback_viconOdometry_base_safe);
    ros::Subscriber sub_d1_vicon = n.subscribe< geometry_msgs::TransformStamped>(pCfgs[EE_DEV  ]->VICON_TOPIC, SUB_ARM_BUFFER_SIZE, callback_viconOdometry_EE_safe);
#endif
    
    PRINT_INFO("==== [ Subscriptions Completed ] ==== ");
    PRINT_INFO("[Node] Sub IMU Topic   d0.0: %s", pCfgs[BASE_DEV]->IMU_TOPIC.c_str());
    PRINT_INFO("[Node] Sub IMU Topic   d1.0: %s", pCfgs[EE_DEV  ]->IMU_TOPIC.c_str());
    PRINT_INFO("[Node] Sub Image Topic d0.0: %s", pCfgs[BASE_DEV]->IMAGE_TOPICS[0].c_str());
    PRINT_INFO("[Node] Sub Image Topic d1.0: %s", pCfgs[EE_DEV  ]->IMAGE_TOPICS[0].c_str());
    
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);

    std::thread img_sync_thread{sync_process_IMG_thread};
    
    ////////// ENDS HERE //////////////////////////////////
    ros::spin();
    img_sync_thread.join();

    return 0;
}
