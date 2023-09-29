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
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define SUB_IMG_BUFFER_SIZE         (30U)
#define SUB_IMU_BUFFER_SIZE         (100U)
#define SUB_FEAT_BUFFER_SIZE        (100U)

#define IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX    (double)(0.003) // 0.003s sync tolerance [33Hz]
#define IMU_SYNCHRONIZATION_TIME_DELTA_MAX      (double)(0.001) // 0.0001s sync tolerance [100Hz]
#define IMU_SYNCHRONIZATION_DO_NOT_USE          (DISABLED) // Assume: IMU is not sync due to high rate

typedef struct{
    // buffer:
#if (IMU_SYNCHRONIZATION_DO_NOT_USE)
    queue<sensor_msgs::ImuConstPtr>         imu_buf;
    std::mutex                              imu_mutex;
#endif
    queue<sensor_msgs::ImageConstPtr>       img0_buf;
    std::mutex                              img0_mutex;
    // queue<sensor_msgs::JointStateConstPtr>  jnt_buf;
    // std::mutex                              jnt_mutex;
    
    // module:
    Estimator                               estimator;
#if (FEATURE_ENABLE_STEREO_SUPPORT)
    queue<sensor_msgs::ImageConstPtr>   img1_buf;
#endif
} NodeData_t;

typedef Matrix<double, 7, 1> Vector7d_t; // for storing <time, acc, gyr>

///////////////////////////
///////   DATA     ////////
///////////////////////////
NodeData_t  m_data[MAX_NUM_DEVICES]; // each device has its own estimator


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
#if (FEATURE_ENABLE_STEREO_SUPPORT)
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}
#endif

void buf_img_safely(NodeData_t * const pD, const sensor_msgs::ImageConstPtr &img_msg)
{
    pD->img0_mutex.lock();
    pD->img0_buf.push(img_msg);
    pD->img0_mutex.unlock();
}
void d0_img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    buf_img_safely(& m_data[BASE_DEV], img_msg);
}
void d1_img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    buf_img_safely(& m_data[EE_DEV], img_msg);
}

cv::Mat process_IMG_from_msg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
#if (FEATURE_ENABLE_8UC1_IMAGE_SUPPORT)
    if (img_msg->encoding == "8UC1") // mono 
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else // color --> mono
#endif
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process_IMG()
{
    NodeData_t * const pD0 = & m_data[BASE_DEV];
    NodeData_t * const pD1 = & m_data[EE_DEV  ];
    
    while(FOREVER)
    {
#if (FEATURE_ENABLE_STEREO_SUPPORT)
        assert("stereo support is not implemented yet");
#endif
#if (FEATURE_ENABLE_STANDALONE_SUPPORT)
        if (N_DEVICES == 1) 
        {
            // Standalone Monocular Camera:
            cv::Mat image;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = pD0->img0_buf.front()->header.stamp.toSec();
                image = process_IMG_from_msg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                pD0->estimator.inputImage(time, image);
        }
        else
#endif // we assume dual by default.
        {
            // Dual Monocular Cameras: --> Requires image synchronization
            // placeholders:
            cv::Mat d0_img, d1_img;
            double d0_time, d1_time, d01_delta;
            
            bool if_image_synced = false;
            
            // cache data:
            pD0->img0_mutex.lock();
            pD1->img0_mutex.lock();
            if (!pD0->img0_buf.empty() && !pD1->img0_buf.empty())
            {
                d0_time = pD0->img0_buf.front()->header.stamp.toSec();
                d1_time = pD1->img0_buf.front()->header.stamp.toSec();
                // time delta = t_d0 - t_d1 = base - ee
                d01_delta = (d0_time - d1_time);
                // 0.003s sync tolerance
                if(d01_delta < (- IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX)) // img0 is ahead of img1
                {
                    pD0->img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(d01_delta > (+ IMAGE_SYNCHRONIZATION_TIME_DELTA_MAX)) // img1 is ahead of img0
                {
                    pD1->img0_buf.pop();
                    printf("throw img1\n");
                }
                else // in-bound sync:
                {
                    d0_img     = process_IMG_from_msg(pD0->img0_buf.front());
                    pD0->img0_buf.pop();
                    d1_img     = process_IMG_from_msg(pD1->img0_buf.front());
                    pD1->img0_buf.pop();
                    if_image_synced = true;
                    // printf("synced img0 and img1\n");
                }
            }
            pD0->img0_mutex.unlock();
            pD1->img0_mutex.unlock();

            if(if_image_synced)
            {
                // decoupled estimator
                // TODO: we should consider coupling the estimators (stereo for the same states)
                pD0->estimator.inputImage(d0_time, d0_img);
                pD1->estimator.inputImage(d0_time, d1_img);
            }
        }
        
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

#if (IMU_SYNCHRONIZATION_DO_NOT_USE)
    void buf_imu_safely(NodeData_t * const pD, const sensor_msgs::ImuConstPtr &imu_msg)
    {
        pD->imu_mutex.lock();
        pD->imu_buf.push(imu_msg);
        pD->imu_mutex.unlock();
    }
    void d0_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        buf_imu_safely(& m_data[BASE_DEV]);
    }
    void d1_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        buf_imu_safely(& m_data[EE_DEV]);
    }
    void process_IMU_from_msg(const sensor_msgs::ImuConstPtr &imu_msg, Vector3d &acc, Vector3d &gyr)
    {
        acc[1] = (double)(imu_msg->linear_acceleration.x);
        acc[2] = (double)(imu_msg->linear_acceleration.y);
        acc[3] = (double)(imu_msg->linear_acceleration.z);
        gyr[1] = (double)(imu_msg->angular_velocity.x);
        gyr[2] = (double)(imu_msg->angular_velocity.y);
        gyr[3] = (double)(imu_msg->angular_velocity.z);
    }
    void sync_process_IMU()
    {
        NodeData_t * const pD0 = & m_data[BASE_DEV];
        NodeData_t * const pD1 = & m_data[EE_DEV  ];
        
        while(FOREVER)
        {
    #if (FEATURE_ENABLE_STANDALONE_SUPPORT)
            if (N_DEVICES == 1)
            { // Standalone IMU:
                double time;
                Vector3d acc, gyr;
                bool processed = false;
                
                pD0->imu_mutex.lock();
                if (!pD0->imu_buf.empty())
                {
                    time = pD0->img0_buf.front()->header.stamp.toSec();
                    process_IMU_from_msg(pD0->imu_buf.front(), acc, gyr);
                    pD0->imu_buf.pop();
                    processed = true;
                }
                pD0->imu_mutex.unlock();
                if (processed)
                {
                    pD0->estimator.inputIMU(time, acc, gyr); // TODO: feed in IMU data
                }
            }
            else
    #endif
            {
                // Dual IMU Sync: --> Requires imu synchronization
                // placeholders:
                Vector3d d0_acc, d0_gyr;
                Vector3d d1_acc, d1_gyr;
                double d0_time, d1_time, d01_delta;
                
                bool if_imu_synced = false;
                
                // cache data:
                pD0->imu_mutex.lock();
                pD1->imu_mutex.lock();
                if (!pD0->imu_buf.empty() && !pD1->imu_buf.empty())
                {
                    d0_time = pD0->imu_buf.front()->header.stamp.toSec();
                    d1_time = pD1->imu_buf.front()->header.stamp.toSec();
                    // time delta = t_d0 - t_d1 = base - ee
                    d01_delta = (d0_time - d1_time);
                    // 0.003s sync tolerance
                    if(d01_delta < (- IMU_SYNCHRONIZATION_TIME_DELTA_MAX)) // img0 is ahead of img1
                    {
                        pD0->imu_buf.pop(); 
                        printf("throw imu0\n");
                    }
                    else if(d01_delta > (+ IMU_SYNCHRONIZATION_TIME_DELTA_MAX)) // img1 is ahead of img0
                    {
                        pD1->imu_buf.pop();
                        printf("throw imu1\n");
                    }
                    else // in-bound sync:
                    {
                        process_IMU_from_msg(pD0->imu_buf.front(), d0_acc, d0_gyr);
                        pD0->imu_buf.pop();
                        process_IMU_from_msg(pD1->imu_buf.front(), d1_acc, d1_gyr);
                        pD1->imu_buf.pop();
                        if_image_synced = true;
                        // printf("synced imu0 and imu1\n");
                    }
                }
                pD0->imu_mutex.unlock();
                pD1->imu_mutex.unlock();

                if(if_imu_synced)
                {
                    // decoupled estimator
                    // TODO: we should consider coupling the estimators (stereo for the same states)
                    pD0->estimator.inputIMU(d0_time, d0_acc, d0_gyr);
                    pD1->estimator.inputIMU(d0_time, d1_acc, d1_gyr);
                }
            }
            
            std::chrono::milliseconds dura(1);
            std::this_thread::sleep_for(dura);
        }
    }
#else // IMU not synchronized
    double process_IMU_from_msg(const sensor_msgs::ImuConstPtr &imu_msg, Vector3d &acc, Vector3d &gyr)
    {
        double t = imu_msg->header.stamp.toSec();
        acc[1] = (double)(imu_msg->linear_acceleration.x);
        acc[2] = (double)(imu_msg->linear_acceleration.y);
        acc[3] = (double)(imu_msg->linear_acceleration.z);
        gyr[1] = (double)(imu_msg->angular_velocity.x);
        gyr[2] = (double)(imu_msg->angular_velocity.y);
        gyr[3] = (double)(imu_msg->angular_velocity.z);
        return t;
    }
    void d0_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        Vector3d acc, gyr;
        double t;
        t = process_IMU_from_msg(imu_msg, acc, gyr);
        m_data[BASE_DEV].estimator.inputIMU(t, acc, gyr);
    }
    void d1_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        Vector3d acc, gyr;
        double t;
        t = process_IMU_from_msg(imu_msg, acc, gyr);
        m_data[EE_DEV  ].estimator.inputIMU(t, acc, gyr);
    }
#endif // END of IMU_SYNCHRONIZATION_DO_NOT_USE

#if (FEATURE_ENABLE_PT_CLOUD_SUPPORT)
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}
#endif // FEATURE_ENABLE_PT_CLOUD_SUPPORT
#if (FEATURE_ENABLE_STEREO_SUPPORT)
void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, DEV_CONFIGS[BASE_DEV].STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, DEV_CONFIGS[BASE_DEV].STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(DEV_CONFIGS[BASE_DEV].USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(DEV_CONFIGS[BASE_DEV].USE_IMU, 0);
    }
    return;
}
#endif // FEATURE_ENABLE_STEREO_SUPPORT

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_data[BASE_DEV].estimator.clearState();
        m_data[EE_DEV  ].estimator.clearState();
        m_data[BASE_DEV].estimator.setParameter();
        m_data[EE_DEV  ].estimator.setParameter();
    }
    return;
}

int main(int argc, char **argv)
{
    // Initialization:
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //TODO: need some sort of verbosity flag

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    
    m_data[BASE_DEV].estimator.setParameter();
    m_data[EE_DEV  ].estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ////////// BEGIN HERE //////////////////////////////////
    ROS_INFO("waiting for image and imu...");
    ROS_WARN("waiting for image and imu...");
    ROS_DEBUG("waiting for image and imu...");

    registerPub(n);

    // Subscribe:
    ros::Subscriber sub_d0_imu = n.subscribe(DEV_CONFIGS[BASE_DEV].IMU_TOPIC, SUB_IMU_BUFFER_SIZE, d0_imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_d1_imu = n.subscribe(DEV_CONFIGS[EE_DEV  ].IMU_TOPIC, SUB_IMU_BUFFER_SIZE, d1_imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_d0_img0 = n.subscribe(DEV_CONFIGS[BASE_DEV].IMAGE_TOPICS[0], SUB_IMG_BUFFER_SIZE, d0_img0_callback);
    ros::Subscriber sub_d1_img0 = n.subscribe(DEV_CONFIGS[EE_DEV  ].IMAGE_TOPICS[0], SUB_IMG_BUFFER_SIZE, d1_img0_callback);
    
    printf("==== [ Subscriptions Completed ] ==== \n");
    printf("[Node] Sub IMU Topic   d0.0: %s\n", DEV_CONFIGS[BASE_DEV].IMU_TOPIC.c_str());
    printf("[Node] Sub IMU Topic   d1.0: %s\n", DEV_CONFIGS[EE_DEV  ].IMU_TOPIC.c_str());
    printf("[Node] Sub Image Topic d0.0: %s\n", DEV_CONFIGS[BASE_DEV].IMAGE_TOPICS[0].c_str());
    printf("[Node] Sub Image Topic d1.0: %s\n", DEV_CONFIGS[EE_DEV  ].IMAGE_TOPICS[0].c_str());
    
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    
#if (FEATURE_ENABLE_PT_CLOUD_SUPPORT) // NOTE: point cloud need to be implemented per device
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 100, feature_callback);
#endif
#if (FEATURE_ENABLE_STEREO_SUPPORT) // NOTE: stereo need to be implemented per device
    printf("Image Topic 1: %s\n", DEV_CONFIGS[BASE_DEV].IMAGE_TOPICS[1].c_str());
    ros::Subscriber sub_img1 = n.subscribe(DEV_CONFIGS[BASE_DEV].IMAGE_TOPICS[1], 30, img1_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback); // UNUSED
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback); // UNUSED
#endif

    std::thread img_sync_thread{sync_process_IMG};
    
    ros::spin();
    img_sync_thread.join();
    ////////// ENDS HERE //////////////////////////////////

    return 0;
}
