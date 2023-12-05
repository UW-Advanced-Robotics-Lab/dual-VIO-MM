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

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
using namespace std;
// ----------------------------------------------------------------
// : Definitions:
// ----------------------------------------------------------------
typedef struct{
    // buffer:
    queue<sensor_msgs::ImageConstPtr>       image;
    queue<sensor_msgs::PointCloudConstPtr>  point;
    queue<nav_msgs::Odometry::ConstPtr>     pose;
    queue<Eigen::Vector3d>                  odometry;
    std::mutex                              guard;
} NodeBuffer_t;

enum POSE_GRAPH_STATUS_E{
    POSE_GRAPH_STATUS_UNKNOWN          = 0,
    POSE_GRAPH_STATUS_INITED           = (1<<0),
    POSE_GRAPH_STATUS_PREVIOUS_LOADED  = (1<<1),
    POSE_GRAPH_STATUS_NOTHING_TO_LOAD  = (1<<2),
    POSE_GRAPH_STATUS_PROCESS_RUNNING  = (1<<3),
};

typedef struct{
    int     frame_index            = 0;
    int     sequence               = 1;
    double  last_image_time        = -1;
    uint8_t status                 = POSE_GRAPH_STATUS_UNKNOWN;
    CameraPoseVisualization cameraposevisual = CameraPoseVisualization(1, 0, 0, 1);
    Eigen::Vector3d         last_t           = Eigen::Vector3d(-100, -100, -100);
    PoseGraph*              posegraph;
    std::mutex              guard;
} NodeData_t;

// ----------------------------------------------------------------
// : Data Placeholder:
// ----------------------------------------------------------------
// parameters:
int              N_DEVICES;
DeviceConfig_t   DEV_CONFIGS[MAX_NUM_DEVICES];
LoopDevice_t     LoopDevices[MAX_NUM_DEVICES];

// private:
NodeBuffer_t     m_buffer[MAX_NUM_DEVICES];
NodeData_t       m_data[MAX_NUM_DEVICES];
  
ros::Publisher   pub_match_img[MAX_NUM_CAMERAS];
ros::Publisher   pub_camera_pose_visual[MAX_NUM_CAMERAS];
ros::Publisher   pub_odometry_rect[MAX_NUM_CAMERAS];
ros::Publisher   pub_point_cloud[MAX_NUM_CAMERAS];
ros::Publisher   pub_margin_cloud[MAX_NUM_CAMERAS];

// ----------------------------------------------------------------
// : Private Functions :
// ----------------------------------------------------------------

void new_sequence(const int device_id)
{
    NodeData_t * const data = & m_data[device_id];
    NodeBuffer_t * const buf  = & m_buffer[device_id];

    PRINT_INFO("new sequence\n");
    data->sequence++;
    PRINT_INFO("sequence cnt %d \n", data->sequence);
    if (data->sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences??");
        ROS_BREAK();
    }
    data->posegraph->posegraph_visualization->reset();
    data->posegraph->publish();
    // Empty buffers: TODO: see if this quick swap performs better
    // NodeBuffer_t empty;
    buf->guard.lock();
    // std::swap(buf->image, empty.image);
    // std::swap(buf->point, empty.point);
    // std::swap(buf->pose, empty.pose);
    // std::swap(buf->odometry, empty.odometry);
    while(!buf->image.empty())       buf->image.pop();
    while(!buf->point.empty())       buf->point.pop();
    while(!buf->pose.empty())        buf->pose.pop();
    while(!buf->odometry.empty())    buf->odometry.pop();
    buf->guard.unlock();
}

void image_buf_callback(const sensor_msgs::ImageConstPtr &image_msg,  const int device_id)
{
    NodeData_t * const data = & m_data[device_id];
    NodeBuffer_t * const buf  = & m_buffer[device_id];
    //ROS_INFO("image_buf_callback!");
    // cache:
    buf->guard.lock();
    buf->image.push(image_msg);
    buf->guard.unlock();
    //PRINT_INFO(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream:
    // buff:
    data->guard.lock();
    double t_last = data->last_image_time;
    data->guard.unlock();
    double t_curr = image_msg->header.stamp.toSec();
    if (t_last == -1)
    {
        t_last = t_curr;
    }
    else if (t_curr - t_last > HYPERPARAMS_NEW_IMAGE_TIME_TOLERANCE_MIN || t_curr < t_last)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence(device_id); 
    }
    // cache:
    data->guard.lock();
    data->last_image_time = t_curr;
    data->guard.unlock();
}

void point_buf_vis_callback(const sensor_msgs::PointCloudConstPtr &point_msg,  const int device_id)
{
    NodeBuffer_t * const buf  = & m_buffer[device_id];
    //ROS_INFO("point_buf_vis_callback!");
    buf->guard.lock();
    buf->point.push(point_msg);
    buf->guard.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        PRINTF("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
#if (FEATURE_ENABLE_VISUAL_POINT_CLOUD_SUPPORT)
    NodeData_t * const data = & m_data[device_id];
    Eigen::Vector3d t_drift;
    Eigen::Matrix3d r_drift;
    data->guard.lock();
    r_drift = data->posegraph->r_drift;
    t_drift = data->posegraph->t_drift;
    data->guard.unlock();
    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud[device_id].publish(point_cloud);
#endif
}

#if (FEATURE_ENABLE_VISUAL_MARGIN_POINT_CLOUD_SUPPORT)
// only for visualization
void margin_ptcloud_vis_callback(const sensor_msgs::PointCloudConstPtr &point_msg, const int device_id)
{
    // fetch:
    Eigen::Vector3d t_drift;
    Eigen::Matrix3d r_drift;
    NodeData_t * const data = & m_data[device_id];
    data->guard.lock();
    r_drift = data->posegraph->r_drift;
    t_drift = data->posegraph->t_drift;
    data->guard.unlock();
    // process:
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    // publish:
    pub_margin_cloud[device_id].publish(point_cloud);
}
#endif

void pose_buf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int device_id)
{
    NodeBuffer_t * const buf  = & m_buffer[device_id];
    //ROS_INFO("pose_callback!");
    buf->guard.lock();
    buf->pose.push(pose_msg);
    buf->guard.unlock();
    /*
    PRINTF("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void vio_buf_pro_pub_callback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int device_id)
{
    // placeholder:
    Vector3d tic, w_t_vio, t_drift;
    Matrix3d qic, w_r_vio, r_drift;
    
    // fetch:
    NodeData_t * const data = & m_data[device_id];
    data->guard.lock();
    w_r_vio = data->posegraph->w_r_vio;
    w_t_vio = data->posegraph->w_t_vio;
    r_drift = data->posegraph->r_drift;
    t_drift = data->posegraph->t_drift;
    data->guard.unlock();
    
    // fetch:
    LoopDevice_t * const dev = & LoopDevices[device_id];
    dev->guard.lock();
    tic = dev->tic;
    qic = dev->qic;
    dev->guard.unlock();

    // process:
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    
    // rectify:
    vio_t = w_r_vio * vio_t + w_t_vio;
    vio_q = w_r_vio *  vio_q;

    vio_t = r_drift * vio_t + t_drift;
    vio_q = r_drift * vio_q;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();

    // process should not hold the lock:
    pub_odometry_rect[device_id].publish(odometry);

#if (FEATURE_ENABLE_VISUAL_CAMERA_POSE_SUPPORT)
    // visual only:
    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    // cache results:
    data->guard.lock();
    data->cameraposevisual.reset();
    data->cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    data->guard.unlock();

    // process should not hold the lock:
    data->cameraposevisual.publish_by(pub_camera_pose_visual[device_id], pose_msg->header);
#endif
}

void extrinsic_buf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int device_id)
{
    // process:
    Vector3d tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    Matrix3d qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    // cache:
    LoopDevice_t * const dev = & LoopDevices[device_id];
    dev->guard.lock();
    dev->tic = tic;
    dev->qic = qic;
    dev->guard.unlock();
}

void process(const int device_id)
{
    DeviceConfig_t * const cfg = & DEV_CONFIGS[device_id];
    NodeBuffer_t * const buf  = & m_buffer[device_id];
    NodeData_t * const data = & m_data[device_id];
    LoopDevice_t * const dev = & LoopDevices[device_id];
    
    int first_count = 0;
    int skip_count = 0;

    while (true)
    {
        sensor_msgs::ImageConstPtr      image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr    pose_msg = NULL;
        
        // cache:
        // find out the messages with same time stamp, TODO: this acquires resources from the network, optimize???
        buf->guard.lock();
        if(!buf->image.empty() && !buf->point.empty() && !buf->pose.empty())
        {
            if (buf->image.front()->header.stamp.toSec() > buf->pose.front()->header.stamp.toSec())
            {
                buf->pose.pop();
                PRINT_WARN("throw pose at beginning\n");
            }
            else if (buf->image.front()->header.stamp.toSec() > buf->point.front()->header.stamp.toSec())
            {
                buf->point.pop();
                PRINT_WARN("throw point at beginning\n");
            }
            else if (buf->image.back()->header.stamp.toSec() >= buf->pose.front()->header.stamp.toSec() 
                && buf->point.back()->header.stamp.toSec() >= buf->pose.front()->header.stamp.toSec())
            {
                pose_msg = buf->pose.front();
                buf->pose.pop();
                while (!buf->pose.empty())
                    buf->pose.pop();
                while (buf->image.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    buf->image.pop();
                image_msg = buf->image.front();
                buf->image.pop();

                while (buf->point.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    buf->point.pop();
                point_msg = buf->point.front();
                buf->point.pop();
            }
        }
        buf->guard.unlock();

        // process:
        if (pose_msg != NULL)
        {
            //PRINTF(" pose time %f \n", pose_msg->header.stamp.toSec());
            //PRINTF(" point time %f \n", point_msg->header.stamp.toSec());
            //PRINTF(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few

            if (first_count < HYPERPARAMS_SKIP_FIRST_N_CNTS_IMAGES)
            {
                first_count ++;
                continue;
            }

            if (skip_count < cfg->SKIP_CNT)
            {
                skip_count ++;
                continue;
            }
            else
            {
                skip_count = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - data->last_t).norm() > cfg->SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //PRINTF("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }
                // fetch status:
                data->guard.lock();
                int frame_index = data->frame_index;
                int sequence = data->sequence;
                // data->guard.unlock();
                // create keyframe:
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence, cfg, dev);   
                // attach keyframe & update status:
                // data->guard.lock();
                data->status |= POSE_GRAPH_STATUS_PROCESS_RUNNING;
                data->posegraph->addKeyFrame(keyframe, 1);
                data->frame_index++;
                data->last_t = T;
                data->guard.unlock();
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

#if (FEATURE_ENABLE_KEYBOARD_INTERFACE)
void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            data->guard.lock();
            posegraph->savePoseGraph();
            data->guard.unlock();
            PRINT_WARN("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            PRINT_WARN("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_fusion");
    ros::NodeHandle n("~");
    
    if(argc != 2)
    {
        PRINT_ERROR("please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }
    
    const string config_file = argv[1];
    PRINT_INFO("config_file: %s\n", argv[1]);

    const std::string pkg_path = ros::package::getPath("loop_fusion");
    try
    {
        N_DEVICES = readParameters(config_file, pkg_path, DEV_CONFIGS, LoopDevices); // --> loading configuration
    }
    catch (const std::exception &)
    {
        PRINT_ERROR("Unable to load configuration");
        return 0;
    }

    //////////
    ros::Subscriber sub_image[MAX_NUM_DEVICES];
    ros::Subscriber sub_vio[MAX_NUM_DEVICES];
    ros::Subscriber sub_pose[MAX_NUM_DEVICES];
    ros::Subscriber sub_extrinsic[MAX_NUM_DEVICES];
    ros::Subscriber sub_point[MAX_NUM_DEVICES];

#if (FEATURE_ENABLE_VISUAL_MARGIN_POINT_CLOUD_SUPPORT)
    ros::Subscriber sub_margin_point[MAX_NUM_DEVICES];
#endif

    ////////// BEGIN HERE //////////////////////////////////
    bool if_old_config = (N_DEVICES == 0);
    N_DEVICES = if_old_config? 1:N_DEVICES; 
    for (int i = 0; i < N_DEVICES; i++)
    {
        DeviceConfig_t* const cfg = & (DEV_CONFIGS[i]);
        NodeData_t * const data = & m_data[i];
        LoopDevice_t * const dev = & LoopDevices[i];
        cfg->DEVICE_ID = i;
        
        std::string pre_ = "d"+ std::to_string(i) + "_";
        if (if_old_config){
            pre_ = ""; // no prefixes, backwards compatibility
        }
        PRINT_INFO("=== [Indexing %scameras ]", pre_.c_str());
        // reg:
        data->posegraph = new PoseGraph(cfg, dev);
        data->posegraph->registerPub(n);
        // set:
        cfg->VISUALIZATION_SHIFT_X = 0;
        cfg->VISUALIZATION_SHIFT_Y = 0;
        cfg->SKIP_CNT = 0;
        cfg->SKIP_DIS = 0;
        // config cam node:
        dev->m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cfg->CAM0_CALIB_PATH.c_str());
        // config visual:
        data->cameraposevisual.setScale(0.1);
        data->cameraposevisual.setLineWidth(0.01);
        // create output filesystem:
        std::ofstream fout(cfg->VINS_RESULT_PATH, std::ios::out);
        fout.close();
        // config posegraph:  
        data->posegraph->loadVocabulary(cfg->VOCAB_FILE_PATH);
        data->posegraph->setIMUFlag(cfg->USE_IMU);
        // load path:        
        if (cfg->LOAD_PREVIOUS_POSE_GRAPH)
        {
            ROS_WARN("load pose graph\n");
            data->guard.lock();
            data->posegraph->loadPoseGraph();
            data->guard.unlock();
            ROS_WARN("load pose graph finish\n");
            data->status |= POSE_GRAPH_STATUS_PREVIOUS_LOADED;
        }
        else
        {
            ROS_ERROR("no previous pose graph\n");
            data->status |= POSE_GRAPH_STATUS_NOTHING_TO_LOAD;
        }
        
        data->status |= POSE_GRAPH_STATUS_INITED;
        
        // create subscription:
        sub_image[i]    = n.subscribe<sensor_msgs::Image>(cfg->IMAGE_TOPIC, 
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(image_buf_callback, _1, i));
        sub_vio[i]      = n.subscribe<nav_msgs::Odometry>(
            ((i)?(SUB_TOPIC_ODOMETRY_E):(SUB_TOPIC_ODOMETRY_B)), 
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(vio_buf_pro_pub_callback, _1, i));
        sub_pose[i]     = n.subscribe<nav_msgs::Odometry>(
            ((i)?(SUB_TOPIC_KEYFRAME_POSE_E):(SUB_TOPIC_KEYFRAME_POSE_B)), 
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(pose_buf_callback, _1, i));
        sub_extrinsic[i]= n.subscribe<nav_msgs::Odometry>(
            ((i)?(SUB_TOPIC_EXTRINSIC_E):(SUB_TOPIC_EXTRINSIC_B)), 
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(extrinsic_buf_callback, _1, i));
        sub_point[i]    = n.subscribe<sensor_msgs::PointCloud>(
            ((i)?(SUB_TOPIC_KEYFRAME_POINT_E):(SUB_TOPIC_KEYFRAME_POINT_B)), 
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(point_buf_vis_callback, _1, i));
#if (FEATURE_ENABLE_VISUAL_MARGIN_POINT_CLOUD_SUPPORT)
        sub_margin_point[i] = n.subscribe<sensor_msgs::PointCloud>(
            ((i)?(SUB_TOPIC_MARGIN_CLOUD_E):(SUB_TOPIC_MARGIN_CLOUD_B)),
            SUBSCRIPTION_BUFFER_SIZE, boost::bind(margin_ptcloud_vis_callback, _1, i));
#endif 
        pub_match_img[i]          = n.advertise<sensor_msgs::Image>(
            ((i)?(TOPIC_MATCH_IMAGE_E):(TOPIC_MATCH_IMAGE_B)), 
            PUBLISHER_BUFFER_SIZE);
        pub_camera_pose_visual[i] = n.advertise<visualization_msgs::MarkerArray>(
            ((i)?(TOPIC_CAMERA_POSE_VISUAL_E):(TOPIC_CAMERA_POSE_VISUAL_B)), 
            PUBLISHER_BUFFER_SIZE);
        pub_point_cloud[i]        = n.advertise<sensor_msgs::PointCloud>(
            ((i)?(TOPIC_POINT_CLOUD_LOOP_RECT_E):(TOPIC_POINT_CLOUD_LOOP_RECT_B)), 
            PUBLISHER_BUFFER_SIZE);
        pub_margin_cloud[i]       = n.advertise<sensor_msgs::PointCloud>(
            ((i)?(TOPIC_MARGIN_CLOUD_LOOP_RECT_E):(TOPIC_MARGIN_CLOUD_LOOP_RECT_B)), 
            PUBLISHER_BUFFER_SIZE);
        pub_odometry_rect[i]      = n.advertise<nav_msgs::Odometry>(
            ((i)?(TOPIC_ODOMETRY_RECT_E):(TOPIC_ODOMETRY_RECT_B)), 
            PUBLISHER_BUFFER_SIZE);
        
        dev->p_match_image_publisher = & pub_match_img[i];
    }
    
    std::thread measurement_process_b = std::thread(process, BASE_DEV);
    std::thread measurement_process_e = std::thread(process, EE_DEV);

#if(FEATURE_ENABLE_KEYBOARD_INTERFACE)
    std::thread keyboard_command_process = std::thread(command);
#endif
    
    ros::spin();

    measurement_process_b.join();
    measurement_process_e.join();
    return 0;
}
