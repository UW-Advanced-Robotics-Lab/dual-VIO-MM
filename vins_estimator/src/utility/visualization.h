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
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include <fstream>
#include "../robot/Lie.h"
#include "../robot/ArmModel.h"

// extern ros::Publisher pub_odometry;
// extern ros::Publisher pub_path, pub_pose;
// extern ros::Publisher pub_cloud, pub_map;
// extern ros::Publisher pub_key_poses;
// extern ros::Publisher pub_ref_pose, pub_cur_pose;
// extern ros::Publisher pub_key;
// extern nav_msgs::Path path;
// extern ros::Publisher pub_pose_graph;
// extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n, const int N_DEVICES);

void visualization_guard_lock(const Estimator &estimator);
void visualization_guard_unlock(const Estimator &estimator);
#if (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
void pubLatestOdometry_immediately(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const double t, const int device_id);
#endif //(FEATURE_ROS_PUBLISH_IMU_PROPAGATION)

#if (FEATURE_TRACKING_IMAGE_SUPPORT)
void queue_TrackImage_safe(const cv::Mat &imgTrack, const double t, const int device_id);
void pubTrackImage_safe(const int device_id);
#endif
#if (FEATURE_ENABLE_STATISTICS_LOGGING)
void printStatistics(const Estimator &estimator, const double t);
#endif

void queue_Odometry_unsafe(const Estimator &estimator, const std_msgs::Header &header);
void pubOdometry_safe(const int device_id);
void pubOdometryPath_safe(const int device_id);

#if (FEATURE_ENABLE_VICON_SUPPORT)
// vicon:
void queue_ViconOdometry_safe(const double t, const int device_id, const Eigen::Quaterniond q, const Vector3d p);
void pubViconOdometryPath_safe(const int device_id);
#endif

void queue_KeyPoses_unsafe(const Estimator &estimator, const std_msgs::Header &header);
void pubKeyPoses_safe(const size_t device_id);

void queue_CameraPose_unsafe(const Estimator &estimator, const std_msgs::Header &header);
void pubCameraPose_safe(const size_t device_id);

void queue_PointCloud_unsafe(const Estimator &estimator, const std_msgs::Header &header);
void pubPointClouds_safe(const size_t device_id);

void queue_TF_unsafe(const Estimator &estimator, const std_msgs::Header &header);
void pubExtrinsic_TF_safe(const size_t device_id);

void pubKeyframe_Odometry_and_Points_immediately(const Estimator &estimator);

// void pubRelocalization(const Estimator &estimator);???
// void pubCar(const Estimator & estimator, const std_msgs::Header &header);

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
void queue_ArmOdometry_safe(const double t, const Lie::SO3& R, const Lie::R3& p, const int device_id);
void pubArmOdometry_safe(const int device_id);
#endif
#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
void pubArmModel_safe(std::shared_ptr<ArmModel> model);
#endif
