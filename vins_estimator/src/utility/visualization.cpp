/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

using namespace ros;
using namespace Eigen;

ros::Publisher pub_key_poses[MAX_NUM_DEVICES];
ros::Publisher pub_camera_pose[MAX_NUM_DEVICES];
ros::Publisher pub_camera_pose_visual[MAX_NUM_DEVICES];

nav_msgs::Path path[MAX_NUM_DEVICES]; // buffer

ros::Publisher pub_keyframe_pose[MAX_NUM_DEVICES];
ros::Publisher pub_keyframe_point[MAX_NUM_DEVICES];
ros::Publisher pub_extrinsic[MAX_NUM_DEVICES];

ros::Publisher pub_point_cloud[MAX_NUM_DEVICES];
ros::Publisher pub_margin_cloud[MAX_NUM_DEVICES];

ros::Publisher pub_path[MAX_NUM_DEVICES];
ros::Publisher pub_odometry[MAX_NUM_DEVICES];

ros::Publisher pub_latest_odometry[MAX_NUM_DEVICES];
ros::Publisher pub_image_track[MAX_NUM_DEVICES];

#if (FEATURE_ENABLE_VICON_SUPPORT)
ros::Publisher pub_vicon[MAX_NUM_DEVICES];
nav_msgs::Path vicon_path[MAX_NUM_DEVICES]; // buffer
#endif

CameraPoseVisualization cameraposevisual[MAX_NUM_DEVICES]={
    CameraPoseVisualization(1, 0, 0, 1), CameraPoseVisualization(1, 0, 0, 1)
};
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n)
{
    // TODO: feature flags for run-time instead of visualizations

    for (int i = 0; i < N_DEVICES; i++)
    {
        // Data Publish:
        pub_camera_pose[i]       = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_CAMERA_POSE_E):(TOPIC_CAMERA_POSE_B)), PUBLISHER_BUFFER_SIZE);
        pub_odometry[i]          = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_ODOMETRY_E):(TOPIC_ODOMETRY_B)), PUBLISHER_BUFFER_SIZE);
        pub_latest_odometry[i]   = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_IMU_PROPAGATE_E):(TOPIC_IMU_PROPAGATE_B)), PUBLISHER_BUFFER_SIZE);
        pub_key_poses[i]         = n.advertise<visualization_msgs::Marker>(((i)?(TOPIC_KEY_POSES_E):(TOPIC_KEY_POSES_B)), PUBLISHER_BUFFER_SIZE);
        pub_keyframe_point[i]    = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_KEYFRAME_POINT_E):(TOPIC_KEYFRAME_POINT_B)), PUBLISHER_BUFFER_SIZE);
        pub_keyframe_pose[i]     = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_KEYFRAME_POSE_E):(TOPIC_KEYFRAME_POSE_B)), PUBLISHER_BUFFER_SIZE);
        pub_extrinsic[i]         = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_EXTRINSIC_E):(TOPIC_EXTRINSIC_B)), PUBLISHER_BUFFER_SIZE);

        // Rviz Visuals:
        pub_camera_pose_visual[i]= n.advertise<visualization_msgs::MarkerArray>(((i)?(TOPIC_CAMERA_POSE_VISUAL_E):(TOPIC_CAMERA_POSE_VISUAL_B)), PUBLISHER_BUFFER_SIZE);
        pub_path[i]              = n.advertise<nav_msgs::Path>(((i)?(TOPIC_PATH_E):(TOPIC_PATH_B)), PUBLISHER_BUFFER_SIZE);
        pub_image_track[i]       = n.advertise<sensor_msgs::Image>(((i)?(TOPIC_IMAGE_TRACK_E):(TOPIC_IMAGE_TRACK_B)), PUBLISHER_BUFFER_SIZE);
        pub_point_cloud[i]       = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_POINT_CLOUD_E):(TOPIC_POINT_CLOUD_B)), PUBLISHER_BUFFER_SIZE);
        pub_margin_cloud[i]      = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_MARGIN_CLOUD_E):(TOPIC_MARGIN_CLOUD_B)), PUBLISHER_BUFFER_SIZE);
        
        // vicon rviz visuals: 
#if (FEATURE_ENABLE_VICON_SUPPORT)
        pub_vicon[i] = n.advertise<nav_msgs::Path>(((i)?(TOPIC_VICON_PATH_E):(TOPIC_VICON_PATH_B)), PUBLISHER_BUFFER_SIZE);
#endif

        // placeholders
        cameraposevisual[i].setScale(0.1);
        cameraposevisual[i].setLineWidth(0.01);
    }

}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const double t, const int device_id)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry[device_id].publish(odometry);
}

void pubTrackImage(const cv::Mat &imgTrack, const double t, const int device_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track[device_id].publish(imgTrackMsg);
}


void printStatistics(const Estimator &estimator, const double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (estimator.pCfg->ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(estimator.pCfg->EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < estimator.pCfg->NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (estimator.pCfg->ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int device_id = estimator.pCfg->DEVICE_ID;
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        //TODO: opt, static cast
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry[device_id].publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path[device_id].header = header;
        path[device_id].header.frame_id = "world";
        path[device_id].poses.push_back(pose_stamped);
        pub_path[device_id].publish(path[device_id]);

        // write result to file
        ofstream foutC(estimator.pCfg->VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

#if (FEATURE_ENABLE_VICON_SUPPORT)
void pubViconOdometry(const geometry_msgs::TransformStampedConstPtr &transform_msg, const int device_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = transform_msg->header.stamp; // copy the timestamp, TODO: should we do a sync?
    
    geometry_msgs::Pose pose;
    //TODO: opt, static cast
    pose.position.x = transform_msg->transform.translation.x;
    pose.position.y = transform_msg->transform.translation.y;
    pose.position.z = transform_msg->transform.translation.z;
    pose.orientation = transform_msg->transform.rotation;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;
    vicon_path[device_id].header = header;
    vicon_path[device_id].header.frame_id = "world";
    vicon_path[device_id].poses.push_back(pose_stamped);
    // publish the path:
    pub_vicon[device_id].publish(vicon_path[device_id]);
}
#endif

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses[estimator.pCfg->DEVICE_ID].publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose[estimator.pCfg->DEVICE_ID].publish(odometry);

        cameraposevisual[estimator.pCfg->DEVICE_ID].reset();
        cameraposevisual[estimator.pCfg->DEVICE_ID].add_pose(P, R);
        
#if (FEATURE_ENABLE_STEREO_SUPPORT) // for stereo camera pose
        if(estimator.pCfg->STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
#endif
        cameraposevisual[estimator.pCfg->DEVICE_ID].publish_by(pub_camera_pose_visual[estimator.pCfg->DEVICE_ID], odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;


    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud[estimator.pCfg->DEVICE_ID].publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud[estimator.pCfg->DEVICE_ID].publish(margin_cloud);
}


void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic[estimator.pCfg->DEVICE_ID].publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose[estimator.pCfg->DEVICE_ID].publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point[estimator.pCfg->DEVICE_ID].publish(point_cloud);
    }
}