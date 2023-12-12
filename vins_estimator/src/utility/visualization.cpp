/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
// #include "eigen_conversions/eigen_msg.h" // instead we can use MsgToEigen directly

using namespace ros;
using namespace Eigen;

ros::Publisher pub_key_poses[MAX_NUM_DEVICES];
ros::Publisher pub_camera_pose[MAX_NUM_DEVICES];
ros::Publisher pub_camera_pose_visual[MAX_NUM_DEVICES];


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
#endif

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
ros::Publisher pub_arm_odometry[MAX_NUM_DEVICES];
ros::Publisher pub_arm_visual;
#endif

typedef struct{
    double              t;
    Eigen::Vector3d     p;
    Eigen::Quaterniond  q;
} pose_data_t;

typedef struct{
#if (FEATURE_ENABLE_VICON_SUPPORT)
    // GT:
    nav_msgs::Path      vicon_path; // buffer
    std::mutex          vicon_guard;
#   if (FEATURE_ENABLE_VICON_ZEROING_SUPPORT)
    Eigen::Vector3d     vicon_p0;
    Eigen::Matrix3d     vicon_R0;
    bool                vicon_inited;
#   endif
#   if (FEATURE_ENABLE_ARM_VICON_SUPPORT)
    geometry_msgs::Pose     latest_vicon_pose;
#   endif // (FEATURE_ENABLE_ARM_VICON_SUPPORT)
#endif
#if (FEATURE_TRACKING_IMAGE_SUPPORT)
    // Track image:
    sensor_msgs::ImagePtr   imgTrackMsg;
    std::mutex              track_guard;
#endif
#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
    queue<pose_data_t>          arm_vicon_odom_buf;
    nav_msgs::Path              arm_path;
    std::mutex                  arm_guard;
#endif
    // visual:
    nav_msgs::Path              path;
    nav_msgs::Odometry          odometry;
    double                      latest_path_time;
    bool                        if_odometry;

    visualization_msgs::Marker  key_poses;
    bool                        if_key_poses = false;

    nav_msgs::Odometry          camerapose_odometry;
    CameraPoseVisualization     cameraposevisual = CameraPoseVisualization(1, 0, 0, 1);
    bool                        if_camerapose_odometry = false;

    sensor_msgs::PointCloud     point_cloud;
    sensor_msgs::PointCloud     margin_cloud;

    nav_msgs::Odometry          TF_odometry;
    bool                        if_TF_odometry = false;

    std::mutex                  guard;
} VisualizationBuffer_t;
VisualizationBuffer_t m_buf[MAX_NUM_DEVICES];


static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n, const int N_DEVICES)
{
    // TODO: feature flags for run-time instead of visualizations

    for (int i = 0; i < N_DEVICES; i++)
    {
        // TODO: flag unnecessary pubs for run-time performance
        // Data Publish (Sub by LoopFusion, Immediately):
        pub_odometry[i]          = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_ODOMETRY_E):(TOPIC_ODOMETRY_B)), PUBLISHER_BUFFER_SIZE);
        pub_keyframe_pose[i]     = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_KEYFRAME_POSE_E):(TOPIC_KEYFRAME_POSE_B)), PUBLISHER_BUFFER_SIZE);
        pub_extrinsic[i]         = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_EXTRINSIC_E):(TOPIC_EXTRINSIC_B)), PUBLISHER_BUFFER_SIZE);
        pub_keyframe_point[i]    = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_KEYFRAME_POINT_E):(TOPIC_KEYFRAME_POINT_B)), PUBLISHER_BUFFER_SIZE);
        
        // Data Publish:
        pub_camera_pose[i]       = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_CAMERA_POSE_E):(TOPIC_CAMERA_POSE_B)), PUBLISHER_BUFFER_SIZE);
#if (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
        pub_latest_odometry[i]   = n.advertise<nav_msgs::Odometry>(((i)?(TOPIC_IMU_PROPAGATE_E):(TOPIC_IMU_PROPAGATE_B)), PUBLISHER_BUFFER_SIZE); // TODO: not being sub
#endif 
        pub_key_poses[i]         = n.advertise<visualization_msgs::Marker>(((i)?(TOPIC_KEY_POSES_E):(TOPIC_KEY_POSES_B)), PUBLISHER_BUFFER_SIZE);

        // Rviz Visuals:
        pub_camera_pose_visual[i]= n.advertise<visualization_msgs::MarkerArray>(((i)?(TOPIC_CAMERA_POSE_VISUAL_E):(TOPIC_CAMERA_POSE_VISUAL_B)), PUBLISHER_BUFFER_SIZE);
        pub_path[i]              = n.advertise<nav_msgs::Path>(((i)?(TOPIC_PATH_E):(TOPIC_PATH_B)), PUBLISHER_BUFFER_SIZE);
        pub_image_track[i]       = n.advertise<sensor_msgs::Image>(((i)?(TOPIC_IMAGE_TRACK_E):(TOPIC_IMAGE_TRACK_B)), PUBLISHER_BUFFER_SIZE);
        pub_point_cloud[i]       = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_POINT_CLOUD_E):(TOPIC_POINT_CLOUD_B)), PUBLISHER_BUFFER_SIZE);
        pub_margin_cloud[i]      = n.advertise<sensor_msgs::PointCloud>(((i)?(TOPIC_MARGIN_CLOUD_E):(TOPIC_MARGIN_CLOUD_B)), PUBLISHER_BUFFER_SIZE); // (sub by loopfusion for visual)
        
        // vicon rviz visuals: 
#if (FEATURE_ENABLE_VICON_SUPPORT)
        pub_vicon[i]            = n.advertise<nav_msgs::Path>(((i)?(TOPIC_VICON_PATH_E):(TOPIC_VICON_PATH_B)), PUBLISHER_BUFFER_SIZE);
#endif

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
        pub_arm_odometry[i]     = n.advertise<nav_msgs::Path>(((i)?(TOPIC_ARM_PATH_E):(TOPIC_ARM_PATH_B)), PUBLISHER_BUFFER_SIZE);
#endif
        // placeholders
        m_buf[i].cameraposevisual.setScale(0.1);
        m_buf[i].cameraposevisual.setLineWidth(0.01);
    }

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
        pub_arm_visual           = n.advertise<nav_msgs::Path>(TOPIC_ARM_PATH_VIZ, PUBLISHER_BUFFER_SIZE);
#endif

}


void visualization_guard_lock(const Estimator &estimator)
{
    m_buf[estimator.pCfg->DEVICE_ID].guard.lock();
}
void visualization_guard_unlock(const Estimator &estimator)
{
    m_buf[estimator.pCfg->DEVICE_ID].guard.unlock();
}

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
void queue_ArmOdometry_safe(const double t, const Lie::SO3& R, const Lie::R3& p, const int device_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);

    Eigen::Quaterniond q = Eigen::Quaterniond(R);

    // apply transformation to pose:
    geometry_msgs::Pose pose;
    pose.position.x = p(0);
    pose.position.y = p(1);
    pose.position.z = p(2);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // push back
    geometry_msgs::PoseStamped pose_stamped;
    // PRINT_DEBUG("vicon odom ok");
    pose_stamped.header = header;
    pose_stamped.pose = pose;
    m_buf[device_id].arm_guard.lock();
    m_buf[device_id].arm_path.header = header;
    m_buf[device_id].arm_path.header.frame_id = "world";
    m_buf[device_id].arm_path.poses.push_back(pose_stamped);
    m_buf[device_id].arm_guard.unlock();
}
void pubArmOdometry_safe(const int device_id)
{
    // publish the path:
    m_buf[device_id].arm_guard.lock();
    pub_arm_odometry[device_id].publish(m_buf[device_id].arm_path);
    m_buf[device_id].arm_guard.unlock();
}
#endif

#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
void pubArmModel_safe(std::shared_ptr<ArmModel> model)
{
    // publish the path:
    if (model)
    {
        // publish:
        model->pub_Marker(pub_arm_visual);
    }
}
#endif

#if (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
void pubLatestOdometry_immediately(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const double t, const int device_id)
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
#endif //(FEATURE_ROS_PUBLISH_IMU_PROPAGATION)

#if (FEATURE_TRACKING_IMAGE_SUPPORT)
void queue_TrackImage_safe(const cv::Mat &imgTrack, const double t, const int device_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    // queue image:
    m_buf[device_id].track_guard.lock();
    m_buf[device_id].imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    m_buf[device_id].track_guard.unlock();
}
void pubTrackImage_safe(const int device_id)
{
    m_buf[device_id].track_guard.lock();
    if (m_buf[device_id].imgTrackMsg) // in case of no images
    {
        pub_image_track[device_id].publish(m_buf[device_id].imgTrackMsg);
    }
    m_buf[device_id].track_guard.unlock();
}
#endif

#if (FEATURE_ENABLE_STATISTICS_LOGGING)
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
        ROS_INFO("[%d] td %f", estimator.pCfg->DEVICE_ID, estimator.td);
}
#endif

void queue_Odometry_unsafe(const Estimator &estimator, const std_msgs::Header &header)
{
    const int device_id = estimator.pCfg->DEVICE_ID;
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Lie::Qd tmp_Q; Lie::R3 tmp_P; Lie::R3 tmp_V;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        tmp_P = estimator.Ps[WINDOW_SIZE];
        tmp_V = estimator.Vs[WINDOW_SIZE];
        odometry.pose.pose.position.x = tmp_P.x();
        odometry.pose.pose.position.y = tmp_P.y();
        odometry.pose.pose.position.z = tmp_P.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = tmp_V.x();
        odometry.twist.twist.linear.y = tmp_V.y();
        odometry.twist.twist.linear.z = tmp_V.z();
        m_buf[device_id].odometry = odometry;
        m_buf[device_id].if_odometry = true;
        // pub_odometry[device_id].publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;

        // queue for trajectory visualization
        m_buf[device_id].path.header = header;
        m_buf[device_id].path.header.frame_id = "world";
        m_buf[device_id].path.poses.push_back(pose_stamped);
        m_buf[device_id].latest_path_time = header.stamp.toSec(); // atomic

#if(FEATURE_VIZ_ROSOUT_ODOMETRY_SUPPORT)
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
#endif
    }
    else
    {
        m_buf[device_id].if_odometry = false;
    }
}

void pubOdometry_safe(const int device_id)
{
    m_buf[device_id].guard.lock();
    if (m_buf[device_id].if_odometry)
    {
        pub_odometry[device_id].publish(m_buf[device_id].odometry);
    }
    m_buf[device_id].guard.unlock();
}

void pubOdometryPath_safe(const int device_id)
{
    m_buf[device_id].guard.lock();
    pub_path[device_id].publish(m_buf[device_id].path);
    m_buf[device_id].guard.unlock();
}

#if (FEATURE_ENABLE_VICON_SUPPORT)
void queue_ViconOdometry_safe(const Vector7d_t &vicon_msg, const double t, const int device_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t); // copy the timestamp, TODO: should we do a sync?
    
    geometry_msgs::Pose pose;

#if (!FEATURE_ENABLE_VICON_ZEROING_SUPPORT && !FEATURE_ENABLE_VICON_ZEROING_WRT_BASE_SUPPORT)    
    pose.position.x = vicon_msg(0);
    pose.position.y = vicon_msg(1);
    pose.position.z = vicon_msg(2);
    pose.orientation.x = vicon_msg(4);
    pose.orientation.y = vicon_msg(5);
    pose.orientation.z = vicon_msg(6);
    pose.orientation.w = vicon_msg(3);
#else

    // zeroing the pose
    Eigen::Vector3d     p;
    Eigen::Quaterniond  q;
    Eigen::Matrix3d     R;
    
    p = Eigen::Vector3d(vicon_msg.block<3,1>(0,0));
    q = Eigen::Quaterniond(vicon_msg(3), vicon_msg(4), vicon_msg(5), vicon_msg(6)); // w,x,y,z

    R = q.toRotationMatrix();

    // Capture & Reset the first pose to the origin
    bool init_vicon = (m_buf[device_id].vicon_path.poses.size() == 0);

    // capture:
#   if (FEATURE_ENABLE_VICON_ZEROING_WRT_BASE_SUPPORT)
    if (init_vicon & (device_id == BASE_DEV)) // only init on base device;
    {
        // SO3 --proj--> SO2:
        // Extract the angle about the z-axis (ensure SO2 with z axis aligned up)
        // double theta = Utility::R2y_rad(R);
        // R = Utility::y2R_rad(theta);
        // T0_inv : for offset correction
        // X---- NAh

        m_buf[device_id].vicon_R0 = R.transpose();
        m_buf[device_id].vicon_p0 = - p;
        m_buf[device_id].vicon_inited = true;
        // init EE from Base config:
        m_buf[EE_DEV].vicon_inited = true;
        m_buf[EE_DEV].vicon_R0 = m_buf[BASE_DEV].vicon_R0;
        m_buf[EE_DEV].vicon_p0 = m_buf[BASE_DEV].vicon_p0;
        // PRINT_DEBUG("vicon_R0 = \n%s", Lie::to_string(m_buf[BASE_DEV].vicon_R0).c_str());
        // PRINT_DEBUG("vicon_p0 = %s", Lie::to_string(m_buf[BASE_DEV].vicon_p0).c_str());
    }    
#   else
    if (init_vicon)
    {
        const Lie::SO3 R_corr_c2w(
            (Lie::SO3() <<  1,  0,  0,
                            0,  0, -1,
                            0,  1,  0).finished()
        );// convert from world axis to camera axis 
        
        if (device_id == EE_DEV)
        {
            R = R * R_corr_c2w;
        }

#   if (FEATURE_ENABLE_VICON_ZEROING_ENFORCE_SO2)
        // enforce SO2 correction from base? (not SE2)
        double yaw = Utility::R2y_rad(R);
        R = Utility::y2R_rad(yaw); // to avoid pitch/roll bias
#   endif //(FEATURE_ENABLE_VICON_ZEROING_ENFORCE_SO2)

        m_buf[device_id].vicon_R0 = R.transpose();
        m_buf[device_id].vicon_p0 = - p;
        m_buf[device_id].vicon_inited = true;

#   if (FEATURE_ENABLE_VICON_ZEROING_ORIENTATION_WRT_BASE_SUPPORT)
        m_buf[EE_DEV].vicon_R0 = m_buf[BASE_DEV].vicon_R0; // EE pose should be based on base pose, compensate translation
#   endif //(FEATURE_ENABLE_VICON_ZEROING_ORIENTATION_WRT_BASE_SUPPORT)
    }
#   endif // (FEATURE_ENABLE_VICON_ZEROING_WRT_BASE_SUPPORT)

    // apply zeroing correction:
    {
        // offset position wrt initial position:
        p = m_buf[device_id].vicon_p0 + p;
        // correction on orientation:
        p = m_buf[device_id].vicon_R0 * p;
        R = m_buf[device_id].vicon_R0 * R;
        // to quaternion:
        q = Eigen::Quaterniond(R);
    }

    // apply transformation to pose:
    pose.position.x = p(0);
    pose.position.y = p(1);
    pose.position.z = p(2);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
#endif
    // push back
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;
    m_buf[device_id].vicon_guard.lock();
    m_buf[device_id].vicon_path.header = header;
    m_buf[device_id].vicon_path.header.frame_id = "world";
#if (FEATURE_ENABLE_ARM_VICON_SUPPORT)
    m_buf[device_id].latest_vicon_pose = pose;
#endif //(FEATURE_ENABLE_ARM_VICON_SUPPORT)
#if (FEATURE_ENABLE_VICON_ZEROING_SUPPORT)
    if (m_buf[device_id].vicon_inited) // publish only once it is initialized
        m_buf[device_id].vicon_path.poses.push_back(pose_stamped);
#else
    m_buf[device_id].vicon_path.poses.push_back(pose_stamped);
#endif //(!FEATURE_ENABLE_VICON_ZEROING_SUPPORT)
    m_buf[device_id].vicon_guard.unlock();
}

#if (FEATURE_ENABLE_ARM_VICON_SUPPORT)
void getLatestViconPose_safe(geometry_msgs::Pose& pose, const int device_id)
{
    m_buf[device_id].vicon_guard.lock();
    pose = m_buf[device_id].latest_vicon_pose;
    m_buf[device_id].vicon_guard.unlock();
}
#endif //(FEATURE_ENABLE_ARM_VICON_SUPPORT)

void pubViconOdometryPath_safe(const int device_id)
{
    // publish the path:
    m_buf[device_id].vicon_guard.lock();
    pub_vicon[device_id].publish(m_buf[device_id].vicon_path);
    m_buf[device_id].vicon_guard.unlock();
}
#endif // (FEATURE_ENABLE_VICON_SUPPORT)

void queue_KeyPoses_unsafe(const Estimator &estimator, const std_msgs::Header &header)
{
    const int device_id = estimator.pCfg->DEVICE_ID;
    if (estimator.key_poses.size() == 0)
    {
        m_buf[device_id].if_key_poses = false;
    }
    else
    {
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
        m_buf[device_id].key_poses = key_poses;
        m_buf[device_id].if_key_poses = true;
    }
}
void pubKeyPoses_safe(const size_t device_id)
{
    m_buf[device_id].guard.lock();
    if (m_buf[device_id].if_key_poses)
    {
        pub_key_poses[device_id].publish(m_buf[device_id].key_poses);
    }
    m_buf[device_id].guard.unlock();
}

void queue_CameraPose_unsafe(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1; //NOTE; does not matter, as its visual only
    const int device_id = estimator.pCfg->DEVICE_ID;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);
        // pose
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

        // pose:
        m_buf[device_id].camerapose_odometry = odometry;
        // pose visual
        m_buf[device_id].cameraposevisual.reset();
        m_buf[device_id].cameraposevisual.add_pose(P, R);
        
#if (FEATURE_ENABLE_STEREO_SUPPORT) // for stereo camera pose
        if(estimator.pCfg->STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            m_buf[device_id].cameraposevisual.add_pose(P, R);
        }
#endif
        m_buf[device_id].if_camerapose_odometry = true;
    }
    else
    {
        m_buf[device_id].if_camerapose_odometry = false;
    }
}

void pubCameraPose_safe(const size_t device_id)
{
    m_buf[device_id].guard.lock();
    if (m_buf[device_id].if_camerapose_odometry)
    {
        pub_camera_pose[device_id].publish(m_buf[device_id].camerapose_odometry);
        m_buf[device_id].cameraposevisual.publish_by(pub_camera_pose_visual[device_id], m_buf[device_id].camerapose_odometry.header);
    }
    m_buf[device_id].guard.unlock();
}


void queue_PointCloud_unsafe(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;
    const int device_id = estimator.pCfg->DEVICE_ID;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < NTH_FRAME))
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
    m_buf[device_id].point_cloud = point_cloud;
    // pub_point_cloud[device_id].publish(point_cloud);

    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < NTH_FRAME))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && used_num <= 2 
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
    m_buf[device_id].margin_cloud = margin_cloud;
    // pub_margin_cloud[device_id].publish(margin_cloud);
}

void pubPointClouds_safe(const size_t device_id)
{
    m_buf[device_id].guard.lock();
    pub_point_cloud[device_id].publish(m_buf[device_id].point_cloud);
    pub_margin_cloud[device_id].publish(m_buf[device_id].margin_cloud);
    m_buf[device_id].guard.unlock();
}

void queue_TF_unsafe(const Estimator &estimator, const std_msgs::Header &header)
{
    const int device_id = estimator.pCfg->DEVICE_ID;
    if( estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {

#if (FEATURE_BROADCAST_ESTIMATOR_TF)
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
#endif //(FEATURE_BROADCAST_ESTIMATOR_TF)
        
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

        m_buf[device_id].TF_odometry = odometry;    
        m_buf[device_id].if_TF_odometry = true;    
        // pub_extrinsic[device_id].publish(odometry);
    }
    else
    {

        m_buf[device_id].if_TF_odometry = false;    
    }
}

void pubExtrinsic_TF_safe(const size_t device_id)
{
    m_buf[device_id].guard.lock();
    if (m_buf[device_id].if_TF_odometry)
    {
        pub_extrinsic[device_id].publish(m_buf[device_id].TF_odometry);
    }
    m_buf[device_id].guard.unlock();
}

void pubKeyframe_Odometry_and_Points_immediately(const Estimator &estimator)
{
    const int device_id = estimator.pCfg->DEVICE_ID;
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        //Vector3d P = estimator.Ps[NTH_FRAME] + estimator.Rs[NTH_FRAME] * estimator.tic[0];
        Vector3d P = estimator.Ps[NTH_FRAME];
        Quaterniond R = Quaterniond(estimator.Rs[NTH_FRAME]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[NTH_FRAME]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());
        pub_keyframe_pose[device_id].publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[NTH_FRAME]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < NTH_FRAME && it_per_id.start_frame + frame_size - 1 >= NTH_FRAME && it_per_id.solve_flag == 1)
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

                int imu_j = NTH_FRAME - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point[device_id].publish(point_cloud);
    }
}