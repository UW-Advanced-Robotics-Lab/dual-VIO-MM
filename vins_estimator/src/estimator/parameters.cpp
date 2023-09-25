/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"
// ----------------------------------------------------------------
// : ARCHIVES :
// ----------------------------------------------------------------
// double INIT_DEPTH;
// double MIN_PARALLAX;
// double ACC_N, ACC_W;
// double GYR_N, GYR_W;

// std::vector<Eigen::Matrix3d> RIC;
// std::vector<Eigen::Vector3d> TIC;

// Eigen::Vector3d G{0.0, 0.0, 9.8};

// double BIAS_ACC_THRESHOLD;
// double BIAS_GYR_THRESHOLD;
// double SOLVER_TIME;
// int NUM_ITERATIONS;
// int ESTIMATE_EXTRINSIC;
// int ESTIMATE_TD;
// int ROLLING_SHUTTER;
// std::string EX_CALIB_RESULT_PATH;
// std::string VINS_RESULT_PATH;
// std::string OUTPUT_FOLDER;
// std::string IMU_TOPIC;
// int ROW, COL;
// double TD;
// int NUM_OF_CAM;
// int STEREO;
// int USE_IMU;
// int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt; // debug information
// std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
// std::string FISHEYE_MASK;
// std::vector<std::string> CAM_NAMES;
// int MAX_CNT;
// int MIN_DIST;
// double F_THRESHOLD;
// int SHOW_TRACK;
// int FLOW_BACK;

// ----------------------------------------------------------------
// : IMPLEMENTATION :
// ----------------------------------------------------------------
DeviceConfig_t      DEV_CONFIG;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> DEV_CONFIG.IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> DEV_CONFIG.IMAGE1_TOPIC;
    DEV_CONFIG.MAX_CNT        = fsSettings["max_cnt"];
    DEV_CONFIG.MIN_DIST       = fsSettings["min_dist"];
    DEV_CONFIG.F_THRESHOLD    = fsSettings["F_threshold"];
    DEV_CONFIG.SHOW_TRACK     = fsSettings["show_track"];
    DEV_CONFIG.FLOW_BACK      = fsSettings["flow_back"];

    DEV_CONFIG.MULTIPLE_THREAD = fsSettings["multiple_thread"];

    DEV_CONFIG.USE_IMU = fsSettings["imu"];
    printf("DEV_CONFIG.USE_IMU: %d\n", DEV_CONFIG.USE_IMU);
    if(DEV_CONFIG.USE_IMU)
    {
        fsSettings["imu_topic"] >> DEV_CONFIG.IMU_TOPIC;
        printf("DEV_CONFIG.IMU_TOPIC: %s\n", DEV_CONFIG.IMU_TOPIC.c_str());
        DEV_CONFIG.ACC_N = fsSettings["acc_n"];
        DEV_CONFIG.ACC_W = fsSettings["acc_w"];
        DEV_CONFIG.GYR_N = fsSettings["gyr_n"];
        DEV_CONFIG.GYR_W = fsSettings["gyr_w"];
        DEV_CONFIG.G.z() = fsSettings["g_norm"];
    }

    DEV_CONFIG.SOLVER_TIME = fsSettings["max_solver_time"];
    DEV_CONFIG.NUM_ITERATIONS = fsSettings["max_num_iterations"];
    DEV_CONFIG.MIN_PARALLAX = fsSettings["keyframe_parallax"];
    DEV_CONFIG.MIN_PARALLAX = DEV_CONFIG.MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> DEV_CONFIG.OUTPUT_FOLDER;
    DEV_CONFIG.VINS_RESULT_PATH = DEV_CONFIG.OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << DEV_CONFIG.VINS_RESULT_PATH << std::endl;
    std::ofstream fout(DEV_CONFIG.VINS_RESULT_PATH, std::ios::out);
    fout.close();

    DEV_CONFIG.ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (DEV_CONFIG.ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        DEV_CONFIG.RIC.push_back(Eigen::Matrix3d::Identity());
        DEV_CONFIG.TIC.push_back(Eigen::Vector3d::Zero());
        // DEV_CONFIG.RIC[0] = (Eigen::Matrix3d::Identity());
        // DEV_CONFIG.TIC[0] = (Eigen::Vector3d::Zero());
        DEV_CONFIG.EX_CALIB_RESULT_PATH = DEV_CONFIG.OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if (DEV_CONFIG.ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            DEV_CONFIG.EX_CALIB_RESULT_PATH = DEV_CONFIG.OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (DEV_CONFIG.ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        DEV_CONFIG.RIC.push_back(T.block<3, 3>(0, 0));
        DEV_CONFIG.TIC.push_back(T.block<3, 1>(0, 3));
        // DEV_CONFIG.RIC[0] = (T.block<3, 3>(0, 0));
        // DEV_CONFIG.TIC[0] = (T.block<3, 1>(0, 3));
    } 
    
    DEV_CONFIG.NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", DEV_CONFIG.NUM_OF_CAM);

    if(DEV_CONFIG.NUM_OF_CAM != 1 && DEV_CONFIG.NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    DEV_CONFIG.CAM_NAMES.push_back(cam0Path);
    printf("%s cam0 path\n", cam0Path.c_str() );
    // DEV_CONFIG.CAM_NAMES[0] = (cam0Path);

    if(DEV_CONFIG.NUM_OF_CAM == 2)
    {
        DEV_CONFIG.STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        printf("%s cam1 path\n", cam1Path.c_str() );
        DEV_CONFIG.CAM_NAMES.push_back(cam1Path);
        // DEV_CONFIG.CAM_NAMES[1] = (cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        DEV_CONFIG.RIC.push_back(T.block<3, 3>(0, 0));
        DEV_CONFIG.TIC.push_back(T.block<3, 1>(0, 3));
        // DEV_CONFIG.RIC[1] = (T.block<3, 3>(0, 0));
        // DEV_CONFIG.TIC[1] = (T.block<3, 1>(0, 3));
    }

    DEV_CONFIG.INIT_DEPTH = 5.0;
    DEV_CONFIG.BIAS_ACC_THRESHOLD = 0.1;
    DEV_CONFIG.BIAS_GYR_THRESHOLD = 0.1;

    DEV_CONFIG.TD = fsSettings["td"];
    DEV_CONFIG.ESTIMATE_TD = fsSettings["estimate_td"];
    if (DEV_CONFIG.ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << DEV_CONFIG.TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << DEV_CONFIG.TD);

    DEV_CONFIG.ROW = fsSettings["image_height"];
    DEV_CONFIG.COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", DEV_CONFIG.ROW, DEV_CONFIG.COL);

    if(!DEV_CONFIG.USE_IMU)
    {
        DEV_CONFIG.ESTIMATE_EXTRINSIC = 0;
        DEV_CONFIG.ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
