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
// DeviceConfig_t      DEV_CONFIG;
int                 N_DEVICES;
DeviceConfig_t      DEV_CONFIGS[MAX_NUM_DEVICES];

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
    ////////// BEGIN HERE //////////////////////////////////
    N_DEVICES = fsSettings["num_of_devices"];
    bool if_old_config = (N_DEVICES == 0);
    N_DEVICES = if_old_config? 1:N_DEVICES; // only one device supported at the time for old config
    
    for (int i = 0; i < N_DEVICES; i++)
    {
        DeviceConfig_t* cfg = & (DEV_CONFIGS[i]);
        std::string pre_ = "d"+ std::to_string(i) + "_";
        if (if_old_config){
            pre_ = ""; // no prefixes
        }

        printf("=== [Indexing %scameras_imu ]", pre_.c_str());
        
        // # Camera topic
        fsSettings[pre_+"image0_topic"] >> cfg->IMAGE_TOPICS[0];
        fsSettings[pre_+"image1_topic"] >> cfg->IMAGE_TOPICS[1];

        // # feature traker paprameters
        cfg->MAX_CNT         = fsSettings["max_cnt"];
        cfg->MIN_DIST        = fsSettings["min_dist"];
        cfg->F_THRESHOLD     = fsSettings["F_threshold"];
        cfg->SHOW_TRACK      = fsSettings["show_track"];
        cfg->FLOW_BACK       = fsSettings["flow_back"];
        cfg->MULTIPLE_THREAD = fsSettings["multiple_thread"];
        
        // # optimization parameters
        cfg->SOLVER_TIME    = fsSettings["max_solver_time"];
        cfg->NUM_ITERATIONS = fsSettings["max_num_iterations"];
        cfg->MIN_PARALLAX   = static_cast<float>(fsSettings["keyframe_parallax"]) / FOCAL_LENGTH;

        // # other parameters
        cfg->INIT_DEPTH = 5.0;
        cfg->BIAS_ACC_THRESHOLD = 0.1;            //[unused]
        cfg->BIAS_GYR_THRESHOLD = 0.1;            //[unused]
        cfg->ROW = fsSettings["image_height"];    //[unused]
        cfg->COL = fsSettings["image_width"];     //[unused]
        ROS_INFO("ROW: %d COL: %d ", cfg->ROW, cfg->COL);


        // # IMU:
        cfg->USE_IMU        = fsSettings[pre_+"imu"];
        cfg->TD             = fsSettings[pre_+"td"];
        printf("USE_IMU: %d\n", cfg->USE_IMU);
        if(cfg->USE_IMU)
        {
            fsSettings[pre_+"imu_topic"] >> cfg->IMU_TOPIC;
            printf("IMU_TOPIC: %s\n", cfg->IMU_TOPIC.c_str());
            cfg->ACC_N = fsSettings[pre_+"acc_n"];
            cfg->ACC_W = fsSettings[pre_+"acc_w"];
            cfg->GYR_N = fsSettings[pre_+"gyr_n"];
            cfg->GYR_W = fsSettings[pre_+"gyr_w"];
            cfg->G.z() = fsSettings[pre_+"g_norm"];
            // time diff:
            cfg->ESTIMATE_TD = fsSettings[pre_+"estimate_td"];
            if (cfg->ESTIMATE_TD)
                ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << cfg->TD);
            else
                ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << cfg->TD);
        }
        else
        {
            cfg->ESTIMATE_EXTRINSIC   = CALIB_EXACT;
            cfg->ESTIMATE_TD          = CALIB_EXACT;
            printf("no imu, fix extrinsic param; no time offset calibration\n");
        }

        // # output:
        fsSettings["output_path"] >> cfg->OUTPUT_FOLDER;
        cfg->VINS_RESULT_PATH = cfg->OUTPUT_FOLDER + "/vio.csv";
        std::cout << "result path " << cfg->VINS_RESULT_PATH << std::endl;
        std::ofstream fout(cfg->VINS_RESULT_PATH, std::ios::out);
        fout.close();

        // # calibration of first camera
        Eigen::Matrix3d R_ic = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_ic = Eigen::Vector3d::Zero();
        cfg->ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
        switch (cfg->ESTIMATE_EXTRINSIC)
        {
            case (CALIB_NO_PRIOR):
                {
                    ROS_WARN(" No prior about extrinsic param, calibrate extrinsic param");
                    cfg->EX_CALIB_RESULT_PATH = cfg->OUTPUT_FOLDER + "/extrinsic_parameter.csv";
                }
                break;
            case (CALIB_INIT_GUESS):
                {
                    ROS_WARN(" Optimize extrinsic param around initial guess!");
                    cfg->EX_CALIB_RESULT_PATH = cfg->OUTPUT_FOLDER + "/extrinsic_parameter.csv";    
                }
                // fall through
            case (CALIB_EXACT):
                {
                    ROS_WARN(" Exact extrinsic param ");
                }
                // fall through
            default:
                {
                    ROS_WARN(" > Fetching extrinsic param ... ");
                    cv::Mat cv_T;
                    Eigen::Matrix4d T;
                    // extract:
                    fsSettings[pre_+"body_T_cam0"] >> cv_T;
                    cv::cv2eigen(cv_T, T);
                    R_ic = T.block<3, 3>(0, 0);
                    p_ic = T.block<3, 1>(0, 3);
                }        
        }
        cfg->RIC[0] = (R_ic);
        cfg->TIC[0] = (p_ic);


        // # calibration of 2nd camera if has
        int n0_cam = fsSettings[pre_+"num_of_cam"];

        printf("camera number %d\n", n0_cam);
        if(n0_cam != 1 && n0_cam != 2)
        {
            printf("num_of_cam should be 1 or 2\n");
            assert(0);
        }

        // @ cam paths:
        std::string configPath_, cam0Calib_, cam0Path_;
        fsSettings[pre_+"cam0_calib"] >> cam0Calib_;

        int pn = config_file.find_last_of('/');
        configPath_ = config_file.substr(0, pn);
        cam0Path_ = configPath_ + "/" + cam0Calib_;
        
        // - cache:
        cfg->CAM_NAMES[0] = cam0Path_;
        cfg->NUM_OF_CAM = n0_cam;

        // debug:
        printf("%s cam0 path\n", cam0Path_.c_str() );
        cfg->STEREO = (n0_cam == 2)?1:0;
        if(cfg->STEREO)
        {
            // cam1 path:
            fsSettings[pre_+"cam1_calib"] >> cam0Calib_;
            cam0Path_ = configPath_ + "/" + cam0Calib_; 
            printf("%s cam1 path\n", configPath_.c_str() );
            cfg->CAM_NAMES[1] = cam0Path_;
            
            // - transformation:
            cv::Mat cv_T;
            Eigen::Matrix4d T;
            fsSettings[pre_+"body_T_cam1"] >> cv_T;
            cv::cv2eigen(cv_T, T);

            // cache:
            cfg->CAM_NAMES[0] = cam0Path_;
            cfg->RIC[1] = T.block<3, 3>(0, 0);
            cfg->TIC[1] = T.block<3, 1>(0, 3);
        }
    }

    ////////////////////////////////////////////////////////////////
    fsSettings.release();
}
