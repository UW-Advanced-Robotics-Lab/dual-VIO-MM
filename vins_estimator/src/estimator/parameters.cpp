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
// : Public Functions :
// ----------------------------------------------------------------
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    int readParameters(
        const std::string config_file, 
        std::shared_ptr<DeviceConfig_t>     pCfgs[], 
        std::shared_ptr<ArmConfig_t>        pArmCfg) //--> N_DEVICES
#else
    int readParameters(
        const std::string config_file, 
        std::shared_ptr<DeviceConfig_t>     pCfgs[]) //--> N_DEVICES
#endif
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return 0;  
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    ////////// BEGIN HERE //////////////////////////////////
    int N_DEVICES = fsSettings["num_of_devices"];
    bool if_old_config = (N_DEVICES == 0);
    N_DEVICES = if_old_config? 1:N_DEVICES; // only one device supported at the time for old config
    // common:
    int pn = config_file.find_last_of('/');
    std::string configPath_ = config_file.substr(0, pn);

    // loading arm:
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    fsSettings["arm_joint_topic"] >> pArmCfg->JOINTS_TOPIC;
    fsSettings["arm_pose_topic"] >> pArmCfg->POSE_TOPIC;
    pArmCfg->CALIBRATION_FILE_PATH = configPath_ + "/" + static_cast<std::string>(fsSettings["arm_calib"]);
#endif
    
    // loading camera devices:
    for (int i = 0; i < N_DEVICES; i++)
    {
        std::shared_ptr<DeviceConfig_t> cfg = pCfgs[i];
        cfg->DEVICE_ID = i;
        
        std::string pre_ = "d"+ std::to_string(i) + "_";
        if (if_old_config){
            pre_ = ""; // no prefixes
        }

        PRINT_WARN("=== [Indexing %s cameras_imu ]", pre_.c_str());
        
        // # Camera topic
        fsSettings[pre_+"image0_topic"] >> cfg->IMAGE_TOPICS[0];
        fsSettings[pre_+"image1_topic"] >> cfg->IMAGE_TOPICS[1];
        fsSettings[pre_+"focal_length"] >> cfg->FOCAL_LENGTH;

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
        cfg->MIN_PARALLAX   = static_cast<float>(fsSettings["keyframe_parallax"]) / cfg->FOCAL_LENGTH;

        // # other parameters
        cfg->INIT_DEPTH = 5.0;
        cfg->BIAS_ACC_THRESHOLD = 0.1;            //[unused]
        cfg->BIAS_GYR_THRESHOLD = 0.1;            //[unused]
        cfg->ROW = fsSettings["image_height"];    //[unused]
        cfg->COL = fsSettings["image_width"];     //[unused]
        ROS_INFO("ROW: %d COL: %d ", cfg->ROW, cfg->COL);

#if (FEATURE_ENABLE_VICON_SUPPORT)
        // # vicon:
        fsSettings[pre_+"vicon_topic"] >> cfg->VICON_TOPIC;
#endif
        // # IMU:
        cfg->USE_IMU        = fsSettings[pre_+"imu"];
        cfg->TD             = fsSettings[pre_+"td"];
        PRINT_INFO("USE_IMU: %d", cfg->USE_IMU);
        if(cfg->USE_IMU)
        {
            fsSettings[pre_+"imu_topic"] >> cfg->IMU_TOPIC;
            PRINT_INFO("IMU_TOPIC: %s", cfg->IMU_TOPIC.c_str());
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
            PRINT_INFO("no imu, fix extrinsic param; no time offset calibration");
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

        PRINT_INFO("camera number %d", n0_cam);
        if(n0_cam != 1 && n0_cam != 2)
        {
            PRINT_INFO("num_of_cam should be 1 or 2");
            assert(0);
        }

        // @ cam paths:
        std::string cam0Calib_, cam0Path_;
        fsSettings[pre_+"cam0_calib"] >> cam0Calib_;
        cam0Path_ = configPath_ + "/" + cam0Calib_;
        
        // - cache:
        cfg->CAM_NAMES[0] = cam0Path_;
        PRINT_INFO("cam0 path: %s", cam0Path_.c_str() );

#if (FEATURE_ENABLE_STEREO_SUPPORT)
        // stereo?
        cfg->NUM_OF_CAM = n0_cam;
        cfg->STEREO = (n0_cam == 2)?1:0;
        if(cfg->STEREO)
        {
            // cam1 path:
            fsSettings[pre_+"cam1_calib"] >> cam0Calib_;
            cam0Path_ = configPath_ + "/" + cam0Calib_; 
            PRINT_INFO("%s cam1 path", can0Path.c_str() );
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
#else
        cfg->NUM_OF_CAM = 1;    // Forcing monocular camera
        cfg->STEREO = 0;        // Disable stereo support
#endif
    }

    ////////////////////////////////////////////////////////////////
    fsSettings.release();
    return N_DEVICES;
}
