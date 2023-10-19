#include "parameters.h"
// ----------------------------------------------------------------
// : Public Functions :
// ----------------------------------------------------------------
int readParameters(
    const std::string config_file, const std::string pkg_path, 
    DeviceConfig_t DEV_CONFIGS[], LoopDevice_t LoopDevices[]
) //-->N_DEVICES
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
        std::cerr << "[ERROR]: Wrong path to settings" << std::endl;
        throw std::runtime_error("[ERROR]: Wrong path to settings");
    }
    ////////// BEGIN HERE //////////////////////////////////
    int N_DEVICES = fsSettings["num_of_devices"];
    bool if_old_config = (N_DEVICES == 0);
    N_DEVICES = if_old_config? 1:N_DEVICES; // only one device supported at the time for old config
    
    for (int i = 0; i < N_DEVICES; i++)
    {
        DeviceConfig_t* cfg = & (DEV_CONFIGS[i]);
        cfg->DEVICE_ID = i;
        
        std::string pre_ = "d"+ std::to_string(i) + "_";
        if (if_old_config){
            pre_ = ""; // no prefixes, backwards compatibility
        }

        PRINT_INFO("=== [Indexing %s cameras ] \n", pre_.c_str());
        // cache:
        cfg->ROW = fsSettings["image_height"];
        cfg->COL = fsSettings["image_width"];
        cfg->USE_IMU = CV_YAML_TO_BOOL(fsSettings[pre_+"imu"]);

        // cache:
        fsSettings[pre_+"focal_length"] >> cfg->FOCAL_LENGTH;
        fsSettings[pre_+"output_path"] >> cfg->OUTPUT_PATH;
        fsSettings[pre_+"image0_topic"] >> cfg->IMAGE_TOPIC;        
        fsSettings[pre_+"pose_graph_save_path"] >> cfg->POSE_GRAPH_SAVE_PATH;
        cfg->DEBUG_IMAGE = CV_YAML_TO_BOOL(fsSettings[pre_+"save_image"]);

        // generate config path & cache:
        int pn = config_file.find_last_of('/');
        std::string configPath = config_file.substr(0, pn);
        std::string cam0Calib;
        fsSettings[pre_+"cam0_calib"] >> cam0Calib;
        cfg->CAM0_CALIB_PATH = configPath + "/" + cam0Calib;
        PRINT_INFO("cam calib path: %s\n", cfg->CAM0_CALIB_PATH.c_str());

        // generate support path & cache:
        cfg->VOCAB_FILE_PATH = pkg_path + "/../support_files/brief_k10L6.bin";
        PRINT_INFO("VOCAB_FILE_PATH %s\n", cfg->VOCAB_FILE_PATH.c_str());

        // generate pattern path & cache:
        cfg->BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        PRINT_INFO("BRIEF_PATTERN_FILE %s\n", cfg->BRIEF_PATTERN_FILE.c_str());

        cfg->LOAD_PREVIOUS_POSE_GRAPH = CV_YAML_TO_BOOL(fsSettings[pre_+"load_previous_pose_graph"]);
        cfg->VINS_RESULT_PATH = cfg->OUTPUT_PATH + "/vio_loop.csv";
        PRINT_INFO("VINS_RESULT_PATH %s\n", cfg->VINS_RESULT_PATH.c_str());
    }

    ////////////////////////////////////////////////////////////////
    fsSettings.release();
    return N_DEVICES;
}


