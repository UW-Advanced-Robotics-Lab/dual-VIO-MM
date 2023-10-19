#include "estimator_manager.h"

EstimatorManager::EstimatorManager(std::shared_ptr<DeviceConfig_t> _pCfgs[])
{
    ROS_INFO("EstimatorManager Begins~~~~~~~~~~~~~~~~");
    // init:
    pCfgs[BASE_DEV] = _pCfgs[BASE_DEV];
    pCfgs[EE_DEV  ] = _pCfgs[EE_DEV  ];
    // estimators:
    pEsts[BASE_DEV] = std::make_shared<Estimator>(pCfgs[BASE_DEV]);
    pEsts[EE_DEV  ] = std::make_shared<Estimator>(pCfgs[EE_DEV  ]);
}

EstimatorManager::~EstimatorManager()
{
    if (pProcessThread[BASE_DEV] != nullptr)
    {
        pProcessThread[BASE_DEV]->join();
        PRINT_WARN("join base estimator thread \n");
        pProcessThread[BASE_DEV].reset();
    }
    if (pProcessThread[EE_DEV  ] != nullptr)
    {
        pProcessThread[EE_DEV  ]->join();
        PRINT_WARN("join ee estimator thread \n");
        pProcessThread[EE_DEV  ].reset();
    }
    if (pPublishThread != nullptr)
    {
        pPublishThread->join();
        PRINT_WARN("join publisher thread \n");
        pPublishThread.reset();
    }
    // detaching pointers:
    pCfgs[BASE_DEV].reset();
    pCfgs[EE_DEV  ].reset();
    pEsts[BASE_DEV].reset();
    pEsts[EE_DEV  ].reset();
}

void EstimatorManager::restartManager()
{
    // reset estimators:
    pEsts[BASE_DEV]->clearState();
    pEsts[EE_DEV  ]->clearState();
    pEsts[BASE_DEV]->setParameter();
    pEsts[EE_DEV  ]->setParameter();

    // activate threads if not already:
    if (pProcessThread[BASE_DEV] == nullptr)
    {
        pProcessThread[BASE_DEV] = std::make_shared<std::thread>(& Estimator::processMeasurements, pEsts[BASE_DEV]);
    }
    if (pProcessThread[EE_DEV  ] == nullptr)
    {
        pProcessThread[EE_DEV  ] = std::make_shared<std::thread>(& Estimator::processMeasurements, pEsts[EE_DEV  ]);
    }
    if (pPublishThread == nullptr)
    {
        pPublishThread = std::make_shared<std::thread>(& EstimatorManager::publishVisualization, this);
    }
    PRINT_DEBUG("[EstimatorManager::restartManager] Manager Ready!");
}

void EstimatorManager::registerPublishers(ros::NodeHandle &n, const int N_DEVICES)
{
    if (N_DEVICES==MAX_NUM_DEVICES)
    {
        assert("single device not supported");
    }
    registerPub(n, N_DEVICES);
}

void EstimatorManager::inputIMU(const size_t DEV_ID, const double t, const Vector3d &acc, const Vector3d &gyr)
{
    pEsts[DEV_ID]->inputIMU(t, acc, gyr);
}

static void _process_JntVector_from_msg(const sensor_msgs::JointStateConstPtr &_jnt_msg, Vector7d_t &jnt_pos, Vector7d_t &jnt_vel, Vector7d_t &jnt_tau)
{
    // PRINT_ARRAY(_jnt_msg->position, 7);
    for (int i = 0; i < 7; ++i) {
        jnt_pos(i) = _jnt_msg->position[i];
        jnt_vel(i) = _jnt_msg->velocity[i];
        jnt_tau(i) = _jnt_msg->effort[i];
    }
    // std::cout << jnt_pos << std::endl;
}

void EstimatorManager::inputImage(double t, const cv::Mat &_img_b, const cv::Mat &_img_e, const sensor_msgs::JointStateConstPtr &_jnt_msg)
{
    // input joints:
    if (_jnt_msg)
    {
        Vector7d_t jnt_pos, jnt_vel, jnt_tau;
        _process_JntVector_from_msg(_jnt_msg, jnt_pos, jnt_vel, jnt_tau);
    }
    // input images:
    pEsts[BASE_DEV]->inputImage(t, _img_b);
    pEsts[EE_DEV  ]->inputImage(t, _img_e);
}

void EstimatorManager::publishVisualization()
{
    const std::chrono::milliseconds rate(TOPIC_PUBLISH_INTERVAL_MS); 
    TicToc ellapsed_publisher;

    while(FOREVER)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        TIK(ellapsed_publisher);
        // Tasks:
        for(int dev_id=BASE_DEV; dev_id<MAX_NUM_DEVICES; dev_id++)
        {
            // publishing topics:
            pubViconOdometryPath_safe(dev_id);
            pubOdometryPath_safe(dev_id);
            pubKeyPoses_safe(dev_id);
            pubCameraPose_safe(dev_id);
            pubPointClouds_safe(dev_id);
#if (FEATURE_TRACKING_IMAGE_SUPPORT)
            pubTrackImage_safe(dev_id);
#endif
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Sleep for the remaining time to achieve the fixed rate:
        if (elapsed_time < rate) {
            TOK(ellapsed_publisher);
            std::this_thread::sleep_for(rate - elapsed_time);
        }
    }
}

#if (FEATURE_ENABLE_VICON_SUPPORT)
void callback_viconOdometry(const geometry_msgs::TransformStampedConstPtr &transform_msg, const int device_id)
{
    queue_ViconOdometry_safe(transform_msg, device_id); // NOTE: should be behind the estimator time (0.1s ish)
}
#endif