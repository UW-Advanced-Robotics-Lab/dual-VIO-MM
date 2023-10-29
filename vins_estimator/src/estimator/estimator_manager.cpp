#include "estimator_manager.h"
#include "../robot/Lie.h"

EstimatorManager::EstimatorManager(std::shared_ptr<DeviceConfig_t> _pCfgs[])
{
    ROS_INFO("EstimatorManager Begins~~~~~~~~~~~~~~~~");
    // init:
    pCfgs[BASE_DEV] = _pCfgs[BASE_DEV];
    pCfgs[EE_DEV  ] = _pCfgs[EE_DEV  ];
    // estimators:
    pEsts[BASE_DEV] = std::make_shared<Estimator>(pCfgs[BASE_DEV]);
    pEsts[EE_DEV  ] = std::make_shared<Estimator>(pCfgs[EE_DEV  ]);
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    // arm:
    pArm = std::make_shared<ArmModel>();
#endif
}

EstimatorManager::~EstimatorManager()
{
    if (pProcessThread != nullptr)
    {
        pProcessThread->join();
        PRINT_WARN("join base & ee estimator thread \n");
        pProcessThread.reset();
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
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    // arm:
    pArm.reset();
#endif
}

void EstimatorManager::restartManager()
{
    // reset estimators:
    pEsts[BASE_DEV]->clearState();
    pEsts[EE_DEV  ]->clearState();
    pEsts[BASE_DEV]->setParameter();
    pEsts[EE_DEV  ]->setParameter();

    // activate threads if not already:
    if (pProcessThread == nullptr)
    {
        pProcessThread = std::make_shared<std::thread>(& EstimatorManager::processMeasurements_thread, this);
    }
    if (pPublishThread == nullptr)
    {
        pPublishThread = std::make_shared<std::thread>(& EstimatorManager::publishVisualization_thread, this);
    }
    PRINT_DEBUG("[EstimatorManager::restartManager] Manager Ready!");
}

void EstimatorManager::processMeasurements_thread()
{
    const std::chrono::milliseconds rate(EST_MANAGER_PROCESSING_INTERVAL_MS); 
    double prevTime[MAX_NUM_DEVICES] = {0.0, 0.0};

    TicToc ellapsed_process;
    TicToc ellapsed_process_i;
    while (FOREVER)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        TIK(ellapsed_process);
        // Processing Tasks:
        {
            pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature[MAX_NUM_DEVICES];
            vector<pair<double, Eigen::Vector3d>> accVector[MAX_NUM_DEVICES], gyrVector[MAX_NUM_DEVICES];
            pair<double, Vector7d_t> armVector;
            double curTime[MAX_NUM_DEVICES];

            const bool is_base_avail = (pEsts[BASE_DEV]->featureBuf.empty() == false);
            const bool is_EE_avail   = (pEsts[EE_DEV  ]->featureBuf.empty() == false);

            // Only process when both features are available:
            if (is_base_avail && is_EE_avail)
            {
                // fetch features:
                TIK(ellapsed_process_i);
                for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                {
                    pEsts[id]->mBuf.lock();
                    feature[id] = pEsts[id]->featureBuf.front();
                    pEsts[id]->featureBuf.pop(); // drop frame buffer
                    pEsts[id]->mBuf.unlock();
                    curTime[id] = feature[id].first + pEsts[id]->td;
                }
                TOK_TAG(ellapsed_process_i, "fetch_features");

#           if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
                TIK(ellapsed_process_i);
                bool is_arm_avail = (this->arm_buf.data.empty() == false);
                if (is_arm_avail)
                {
                    while (this->arm_buf.data.empty() == false) // ideally one loop needed, but loop to drop excessive in case
                    {
                        // acquire:
                        this->arm_buf.guard.lock();
                        armVector = this->arm_buf.data.front();
                        this->arm_buf.data.pop();
                        this->arm_buf.guard.unlock();
                        // queue:
                        // PRINT_DEBUG("Delta Time dt=%f , dt=%f \n", (armVector.first - feature[0].first), (feature[1].first - feature[0].first));
                        if ((armVector.first - feature[0].first) >= -0.00001)
                        {
                            // process:
                            this->processArm_unsafe(armVector.first, armVector.second);
                            break;
                        }
                    }
                }
                else
                {
                    PRINT_DEBUG("Arm is NOT available");
                }
                TOK_TAG(ellapsed_process_i, "fetch_arm_odom");
#           endif

                const bool is_base_imu_avail    = pEsts[BASE_DEV]->IMUAvailable(curTime[BASE_DEV]);
                const bool is_EE_imu_avail      = pEsts[EE_DEV  ]->IMUAvailable(curTime[EE_DEV  ]);
                
                // fetch and process:
                if (is_base_imu_avail && is_EE_imu_avail)
                {
                    // fetch IMU:
                    TIK(ellapsed_process_i);
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                    {
                        // fetch imu buffers from the time window:
                        pEsts[id]->mBuf.lock();
                        // pEsts[id]->featureBuf.pop(); // drop frame buffer
                        pEsts[id]->getIMUInterval_unsafe(prevTime[id], curTime[id], accVector[id], gyrVector[id]);
                        pEsts[id]->mBuf.unlock();
                        
                        // initialize:
                        if (! pEsts[id]->initFirstPoseFlag)
                            pEsts[id]->initFirstIMUPose(accVector[id]);
                    }
                    TOK_TAG(ellapsed_process_i, "fetch_imu");

                    // process imu iteratively:
                    TIK(ellapsed_process_i);
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                    {
                        for(size_t i = 0; i < accVector[id].size(); i++)
                        {
                            double dt;
                            if(i == 0) // head
                                dt = accVector[id][i].first - prevTime[id];
                            else if (i == accVector[id].size() - 1)
                            {
                                // only sample the partial of the tail imu data up to the time the frame was captured
                                dt = curTime[id] - accVector[id][i - 1].first;
                                // double dt2 = accVector[i].first - accVector[i - 1].first;
                                // PRINT_DEBUG("dt=%f|%f, ~%f", dt, dt2, dt2-dt); // idff is 0.001s
                            }
                            else // middle
                                dt = accVector[id][i].first - accVector[id][i - 1].first;
                            pEsts[id]->processIMU(accVector[id][i].first, dt, accVector[id][i].second, gyrVector[id][i].second);
                        }
                    }
                    TOK_TAG(ellapsed_process_i, "imu_process");

                    // process Image:
                    TIK(ellapsed_process_i);
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                    {
                        // arm_buf.R_result, arm_buf.p_result
                        pEsts[id]->mProcess.lock();
                        pEsts[id]->processImage(feature[id].second, feature[id].first);
                        prevTime[id] = curTime[id];

                        /*** Publish ***/
#                   if (FEATURE_ENABLE_STATISTICS_LOGGING)
                        // Print Statistics:
                        printStatistics(*pEsts[id], 0);
#                   endif

                        // Publish:
                        std_msgs::Header header;
                        header.frame_id = "world";
                        header.stamp = ros::Time(feature[id].first);
                        // immediate updates:
                        {
                            pubKeyframe_Odometry_and_Points_immediately(*pEsts[id]);
                            pubTF_immediately(*pEsts[id], header);
                            pubOdometry_Immediately(*pEsts[id], header);
                        }
#                   if (FEATURE_VIZ_PUBLISH)
                        // visualization updates:
                        visualization_guard_lock(*pEsts[id]);
                        {
                            queue_KeyPoses_unsafe(*pEsts[id], header);
                            queue_CameraPose_unsafe(*pEsts[id], header);
                            queue_PointCloud_unsafe(*pEsts[id], header);
                        }
                        visualization_guard_unlock(*pEsts[id]);
#                   endif
                        
                        // End-of-publishing
                        pEsts[id]->mProcess.unlock();
                    }
                    TOK_TAG(ellapsed_process_i, "image_process_and_publish");
                }
                else
                {
                    PRINT_WARN("wait for both imu [%d|%d] ... \n", is_base_imu_avail, is_EE_imu_avail);
                }
            }

        }
        TOK_IF(ellapsed_process,EST_MANAGER_PROCESSING_INTERVAL_MS);

        // dynamic sleeping:
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Sleep for the remaining time to achieve the fixed rate:
        if (elapsed_time < rate) {
            std::this_thread::sleep_for(rate - elapsed_time);
        }
    }
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

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
void EstimatorManager::inputArm(double t, 
    const sensor_msgs::JointStateConstPtr &_jnt_msg_b)
    // const sensor_msgs::JointStateConstPtr &_jnt_msg_E)
{
    // input joints:
    Vector7d_t jnt_msg_b;
    // Vector7d_t jnt_vel, jnt_tau;
    if (_jnt_msg_b)
    {
        for (int i = 0; i < ARM_NUM_DOF; ++i) {
            jnt_msg_b(i) = _jnt_msg_b->position[i];
            // jnt_msg_E(i) = _jnt_msg_E->position[i];
        }
        this->arm_buf.guard.lock();
        this->arm_buf.data.push(make_pair(t, jnt_msg_b));
        // this->arm_buf.data[EE_DEV  ].push(make_pair(t, jnt_msg_E));
        this->arm_buf.guard.unlock();
    }
    else
    {
        PRINT_ERROR("Joint Messages are not Available!!!!");
    }
}
void EstimatorManager::processArm_unsafe(const double t, const Vector7d_t& jnt_vec)
{
    const Lie::SE3 Te = pArm->getCamEE();
    const Lie::SE3 Tb = pArm->getCamBase();
    if (pEsts[BASE_DEV]->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {

        Lie::R3 p_c, v_c; Lie::SO3 R_c;
        // fetch R,p previous
        p_c = pEsts[BASE_DEV]->Ps[WINDOW_SIZE];
        v_c = pEsts[BASE_DEV]->Vs[WINDOW_SIZE];
        R_c = pEsts[BASE_DEV]->Rs[WINDOW_SIZE];

        const Lie::SO3 R_corr(
            (Lie::SO3() <<  0, -1,  0,
                            0,  0, -1,
                            1,  0,  0).finished()
        ); // convert from camera axis back to world axis
        const Lie::SO3 R_corr_T(
            (Lie::SO3() <<   0,  0,  1,
                            -1,  0,  0,
                             0, -1,  0).finished()
        ); // convert from world axis back to camera axis
        
        Lie::SE3 T_c = Lie::SE3_from_SO3xR3(R_c * R_corr, p_c);
        
        // PRINT_DEBUG("jnt: %s", Lie::to_string(jnt_vec.transpose()).c_str());
        // compute theta --> SE3:
        Lie::SE3 g_st;
        // pArm->setAngles_unsafely(Vector7d_t::Zero());
        pArm->setAngles_unsafely(jnt_vec);
        pArm->processJntAngles_unsafely();
        pArm->getEndEffectorPose_unsafely(g_st);
        
        Lie::SE3 T_b = T_c;
        
        // rebase base cam frame onto the summit base frame
        Lie::SE3 Tc_b = Lie::inverse_SE3(Tb);
        T_b = T_b * Tc_b;

        // rebase arm config onto the base frame
        T_b = T_b * g_st;

        // apply tool tip frame
        T_b = T_b * Te;

        // PRINT_DEBUG("> ArmOdometry [%f]: \n R=\n%s \n p=\n%s", t, Lie::to_string(R).c_str(), Lie::to_string(p).c_str());
        Lie::SO3xR3_from_SE3(arm_buf.R_result, arm_buf.p_result, T_b);
        arm_buf.R_result *= R_corr_T;
        
        // publish immediately:
        queue_ArmOdometry_safe(t, arm_buf.R_result, arm_buf.p_result, BASE_DEV);
    }
}
#endif 

void EstimatorManager::inputImage(double t, const cv::Mat &_img_b, const cv::Mat &_img_e)
{
    // input images:
    pEsts[BASE_DEV]->inputImage(t, _img_b);
    pEsts[EE_DEV  ]->inputImage(t, _img_e);
}

void EstimatorManager::publishVisualization_thread()
{
    const std::chrono::milliseconds rate(TOPIC_PUBLISH_INTERVAL_MS); 
    TicToc ellapsed_publisher;

    while(FOREVER)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        TIK(ellapsed_publisher);
        // Publishing Tasks:
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
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
            pubArmOdometry_safe(dev_id);
#endif
        }
        TOK_IF(ellapsed_publisher,TOPIC_PUBLISH_INTERVAL_MS);
        
        // dynamic sleeping:
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Sleep for the remaining time to achieve the fixed rate:
        if (elapsed_time < rate) {
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