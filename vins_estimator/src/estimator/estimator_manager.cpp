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

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
    // arm:
    arm_prev_data.t_header = -1;
    arm_prev_data.arm_pose_header = -1;
    arm_prev_data.arm_pose_ready = false;
#endif
    PRINT_DEBUG("[EstimatorManager::restartManager] Manager Ready!");
}

void EstimatorManager::processMeasurements_thread()
{
    const std::chrono::milliseconds rate(EST_MANAGER_PROCESSING_INTERVAL_MS); 
    double prevTime[MAX_NUM_DEVICES] = {0.0, 0.0};

    TicToc ellapsed_process;
    while (FOREVER)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        TIK(ellapsed_process);
        TicToc ellapsed_process_i;
        // Processing Tasks:
        {
            pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature[MAX_NUM_DEVICES];
            vector<pair<double, Eigen::Vector3d>> accVector[MAX_NUM_DEVICES], gyrVector[MAX_NUM_DEVICES];
            pair<double, Vector7d_t> armVector[MAX_NUM_DEVICES];
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

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
                TIK(ellapsed_process_i);
                bool is_arm_avail = (this->arm_buf.data.empty() == false);
                if (is_arm_avail)
                {
                    // iterate to find the closest arm joints vector:
                    is_arm_avail &= this->_getJointVector_safe(armVector[BASE_DEV], feature[BASE_DEV].first, true);
                    // is_arm_avail &= this->_getJointVector_safe(armVector[EE_DEV  ], feature[EE_DEV  ].first, true );
                    if (is_arm_avail)
                    {
                        // TODO: armvector EE the other way around
                        pArm->acquireLock();
                        this->_postProcessArmJnts_unsafe(armVector[BASE_DEV].first, armVector[BASE_DEV].second);
                        pArm->releaseLock();
                    }
                }

                TOK_TAG(ellapsed_process_i,"fetch_arm_odom");
#endif //FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT

                const bool is_base_imu_avail    = pEsts[BASE_DEV]->IMUAvailable(curTime[BASE_DEV]);
                const bool is_EE_imu_avail      = pEsts[EE_DEV  ]->IMUAvailable(curTime[EE_DEV  ]);
                
                // fetch and process:
                if (is_base_imu_avail && is_EE_imu_avail && is_arm_avail)
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
                        {
                            pEsts[id]->initFirstIMUPose(accVector[id]);
                            if ((id == EE_DEV) && (is_arm_avail))
                            {
                                pEsts[EE_DEV]->initFirstPose(this->arm_prev_data.arm_pose_st);
                            }
                            else
                            {
                                PRINT_ERROR("Error initializing");
                            }
                        }
                    }

                    TOK_TAG(ellapsed_process_i,"fetch_imu");

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
                    TOK_TAG(ellapsed_process_i,"imu_proc");

                    //TODO: around 20ms x2 = 40ms, we should parallel these two processes? [@later, @run-time-concern]
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                    {
                        // process Image:
                        // NOTE: process and publish has to be under same lock!
                        // TODO: [@urgent] please investigate the source of error that does not allow the process and publish to be separated
                        pEsts[id]->mProcess.lock();
                        TIK(ellapsed_process_i);
                        pEsts[id]->processImage(feature[id].second, feature[id].first, arm_prev_data.arm_pose_st);
                        prevTime[id] = curTime[id];
                        TOK_TAG(ellapsed_process_i,"img_proc");

                        /*** Publish ***/
                        TIK(ellapsed_process_i);
#if (FEATURE_ENABLE_STATISTICS_LOGGING)
                        // Print Statistics:
                        printStatistics(*pEsts[id], 0);
#endif //(FEATURE_ENABLE_STATISTICS_LOGGING)
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
#if (FEATURE_VIZ_PUBLISH)
                        // visualization updates:
                        visualization_guard_lock(*pEsts[id]);
                        {
                            queue_KeyPoses_unsafe(*pEsts[id], header);
                            queue_CameraPose_unsafe(*pEsts[id], header);
                            queue_PointCloud_unsafe(*pEsts[id], header);
                        }
                        visualization_guard_unlock(*pEsts[id]);
#endif //(FEATURE_VIZ_PUBLISH)
                        TOK_TAG(ellapsed_process_i,"img_pub");
                        
                        // End-of-publishing
                        pEsts[id]->mProcess.unlock();
                    }
                }
                else
                {
                    PRINT_WARN("wait for both imu and arm [%d|%d|%d] ... \n", is_base_imu_avail, is_EE_imu_avail, is_arm_avail);
                }
            }
            else
            {
                // When no images available, do nothing
            }

#if (FEATURE_ENABLE_VICON_SUPPORT)
            // TODO: should we sync against the image to have alignment evaluation? (or a separate topic)
            // queue vicon to publisher later:
            int i = 0;
            for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
            {
                this->m_vicon[id].guard.lock();
                while(! this->m_vicon[id].data.empty())
                {
                    if (i++ % FEATURE_VICON_DOWN_SAMPLE_RATE_PER == 0) // downsample 10%
                    {
                        queue_ViconOdometry_safe(this->m_vicon[id].data.front().second, this->m_vicon[id].data.front().first, id);
                    }
                    this->m_vicon[id].data.pop(); // popping :)
                }
                this->m_vicon[id].guard.unlock();
            }
#endif //(FEATURE_ENABLE_VICON_SUPPORT)
        }
        TOK_IF(ellapsed_process,EST_MANAGER_PROCESSING_INTERVAL_MS);

#if (RUN_EST_MANAGER_AT_FIXED_RATE)
        // dynamic sleeping:
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Sleep for the remaining time to achieve the fixed rate:
        if (elapsed_time < rate) {
            std::this_thread::sleep_for(rate - elapsed_time);
        }
#else
#   ifdef EST_MANAGER_PROCESSING_SLEEP_MS
        std::chrono::milliseconds dura(EST_MANAGER_PROCESSING_SLEEP_MS);
        std::this_thread::sleep_for(dura);
#   endif
#endif //(!RUN_EST_MANAGER_AT_FIXED_RATE)
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

#if (FEATURE_ENABLE_VICON_SUPPORT)
void EstimatorManager::inputVicon_safe(const size_t DEV_ID, const pair<double, Vector7d_t> &_vicon_msg)
{
    std::lock_guard<std::mutex> lock(this->m_vicon[BASE_DEV].guard);
    this->m_vicon[DEV_ID].data.push(_vicon_msg);
}
#endif // (FEATURE_ENABLE_VICON_SUPPORT)

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
void EstimatorManager::inputArmJnts_safe(const double t, const Vector7d_t &_jnt_msg)
{
    std::lock_guard<std::mutex> lock(this->arm_buf.guard);
    this->arm_buf.data.push(make_pair(t, _jnt_msg));
}
void EstimatorManager::emptyArmJnts_safe(const double t)
{
    std::lock_guard<std::mutex> lock(this->arm_buf.guard);
    while ((!this->arm_buf.data.empty()) && (t - this->arm_buf.data.front().first))
    {
        this->arm_buf.data.pop();
    }
}
bool EstimatorManager::_getJointVector_safe(pair<double, Vector7d_t> &jnt_buf, const double t, const bool if_pop)
{
    double best_dt;
    bool success = false;
    std::lock_guard<std::mutex> lock(this->arm_buf.guard);
    while (!this->arm_buf.data.empty())
    {
        pair<double, Vector7d_t> buf = this->arm_buf.data.front();
        double dt = t - buf.first; // with offset

        if (dt > 0)
        {
            if (dt < IMAGE_ARM_SYNC_TIME_DELTA_MAX)
            {
                jnt_buf.first = buf.first;
                jnt_buf.second = buf.second;
                best_dt = dt;
                success = true;
            }

            // pop
            if (if_pop)
                this->arm_buf.data.pop();
        }
        else
        {
            // picking the closest one:
            if (-dt < best_dt)
            {
                jnt_buf.first = buf.first;
                jnt_buf.second = buf.second;
                success = true;
            }
            break;
        }
    }
    return success;
}
void EstimatorManager::_postProcessArmJnts_unsafe(const double t, const Vector7d_t& jnt_vec)
{
    bool success = false;
    const Lie::SE3 Te = pArm->getCamEE();
    const Lie::SE3 Tb = pArm->getCamBase();
    const Lie::SE3 Tc_b = Lie::inverse_SE3(Tb);
    const Lie::SO3 R_corr(
        (Lie::SO3() <<  0, -1,  0,
                        0,  0, -1,
                        1,  0,  0).finished()
    ); // convert from camera axis back to world axis
    const Lie::SE3 Rp_corr_T(
        (Lie::SE3() <<   0,  0,  1,  0,
                        -1,  0,  0,  0,
                         0, -1,  0,  0,
                         0,  0,  0,  1).finished()
    ); // convert from world axis back to camera axis

    // placeholders:
    Lie::SE3 T_c, g_st, T_b;

    // compute:
    if ((arm_prev_data.t_header > 0) 
        && (pEsts[BASE_DEV]->solver_flag == Estimator::SolverFlag::NON_LINEAR))
    {
        // 1. fetch R,p previous:
        Lie::R3 p_c; Lie::SO3 R_c;

#   if (FEATURE_ENABLE_ARM_VICON_SUPPORT) // stub Ground Truth Vicon data:
        const Lie::SO3 R_corr2(
            (Lie::SO3() <<  0, -1,  0,
                            1,  0,  0,
                            0,  0,  1).finished()
        ); // convert vicon x axis fwd to y axis fwd, as summit face y axis

        geometry_msgs::Pose pose;
        getLatestViconPose_safe(pose, BASE_DEV);
        p_c = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        // R_c = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
        // T_c = Lie::SE3_from_SO3xR3(R_c * R_corr2, p_c); // vicon is aligned with world axis
        R_c = pEsts[BASE_DEV]->latest_Q.toRotationMatrix(); // previous frame
        T_c = Lie::SE3_from_SO3xR3(R_c * R_corr, p_c);
#   else // feed in estimator latest state:
        p_c = pEsts[BASE_DEV]->latest_P; // previous frame
        R_c = pEsts[BASE_DEV]->latest_Q.toRotationMatrix(); // previous frame
        // correction:
        T_c = Lie::SE3_from_SO3xR3(R_c * R_corr, p_c);
#   endif // (!FEATURE_ENABLE_ARM_VICON_SUPPORT)
        
        this->arm_prev_data.arm_pose_ready = true;

        // 2. [Arm] compute theta --> SE3:
        // pArm->setAngles_unsafely(Vector7d_t::Zero());
        pArm->setAngles_unsafely(arm_prev_data.arm_vec, t);
        pArm->processJntAngles_unsafely();
        pArm->getEndEffectorPose_unsafely(g_st);
        
        // // - rebase base cam frame onto the summit base frame
        // T_b = Tc_b;
        // // - rebase arm config onto the base frame
        // T_b = T_b * g_st;
        // // - apply tool tip frame
        // T_b = T_b * Te;

        Lie::SE3 T_out = T_c * Tc_b;
        T_c = T_out * g_st * Te * Rp_corr_T; // ^s_T_{cam_EE} = ^s_T_{cam_base} * {cam_base}_T_{cam_EE} * coord-axis

#   if (FEATURE_ENABLE_ARM_ODOMETRY_ZEROING) // FIXME this compensation is not right
        if (this->arm_prev_data.arm_pose_ready && !this->arm_prev_data.arm_pose_inited)
        {
            this->arm_prev_data.arm_pose_inited = true;
            this->arm_prev_data.arm_pose_st_0 = Lie::inverse_SE3(T_out);
        }

        // 3. apply compensation
        T_out = this->arm_prev_data.arm_pose_st_0 * T_out;
        T_c = T_out * g_st * Te * Rp_corr_T;
#   endif

        pArm->storeTransformations_unsafely(T_out);

        this->arm_prev_data.arm_pose_st = T_c;
        this->arm_prev_data.arm_pose_header = arm_prev_data.t_header;

        // PRINT_DEBUG("> ArmOdometry [%f]: \n R=\n%s \n p=\n%s", t, Lie::to_string(R).c_str(), Lie::to_string(p).c_str());

#   if(FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
        // publish immediately:
        Lie::SO3xR3_from_SE3(R_c, p_c, T_c); // reuse placeholder R_c, p_c
        queue_ArmOdometry_safe(arm_prev_data.t_header, R_c, p_c, EE_DEV);
#   endif //(FEATURE_ENABLE_ARM_ODOMETRY_VIZ)
    }
    else
    {
        PRINT_WARN("ArmOdometry: no previous data available!");
    }
    // cache, compute next time:
    arm_prev_data.t_header = t;
    arm_prev_data.arm_vec = jnt_vec;
}
#endif 

void EstimatorManager::inputImage(double t_b, const cv::Mat &_img_b, double t_e, const cv::Mat &_img_e)
{
    // input images:
    pEsts[BASE_DEV]->inputImage(t_b, _img_b);
    pEsts[EE_DEV  ]->inputImage(t_e, _img_e);
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
#if (FEATURE_ENABLE_VICON_SUPPORT)
            pubViconOdometryPath_safe(dev_id);
#endif // (FEATURE_ENABLE_VICON_SUPPORT)
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
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)
            // print evaluation for optimization process for every estimator:
            const double par_k = pEsts[dev_id]->perf.opt_margin_key * 100.0 / pEsts[dev_id]->perf.opt_total_count;
            const double par_nk = pEsts[dev_id]->perf.opt_margin_not_key * 100.0 / pEsts[dev_id]->perf.opt_total_count;
            PRINT_DEBUG("[%d] Est Optimization Perf: [Key: %.2f\%, NotKey: %.2f\%]", dev_id, par_k, par_nk);
#endif // (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)
        }
#if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
        pubArmModel_safe(pArm);
#endif
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
