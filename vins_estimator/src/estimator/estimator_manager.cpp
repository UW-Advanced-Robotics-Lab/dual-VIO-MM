#include "estimator_manager.h"
#include "../robot/Lie.h"
// #include "../utility/pool.hpp"

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
    pEsts[BASE_DEV]->clearState_safe();
    pEsts[EE_DEV  ]->clearState_safe();
    pEsts[BASE_DEV]->setParameter_safe();
    pEsts[EE_DEV  ]->setParameter_safe();

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
    this->arm_prev_data.t_header = -1;
    this->arm_prev_data.arm_pose_header = -1;
    this->arm_prev_data.arm_pose_st_0 = Lie::SE3::Identity();
    this->arm_prev_data.arm_pose_ts_0 = Lie::SE3::Identity();
    this->arm_prev_data.arm_pose_st_inited = false;
    this->arm_prev_data.arm_pose_ts_inited = false;
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

                // PRINT_INFO("pEsts[BASE_DEV]->_status : %d", pEsts[BASE_DEV]->_status);
                // PRINT_INFO("pEsts[  EE_DEV]->_status : %d", pEsts[  EE_DEV]->_status);
#  if (FEATURE_ENABLE_ARM_ODOMETRY_TO_POSE_INIT)
//                 // Pose Initialization based on arm:
                // based on previous status:
                if (    ((pEsts[  EE_DEV]->_status & Estimator::STATUS_NON_LINEAR_OPT) && (pEsts[BASE_DEV]->_status & Estimator::STATUS_INITIALIZED))
                    ||  ((pEsts[BASE_DEV]->_status & Estimator::STATUS_NON_LINEAR_OPT) && (pEsts[  EE_DEV]->_status & Estimator::STATUS_INITIALIZED)) )
                {
                    Lie::SE3 pose = Lie::inverse_SO3xR3(m_data.last_R[EE_DEV], m_data.last_P[EE_DEV]);
                    pEsts[EE_DEV]->initFirstBodyPose(pose);
                    PRINT_DEBUG("[EE<-BASE:] initFirstPose: \nST:\n%s\nTS:\n%s", 
                        Lie::to_string(this->arm_prev_data.arm_pose_st).c_str(),
                        Lie::to_string(this->arm_prev_data.arm_pose_ts).c_str());
                }
                else
                {
                    // DO NOTHING;
                }
#  endif //! (FEATURE_ENABLE_ARM_ODOMETRY_TO_IMU_INIT)
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
                            // process IMU: 
                            pEsts[id]->processIMU(accVector[id][i].first, dt, accVector[id][i].second, gyrVector[id][i].second);
                        }
                    }
                    TOK_TAG(ellapsed_process_i,"imu_proc");

                    //TODO: around 20ms x2 = 40ms, we should parallel these two processes? [@later, @run-time-concern]
                    bool _success[MAX_NUM_DEVICES] = {false, false};
                    // quickpool::parallel_for(BASE_DEV, MAX_NUM_DEVICES, [&] (int id) { // NOTE: not so well
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                    {
                        // process Image:
                        // NOTE: process and publish has to be under same lock?
                        pEsts[id]->mProcess.lock();
                        TIK(ellapsed_process_i);
#if (FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE)
                        Lie::SE3 pose = (id == EE_DEV)?arm_prev_data.arm_pose_st:arm_prev_data.arm_pose_ts;
#else
                        Lie::SE3 pose = arm_prev_data.arm_pose_st;
#endif //(!FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE)
                        pEsts[  EE_DEV]->arm_inited = this->arm_prev_data.arm_pose_st_inited; // indicates if valid
                        pEsts[BASE_DEV]->arm_inited = this->arm_prev_data.arm_pose_ts_inited; // indicates if valid
                        _success[id] = pEsts[id]->processImage(feature[id].second, feature[id].first, pose);
                        prevTime[id] = curTime[id];
                        pEsts[id]->mProcess.unlock();
                        
                        // in case of failure, restart @ optimization failure:  
                        if (_success[id] == false)
                        {
                            if (pEsts[id]->_status & Estimator::STATUS_NON_LINEAR_OPT)
                            {
                                ROS_WARN("failure detection!");
                                PRINT_ERROR("[DEV:%d] failure detection!", id);
                                // TODO: clear state
                                pEsts[id]->clearState_safe();
                                pEsts[id]->setParameter_safe(); // reinit
                                PRINT_WARN("[DEV:%d] system reboot!", id);
                                // device will re-initialize
                                //TODO:
                                // this->arm_prev_data.arm_pose_st_0 = Lie::SE3::Identity();
                                // this->arm_prev_data.arm_pose_ts_0 = Lie::SE3::Identity();
                                // this->arm_prev_data.arm_pose_st_inited = false;
                                // this->arm_prev_data.arm_pose_ts_inited = false;
                            }
                            else if (pEsts[id]->_status & Estimator::STATUS_INITIALIZING)
                            {
                                PRINT_ERROR("[DEV:%d] failed to initialize!", id);
                            }
                        }
                        // PRINT_INFO("pEsts[%d]->_status : %d", id, pEsts[id]->_status);
                        
                        // store latest result: (only valid after initialization)
                        //   -> used by _postProcessArmJnts_unsafe() at the start of next run
                        if (pEsts[id]->_status & Estimator::STATUS_UPDATED) 
                        {
                            m_data.last_R[id] = pEsts[id]->last_R;
                            m_data.last_P[id] = pEsts[id]->last_P;
                            m_data.last_RP_ready[id] = true;
                        }
                        else if (pEsts[id]->_status & Estimator::STATUS_INITIALIZED) 
                        {
                            // for initialization:
                            m_data.last_R[id] = pEsts[id]->latest_R;
                            m_data.last_P[id] = pEsts[id]->latest_P;
                            m_data.last_RP_ready[id] = true;
                        }
                        else
                        {
                            m_data.last_RP_ready[id] = false;
                        }
                    }
                    // });
                    TOK_TAG(ellapsed_process_i,"img_proc");

                    /*** Publish ***/
                    // publish only if successfully processed previously
                    TIK(ellapsed_process_i);
                    for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES && _success[id]; id++)
                    {
                        pEsts[id]->mProcess.lock();
#if (FEATURE_ENABLE_STATISTICS_LOGGING)
                        // Print Statistics:
                        printStatistics(*pEsts[id], 0);
#endif //(FEATURE_ENABLE_STATISTICS_LOGGING)
                        // Publish:
                        std_msgs::Header header;
                        header.frame_id = "world";
                        header.stamp = ros::Time(feature[id].first);
                        // immediate updates (used by loop fusion):
                        {
                            pubKeyframe_Odometry_and_Points_immediately(*pEsts[id]);
                        }
#if (FEATURE_VIZ_PUBLISH)
                        // visualization updates:
                        visualization_guard_lock(*pEsts[id]);
                        {
                            queue_TF_unsafe(*pEsts[id], header);
                            queue_Odometry_unsafe(*pEsts[id], header);
                            queue_KeyPoses_unsafe(*pEsts[id], header);
                            queue_CameraPose_unsafe(*pEsts[id], header);
                            queue_PointCloud_unsafe(*pEsts[id], header);
                        }
                        visualization_guard_unlock(*pEsts[id]);
#endif //(FEATURE_VIZ_PUBLISH)
                        pEsts[id]->mProcess.unlock();
                    }
                    TOK_TAG(ellapsed_process_i,"img_pub");
                }
                else
                {
                    PRINT_WARN("wait for both imu and arm [%d|%d|%d] ... \n", is_base_imu_avail, is_EE_imu_avail, is_arm_avail);
                }
#if (FEATURE_ENABLE_VICON_SUPPORT)
                // TODO: should we sync against the image to have alignment evaluation? (or a separate topic)
                // queue vicon to publisher later:
                for(size_t id = BASE_DEV; id < MAX_NUM_DEVICES; id++)
                {
#   if (FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_SFM)
                //NOTE: If Enabled, the vicon will aligned based on the first initialization, skipping all initialization process
#       if (FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_BOTH_SFM)
                    if ((pEsts[BASE_DEV]->_status & Estimator::STATUS_INITIALIZING) &&
                        (pEsts[  EE_DEV]->_status & Estimator::STATUS_INITIALIZING)) 
#       else
                    if (pEsts[id]->_status & Estimator::STATUS_INITIALIZING)
#       endif //(FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_BOTH_SFM)
                    {
                        // do not process till SFM initialized
                        continue; //
                    }
#   endif //(FEATURE_ENABLE_VICON_ONLY_AFTER_INIT_SFM)
                    this->m_vicon[id].guard.lock();
                    while(! this->m_vicon[id].data.empty())
                    {
                        // vicon time alignment (aligning with the imaging accusition time)
                        if (this->m_vicon[id].data.front().first < feature[id].first)
                        {
                            this->m_vicon[id].data.pop(); // popping :)
                        }
                        else
                        {
                            queue_ViconOdometry_safe(this->m_vicon[id].data.front().second, this->m_vicon[id].data.front().first, id);
                            break; // break the while loop
                        }
                    }
                    this->m_vicon[id].guard.unlock();
                }
#endif //(FEATURE_ENABLE_VICON_SUPPORT)
            }
            else
            {
                // When no images available, do nothing
            }

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
    const Lie::SE3 Tc_b = Lie::inverse_SE3(Tb); // robot axis
    const Lie::SE3 Tc_e = Lie::inverse_SE3(Te);

    // Constant rotation correction, [you may use https://ninja-calc.mbedded.ninja/calculators/mathematics/geometry/3d-rotations]
    const Lie::SO3 R_corr_c2w(
        (Lie::SO3() <<  1,  0,  0,
                        0,  0, -1,
                        0,  1,  0).finished()
    ); // convert from camera axis back to world axis
    const Lie::SO3 R_corr_w2r(
        (Lie::SO3() <<  0, -1,  0,
                        1,  0,  0,
                        0,  0,  1).finished()
    ); // convert from world axis back to robot axis
    const Lie::SE3 T_corr_r2w(
        (Lie::SE3() <<  0,  1,  0, 0, 
                       -1,  0,  0, 0, 
                        0,  0,  1, 0,
                        0,  0,  0, 1 ).finished()
    ); // convert from robot axis back to world axis 
    const Lie::SE3 T_corr_r2c(
        (Lie::SE3() <<  0,  0,  1,  0,
                       -1,  0,  0,  0,
                        0, -1,  0,  0, 
                        0,  0,  0,  1 ).finished()
    ); // convert from robot axis back to base cam axis

    // [Arm] compute current theta --> SE3:
    // pArm->setAngles_unsafely(Vector7d_t::Zero());
    pArm->setAngles_unsafely(jnt_vec, t);
    pArm->processJntAngles_unsafely();

    // compute:
# if (FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE)
    const bool if_estimator_ready[MAX_NUM_DEVICES] = {
        (m_data.last_RP_ready[BASE_DEV] == true), 
        (m_data.last_RP_ready[EE_DEV  ] == true)
    };
# else //(!FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE)
    const bool if_estimator_ready[MAX_NUM_DEVICES] = {
        (m_data.last_RP_ready[BASE_DEV] == true), 
        (false) // Disable
    };
# endif 
    if ((arm_prev_data.t_header > 0))
    {
        // placeholders:
        Lie::SE3 T_c, T_b, T_e, T_c2, T_b2;
        Lie::R3 p_c, p_c2; 
        Lie::SO3 R_c, R_c2;
        Lie::SE3 g_st, b_st;
        T_b = Lie::SE3::Identity();
        
        /* 1. Fetch Pose Reference */
        g_st = arm_prev_data.arm_g_st;
        
#   if (FEATURE_ENABLE_ARM_VICON_SUPPORT) 
        // stub previously published Ground Truth Vicon data:
        geometry_msgs::Pose pose;
        if (if_estimator_ready[BASE_DEV])
        {
            getLatestViconPose_safe(pose, BASE_DEV);
            p_c = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
            R_c = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
            T_c = Lie::SE3_from_SO3xR3((R_c * R_corr_w2r), p_c); // vicon is aligned with world axis -> robot axis
        }
        else
        {
            T_c = Lie::SE3_from_SO3xR3((R_corr_c2w * R_corr_w2r), p_c);
        }
        if (if_estimator_ready[EE_DEV])
        {
            getLatestViconPose_safe(pose, EE_DEV);
            p_c2 = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
            R_c2 = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
            T_c2 = Lie::SE3_from_SO3xR3((R_c2 * R_corr_w2r), p_c2); // robot axis
        }
        else
        {
            T_c2 =Lie::SE3_from_SO3xR3((R_corr_w2r), p_c);
        }
#   else 
        // feed in previously optimized estimator latest state:
        if (if_estimator_ready[BASE_DEV])
        {
            p_c = m_data.last_P[BASE_DEV]; // previous frame
            R_c = m_data.last_R[BASE_DEV]; // previous frame
            // convert frame axis -(R_corr_c2w)-> world frame axis -(R_corr_w2r)-> robot frame axis:
            T_c = Lie::SE3_from_SO3xR3(((R_c * R_corr_c2w) * R_corr_w2r), p_c);// a coordinate transformation is needed
        }
        else
        {
            T_c = Lie::SE3_from_SO3xR3(R_corr_c2w * R_corr_w2r, Lie::R3::Zero());
        }
        if (if_estimator_ready[EE_DEV])
        {
            p_c2 = m_data.last_P[EE_DEV]; // previous frame
            R_c2 = m_data.last_R[EE_DEV]; // previous frame
            T_c2 = Lie::SE3_from_SO3xR3((R_c2 * R_corr_w2r), p_c2); // robot axis
        }
        else
        {
            T_c2 = Lie::SE3_from_SO3xR3(R_corr_w2r, Lie::R3::Zero());
        }
#   endif // (!FEATURE_ENABLE_ARM_VICON_SUPPORT)

        /* 2. Apply Arm Pose */
        /* BASE --> EE */
        {
            // Lie::SE3 T_out = T_c; // summit base if summit_base
            T_b = T_c * Tc_b; // cam_base
            // ^s_T_{cam_EE} = ^s_T_{cam_base} * {cam_base}_T_{cam_EE}  
            T_e = T_b * g_st * Te * T_corr_r2w; // world axis = cam axis (ee camera axis @ 0config)
        }
        /* EE --> BASE */
        {
            // camera @ EE calib @ zero config is aligned with world axis
            b_st = Lie::inverse_SE3(g_st);
            T_b2 = T_c2; // to ee
            T_b2 = T_b2 * b_st; // to base
            T_b2 = T_b2 * Tb; // inv(Tc_b) to base camera
#   if(FEATURE_ENABLE_ARM_VICON_SUPPORT)
            T_b2 = T_b2 * T_corr_r2w; // world axis
#   else
            T_b2 = T_b2 * T_corr_r2c; // camera axis
#   endif //(FEATURE_ENABLE_ARM_VICON_SUPPORT)
        }
        
#   if (!FEATURE_ENABLE_ARM_ODOMETRY_WRT_TO_BASE) 
        /* 2. Apply Zeroing Compensation */
        if (if_estimator_ready[BASE_DEV] && if_estimator_ready[  EE_DEV])
        {
            /* BASE --> EE */
            if (!this->arm_prev_data.arm_pose_st_inited)
            {   
                this->arm_prev_data.arm_pose_st_inited = true;
                this->arm_prev_data.arm_pose_st_0 = T_c2 * T_corr_r2w * Lie::inverse_SE3(T_e); // world/cam_EE
    #   if (FEATURE_ENABLE_ARM_ODOMETRY_POSE_ZERO_ENFORCE_SO2)
                double yaw = Utility::R2y_rad(this->arm_prev_data.arm_pose_st_0.block<3,3>(0,0));
                this->arm_prev_data.arm_pose_st_0.block<3,3>(0,0) = Utility::y2R_rad(yaw); // to avoid pitch/roll bias
    #   endif //(FEATURE_ENABLE_ARM_ODOMETRY_POSE_ZERO_ENFORCE_SO2)
                PRINT_DEBUG("this->arm_prev_data.arm_pose_st_0:\n%s\n", Lie::to_string(this->arm_prev_data.arm_pose_st_0).c_str());
                PRINT_DEBUG("T_c2\n%s\n", Lie::to_string(T_c2).c_str());
            }
            
            /* EE --> BASE */
            if (!this->arm_prev_data.arm_pose_ts_inited)
            {
                this->arm_prev_data.arm_pose_ts_inited = true;
                // do the same for base:
    #   if(FEATURE_ENABLE_ARM_VICON_SUPPORT)
                this->arm_prev_data.arm_pose_ts_0 = T_c * T_corr_r2w * Lie::inverse_SE3(T_b2); // world
    #   else
                this->arm_prev_data.arm_pose_ts_0 = T_c * T_corr_r2c * Lie::inverse_SE3(T_b2); // cam_base
    #   endif //(FEATURE_ENABLE_ARM_VICON_SUPPORT)
    #   if (FEATURE_ENABLE_ARM_ODOMETRY_POSE_ZERO_ENFORCE_SO2)
                double yaw = Utility::R2y_rad(this->arm_prev_data.arm_pose_ts_0.block<3,3>(0,0));
                this->arm_prev_data.arm_pose_ts_0.block<3,3>(0,0) = Utility::y2R_rad(yaw); // to avoid pitch/roll bias
    #   endif //(FEATURE_ENABLE_ARM_ODOMETRY_POSE_ZERO_ENFORCE_SO2)
                PRINT_DEBUG("this->arm_prev_data.arm_pose_ts_0:\n%s\n", Lie::to_string(this->arm_prev_data.arm_pose_ts_0).c_str());
                PRINT_DEBUG("T_c\n%s\n", Lie::to_string(T_c).c_str());
            }
        }

        // Compensation:
        T_e = this->arm_prev_data.arm_pose_st_0 * T_e; 
        T_b = this->arm_prev_data.arm_pose_st_0 * T_b; 
        T_b2 = this->arm_prev_data.arm_pose_ts_0 * T_b2; 
#   endif // (FEATURE_ENABLE_ARM_ODOMETRY_ZEROING) 
        

        /* BASE --> EE */
        {            
            // - update arm pose:
            this->arm_prev_data.arm_pose_st = T_e;
            this->arm_prev_data.arm_pose_header = arm_prev_data.t_header;
            // PRINT_DEBUG("> ArmOdometry [%f]: \n R=\n%s \n p=\n%s", t, Lie::to_string(R).c_str(), Lie::to_string(p).c_str());
        }

        /* EE --> BASE */
        {
# if (FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE_ENFORCE_SE2)
            // for base, project to SO2:
            // SO3 --proj--> SO2:
            double yaw = Utility::R2y_rad(T_b2.block<3,3>(0,0));
            T_b2.block<3,3>(0,0) = Utility::y2R_rad(yaw) * R_corr_c2w.transpose(); // camera axis
            T_b2(2,3) = 0;
# endif //(FEATURE_ENABLE_ARM_ODOMETRY_EE_TO_BASE_ENFORCE_SO2)
            this->arm_prev_data.arm_pose_ts = T_b2;
        }

        /* Queue for rostopic visualization via rviz */
# if(FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM) // arm viz 
        pArm->storeTransformations_unsafely(T_b); // for arm visualization
# endif //(FEATURE_ENABLE_ARM_ODOMETRY_VIZ_ARM)
# if (FEATURE_ENABLE_ARM_ODOMETRY_VIZ) // viz pub
        if (this->arm_prev_data.arm_pose_st_inited) // indicates if it is valid
        {
            Lie::SO3xR3_from_SE3(R_c, p_c, T_e); // reuse placeholder R_c, p_c
            queue_ArmOdometry_safe(arm_prev_data.t_header, R_c, p_c, EE_DEV);
        }
        if (this->arm_prev_data.arm_pose_ts_inited)
        {
            Lie::SO3xR3_from_SE3(R_c2, p_c2, T_b2); // reuse placeholder R_c2, p_c2
            queue_ArmOdometry_safe(arm_prev_data.t_header, R_c2, p_c2, BASE_DEV);
        }
# endif // (FEATURE_ENABLE_ARM_ODOMETRY_VIZ) // viz pub
    }
    else
    {
        PRINT_WARN("ArmOdometry: no previous data available!");
    }
    // cache, we will use them next time:
    if (pArm->getEndEffectorPose_unsafely(arm_prev_data.arm_g_st))
    {
        arm_prev_data.t_header = t;
        arm_prev_data.arm_vec = jnt_vec;
    }
    else
    {
        PRINT_ERROR("ArmOdometry: failed to compute end-effector pose!");
    }
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
            pubOdometry_safe(dev_id);
            pubOdometryPath_safe(dev_id);
            pubExtrinsic_TF_safe(dev_id);
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
