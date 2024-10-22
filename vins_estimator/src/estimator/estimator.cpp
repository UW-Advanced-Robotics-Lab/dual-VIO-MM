/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "estimator_manager.h"
#include "../utility/visualization.h"
#include "../robot/Lie.h"

Estimator::Estimator(
        std::shared_ptr<DeviceConfig_t> _pCfg
    ): pCfg{_pCfg}, f_manager{Rs,_pCfg}, featureTracker{_pCfg}
{
    ROS_INFO("init begins");
    clearState_safe();
    poseInitCounter = 0;
#if (FEATURE_NON_THREADING_SUPPORT)
    assert("> Assume threading supported all the time!");
#endif
#if (FEATURE_PERMIT_WITHOUT_IMU_SUPPORT)
    assert("> Assume IMU must exist all the time!");
#endif
}

Estimator::~Estimator()
{
    // process removed from here
    // detaching pointers:
    pCfg.reset();
}

void Estimator::clearState_safe()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter_safe()
{
	const double FOCAL_LENGTH = pCfg->FOCAL_LENGTH;
    mProcess.lock();
    for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
    {
        tic[i] = pCfg->TIC[i];
        ric[i] = pCfg->RIC[i];
        PRINT_INFO("Extrinsic Camera [%d] \n R_ic=\n %s\n p_ic=\n %s\n", i, 
            Lie::to_string(ric[i]).c_str(), Lie::to_string(tic[i].transpose()).c_str());
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = pCfg->TD;
    g = pCfg->G;
    PRINT_INFO("Gravity: g= %s\n", Lie::to_string(g.transpose()).c_str());
    featureTracker.readIntrinsicParameterArray(pCfg->CAM_MODEL_PATH, pCfg->NUM_OF_CAM);
    mProcess.unlock();
}

// #if (FEATURE_PERMIT_WITHOUT_IMU_SUPPORT)
// void Estimator::changeSensorType(int use_imu, int use_stereo)
// {
//     bool restart = false;
//     mProcess.lock();
//     if(!use_imu && !use_stereo)
//         PRINT_WARN("at least use two sensors! \n");
//     else
//     {
//         if(pCfg->USE_IMU != use_imu)
//         {
//             pCfg->USE_IMU = use_imu;
//             if(pCfg->USE_IMU)
//             {
//                 // reuse imu; restart system
//                 restart = true;
//             }
//             else
//             {
//                 if (last_marginalization_info != nullptr)
//                     delete last_marginalization_info;

//                 tmp_pre_integration = nullptr;
//                 last_marginalization_info = nullptr;
//                 last_marginalization_parameter_blocks.clear();
//             }
//         }
        
//         pCfg->STEREO = use_stereo;
//         PRINT_WARN("use imu %d use stereo %d\n", pCfg->USE_IMU, pCfg->STEREO);
//     }
//     mProcess.unlock();
//     if(restart)
//     {
//         clearState_safe();
//         setParameter_safe();
//     }
// }
// #endif

void Estimator::inputImage(double t, const cv::Mat &_img)
{
    // processing image:
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    featureFrame = featureTracker.trackImage(t, _img);
    TOK(featureTrackerTime);

    // push feature buffer , threading
    {     
#if (FEATURE_ENABLE_PROCESS_FRAME_FPS_FOR_RT_INSIDE_INPUT_IMG) 
        if(inputImageCnt % 2 == 0) // NOTE: idk why it is dropping frame here, but likely is to minimize the rate of processing
#endif
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }

#if (FEATURE_TRACKING_IMAGE_SUPPORT)
    // publish:
    // if ((pCfg->SHOW_TRACK) && (inputImageCnt % 2 == 0))// publish at lower rate
    if (pCfg->SHOW_TRACK)// queue safely
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        queue_TrackImage_safe(imgTrack, t, pCfg->DEVICE_ID);
    }
#endif
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
#if (FEATURE_ROS_PUBLISH_IMU_PROPAGATION) // Publish for debugging
        pubLatestOdometry_immediately(latest_P, latest_Q, latest_V, t, pCfg->DEVICE_ID);
#endif // (FEATURE_ROS_PUBLISH_IMU_PROPAGATION)
        mPropagate.unlock();
    }
}

// void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
// {
//     mBuf.lock();
//     featureBuf.push(make_pair(t, featureFrame));
//     mBuf.unlock();

// // #if (FEATURE_NON_THREADING_SUPPORT)
// //     if(!pCfg->MULTIPLE_THREAD)
// //         processMeasurements();
// // #endif
// }


bool Estimator::getIMUInterval_unsafe(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        PRINT_ERROR("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            gyrVector.push_back(gyrBuf.front());
            accBuf.pop();
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        PRINT_WARN("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

#ifndef ESTIMATOR_MANAGER_H // if manager does not exist, then this is the main estimator processing thread:
void Estimator::processMeasurements_thread()
{
    TicToc ellapsed_process;
    while (FOREVER)
    {
        TIK(ellapsed_process);
        this->processMeasurements_once();
        TOK(ellapsed_process);

        // #if (FEATURE_NON_THREADING_SUPPORT)
        //         if (! pCfg->MULTIPLE_THREAD)
        //             break;
        // #endif
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::processMeasurements_once()
{
    //printf("process measurments\n");
    pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
    vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
    if(!featureBuf.empty())
    {
        mBuf.lock();
        feature = featureBuf.front();
        featureBuf.pop();
        // PRINT_DEBUG("feature buffer size: %d\n",featureBuf.size());
        mBuf.unlock();

        curTime = feature.first + td;
        while (! IMUAvailable(curTime)) //IMU Support
        {
            
            PRINT_WARN("wait for imu ... \n");
            std::chrono::milliseconds dura(1);
            std::this_thread::sleep_for(dura);
        }
        mBuf.lock();
        getIMUInterval_unsafe(prevTime, curTime, accVector, gyrVector); //IMU Support
        mBuf.unlock();

        // if(pCfg->USE_IMU) IMU Support
        {
            if(!initFirstPoseFlag)
                initFirstIMUPose(accVector);
            for(size_t i = 0; i < accVector.size(); i++)
            {
                double dt;
                if(i == 0) // head
                    dt = accVector[i].first - prevTime;
                else if (i == accVector.size() - 1)
                {
                    // only sample the partial of the tail imu data up to the time the frame was captured
                    dt = curTime - accVector[i - 1].first;
                    // double dt2 = accVector[i].first - accVector[i - 1].first;
                    // PRINT_DEBUG("dt=%f|%f, ~%f", dt, dt2, dt2-dt); // diff is 0.001s
                }
                else // middle
                    dt = accVector[i].first - accVector[i - 1].first;
                processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
            }
        }
        // Process Image and Publish:
        {
            mProcess.lock();
            
            // Process Image: 
            processImage(feature.second, feature.first, nullptr);
            prevTime = curTime;

#if (FEATURE_ENABLE_STATISTICS_LOGGING)
            // Print Statistics:
            printStatistics(*this, 0);
#endif

            /*** Publish ***/
            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);
            // immediate updates:
            {
                pubKeyframe_Odometry_and_Points_immediately(*this);
                pubTF_immediately(*this, header);
                pubOdometry_Immediately(*this, header);
            }
            // visualization updates:
            visualization_guard_lock(*this);
            {
                queue_KeyPoses_unsafe(*this, header);
                queue_CameraPose_unsafe(*this, header);
                queue_PointCloud_unsafe(*this, header);
            }
            visualization_guard_unlock(*this);
            
            // End-of-publishing
            mProcess.unlock();
        }
    }
}
#endif // !ESTIMATOR_MANAGER_H

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    PRINT_WARN("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    PRINT_WARN("[%d] averge acc %f %f %f\n", this->pCfg->DEVICE_ID, averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    PRINT_INFO("[%d] init Rs[0]:\nT=\n%s", this->pCfg->DEVICE_ID, Lie::to_string(Rs[0]).c_str());
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstBodyPose(Lie::SE3 &Rp)
{
    poseInitCounter ++;
    Lie::SO3xR3_from_SE3(initR, initP, Rp); // cache initialization
    PRINT_INFO("[%d] init first pose:\nT=\n%s", this->pCfg->DEVICE_ID, Lie::to_string(Rp).c_str());

    double yaw = Utility::R2ypr(initR).x();
    initR = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * initR;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
            // Headers[i] = Headers[i + 1];
            Rs[i] = initR;
            // Ps[i] = initP;
    }
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
#if (FEATURE_ASSUME_INIT_IMU_TO_BE_ZEROED_ABSOLUTELY)
        acc_0 = Vector3d::Zero();
        gyr_0 = Vector3d::Zero();
#else
        // zeroing with respect to current a and w 
        assert(false, "[DO NOT USE] Invalid assumption: init imu to be zeroed absolutely!");
        // If true, system may drift at stopping point
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
#endif
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count], pCfg};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // midPointIntegration in global frame:
        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

bool Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header, const Lie::SE3 pT_arm, const bool pT_arm_valid)
{
    TicToc t_solver;
    this->_status = STATUS_EMPTY; // reset status
    bool success = true;
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        this->_status |= (STATUS_KEYFRAME);
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        this->_status ^= (STATUS_KEYFRAME);
        //printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration; // assign previous pre-integration
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count], pCfg}; // prep for next image

#if (FEATURE_ENABLE_CALIBRATION)
    if(pCfg->ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                pCfg->RIC[0] = calib_ric;
                pCfg->ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
#else
    assert((pCfg->ESTIMATE_EXTRINSIC != 2));//, "Extrinsic Estimation To be Implemented!");
#endif // (FEATURE_ENABLE_STEREO_SUPPORT)

    // A) Initialization from SfM:
    if (solver_flag == INITIAL) 
    {
        this->_status |= (STATUS_INITIALIZING);

#if (FEATURE_ENABLE_CALIBRATION)
    // TODO: add support for overlapped region
#   if (FEATURE_ENABLE_STEREO_SUPPORT) // TODO: support for stereo (for overlap regions of two cameras)
        // monocular + IMU initilization
// #if (FEATURE_ENABLE_STEREO_SUPPORT) // condition only necessary for stereo support 
//         if (!pCfg->STEREO && pCfg->USE_IMU) 
// #endif
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(pCfg->ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    ROS_INFO("Initialization finish!");
                }
                slideWindow();
            }
        }
        // stereo + IMU initilization
        if(pCfg->STEREO && pCfg->USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(pCfg->STEREO && !pCfg->USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }
#   endif
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            const int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
#else
        // pre-requisite assumption:
        assert((pCfg->STEREO == false));//, "Only Monocular!");
        assert((pCfg->USE_IMU == true));//, "Only Monocular with IMU!");

        // queue window before initializing:
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            const int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
        // only init, when new frame incoming while buffer is full! (i.e. else if)
        else if (frame_count == WINDOW_SIZE) 
        {
            // monocular + IMU initilization:
            if((header - initial_timestamp) > 0.1)
            {
                success = initialStructure();
                initial_timestamp = header;
                if(success)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    this->_status |= (STATUS_INITIALIZED);
                    ROS_INFO("Initialization finish!");
                    PRINT_WARN("[%d] Initialization finish!", pCfg->DEVICE_ID);
                    this->inited_R0_T = latest_R.transpose();
                    this->inited_P0   = latest_P;
                }
                else
                {
                    PRINT_WARN("[%d] Initialization failed!", pCfg->DEVICE_ID);
                    // DO NOT Clear State , else the initialization will be based on the current state?
                    // TODO: we should clear state, and reinitialize wrt base
                }
            }
            slideWindow();
            PRINT_INFO("[%d] SlideWindow : %s", pCfg->DEVICE_ID, (marginalization_flag == MARGIN_OLD)?"OLD":"NEW");
        }
#endif // NO CALIBRATION SUPPORT
    
    }
    else
    // B) PnP tracking:
    {
        this->_status |= (STATUS_NON_LINEAR_OPT);

#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
        Lie::SO3xR3_from_SE3(arm_Rs[frame_count-1], arm_Ps[frame_count-1], pT_arm); 
        arm_valid[frame_count-1] = pT_arm_valid;
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)

        TIK(t_solver);
        f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric); // IMU only supports
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        TOK(t_solver);

        if (failureDetection())
        {
#ifndef ESTIMATOR_MANAGER_H // Original, but it has deadlocking
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter(); // reinit
            ROS_WARN("system reboot!");
            return;
#else // Let Manager to do the reset safely
            failure_occur = 1;
            success = false;
#endif //ESTIMATOR_MANAGER_H
        }
        else
        {
            slideWindow();
            f_manager.removeFailures();
            // prepare output of VINS
            key_poses.clear();
            for (int i = 0; i <= WINDOW_SIZE; i++)
                key_poses.push_back(Ps[i]);

            last_R = Rs[WINDOW_SIZE];
            last_P = Ps[WINDOW_SIZE];
            last_R0 = Rs[0];
            last_P0 = Ps[0];
            updateLatestStates();
            this->_status |= STATUS_UPDATED;
        }
    }
    return success;
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    /**************************
     * check imu observibility
     * ************************/
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1)); //TODO: [opt] run-time optimization: DO NOT USE sqrt();
        //ROS_WARN("IMU variation %f!", var);
        if(var < ESTIMATOR_CPP_IMU_EXCITATION_STRICT_MIN_VAR_THRESHOLD) //if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    
    /**************************
     * check visual observibility
     * ************************/
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];

    // - collection of x,y feature points into feature vector for each frame:
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;//TODO: [opt] move imu_j++ after loop, so minimize subtraction
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 

    // - Check Feature Parallax:
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        PRINT_ERROR("Not enough features or parallax; Move device around");
        return false;
    }
    
    /**************************
     * Construct SfM problem
     * ************************/
    // - Construct SfM:
    GlobalSFM sfm;
    map<int, Vector3d> sfm_tracked_points;

    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        PRINT_ERROR("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }
    
    /**************************
     * Solve SfM problem
     * ************************/
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    // - iterate through all feature images:
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        if((frame_it->first) == Headers[i]) // if image frames == header@i frame
        {
            frame_it->second.is_key_frame = true;
            // correct camera orientation --> align with imu body frame orientation:
            frame_it->second.R = Q[i].toRotationMatrix() * pCfg->RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i]) // if image frames appear later than header@i frame
        {
            i++;
        }

        // init:
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        // - iterate through feature_points <feature_id, vector of mapped points>
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            it = sfm_tracked_points.find(feature_id); 
            if(it != sfm_tracked_points.end())
            {
                Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                // iterate through corresponding image points (vector of mapped points)
                for (auto &i_p : id_pts.second) // i_p : <camera_id, xyz_uv_velxy>
                {
                    cv::Point3f pts_3_copy = pts_3; // (make a copy)
                    pts_3_vector.push_back(pts_3_copy);
                    Vector2d img_pts = i_p.second.head<2>(); // x,y (undistorted points)
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        // - minimum 6 pairs of 3d & 2d points to solve pnp:
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            PRINT_ERROR("Not enough points for solve pnp !");
            return false;
        }
        // - solve:
        // Placeholders:
        cv::Mat rvec, tvec; // placeholder result
        cv::Mat r, tmp_r; // buffer
        // Constants:
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat D; // no distortion
        // Initial Guess:
        const bool ININITAL_GUESS = true;
        // translate camera frame (world) --> camera frame (body)
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        // translate init guess variables:
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, tvec);
        // [solve PnP in camera space]
        //cv::solvePnP (InputArray objectPoints, InputArray imagePoints, 
        //      InputArray cameraMatrix, InputArray distCoeffs, 
        //      OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=SOLVEPNP_ITERATIVE)
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, tvec, ININITAL_GUESS))
        {
            ROS_DEBUG("solve pnp fail!");
            PRINT_ERROR("solve pnp fail!");
            return false;
        }
        // translate  camera frame (body) --> imu body frame (world)
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(tvec, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * pCfg->RIC[0].transpose(); // apply extrinsic pose correction
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        PRINT_ERROR("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // -> solve Gyro Bias: Bgs
    // -> solve state variable: x = [vel_img_k, g, s]
    //      -> refine Gravity: g
    //      -> solve scale: s
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x, pCfg);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        PRINT_ERROR("solve g failed!");
        return false;
    }

    // change all states up to frame count:
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    
    // repropagate:
    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }

    // relative correction:
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * (Ps[i] - Ps[0]) - (Rs[i] - Rs[0]) * pCfg->TIC[0];
        // Ps[i] = s * Ps[i] - Rs[i] * pCfg->TIC[0] - (s * Ps[0] - Rs[0] * pCfg->TIC[0]);

    // update Vs values:
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    // from gravity to compute yaw on tangent space, and apply the correction to gravity:
    {
        double yaw;
        Eigen::Matrix3d R0;
        // compute rotation between refined gravity and absolute z axis (world frame):
        R0 = Utility::g2R(g);
        yaw = Utility::R2y_rad(R0 * Rs[0]);
        R0 = Utility::y2R_rad(-yaw) * R0;
        // correction:
        g = R0 * g;
        //Matrix3d rot_diff = R0 * Rs[0].transpose();
        Matrix3d rot_diff = R0;
        for (int i = 0; i <= frame_count; i++)
        {
            Ps[i] = rot_diff * Ps[i];
            Rs[i] = rot_diff * Rs[i];
            Vs[i] = rot_diff * Vs[i];
        }
        // debug:
        PRINT_DEBUG("g0     %s", Lie::to_string(g.transpose()).c_str());
        PRINT_DEBUG("my R0  %s", Lie::to_string(Utility::R2ypr(Rs[0]).transpose()).c_str()); 
    }

    // update fM and re-triangulate:
    {
        f_manager.clearDepth();
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    }

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            vector<cv::Point2f> ll, rr;
            for (int j = 0; j < int(corres.size()); j++)
            {
                cv::Point2f pts_0(corres[j].first(0), corres[j].first(1));
                cv::Point2f pts_1(corres[j].second(0), corres[j].second(1));
                ll.push_back(pts_0);
                rr.push_back(pts_1);
                cv::Point2f delta = pts_1 - pts_0;
                // [opt] cv norm does not exist for Point2f, manually implemented instead of eigen norm
                double parallax = cv::sqrt(delta.x * delta.x + delta.y * delta.y); 
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size()); // [NOTE]: originally, it was hard coded 460
            if(average_parallax > pCfg->MIN_PARALLAX_SFM)
            {
                // [Ours]: [opt] reduce a redundant for loop to recollect points again
                bool solve_5pts = m_estimator.solveRelativeRT(ll, rr, relative_R, relative_T, pCfg->RANSAC_THRESHOLD_SFM);
                if (solve_5pts)
                {
                    l = i;
                    ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax, l);
                    return true;
                }
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // if(pCfg->USE_IMU) IMU Support
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    // if(pCfg->USE_IMU) IMU Support
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x(); // compensation --> convert yaw to relative pose
        //TODO: check if this is correct? should be. as yaw is not fully observable, so it should be cumulating, other axis are world frame
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0)); // yaw is relative, not fully observable
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            // yaw is relative:
            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            // relative:
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
// #if (FEATURE_PERMIT_WITHOUT_IMU_SUPPORT)
//     else
//     {
//         for (int i = 0; i <= WINDOW_SIZE; i++)
//         {
//             Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
//             Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
//         }
//     }
// #endif

    // if(pCfg->USE_IMU) IMU Support
    {
        for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    // if(pCfg->USE_IMU) IMU Support
    td = para_Td[0][0];
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization() // TODO: add arm odometry into optimization problem
{
    /*********************************
     * P O S E - G R A P H - O P T.  *
     * *******************************/
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)
    this->perf.opt_total_count ++;
#endif // (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)

    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS); // IMU Support
    }
    problem.SetParameterBlockConstant(para_Pose[0]); // IMU Support

    for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((pCfg->ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }

    problem.AddParameterBlock(para_Td[0], 1); // time delta estimation
    if (!pCfg->ESTIMATE_TD || Vs[0].norm() < 0.2) // 0.2: large excitation only
        problem.SetParameterBlockConstant(para_Td[0]);

    // - Prior / Marginalization
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    // - IMU Residuals
    // if(pCfg->USE_IMU) IMU Support
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j], pCfg->G);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    // - Camera Feature Residuals
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) // TODO: make 4 as a param? also used in feature_manager.cpp
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

// #if (FEATURE_ENABLE_STEREO_SUPPORT) // stereo only TODO: add stereo support
//             if(pCfg->STEREO && it_per_frame.is_stereo)
//             {                
//                 Vector3d pts_j_right = it_per_frame.pointRight;
//                 if(imu_i != imu_j)
//                 {
//                     ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
//                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
//                     problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
//                 }
//                 else
//                 {
//                     ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
//                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
//                     problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
//                 }
               
//             }
// #endif

            f_m_cnt++;
        }
    }
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);

#if (FEATURE_ENABLE_ARM_ODOMETRY_FACTOR)
    // - TODO: Arm Odometry Residuals
# if (FEATURE_ENABLE_ARM_ODOMETRY_WRT_TO_BASE) 
    if (pCfg->DEVICE_ID == EE_DEV)                      // EE only when initialized
# else // (!FEATURE_ENABLE_ARM_ODOMETRY_WRT_TO_BASE) 
#  if (FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_TO_BASE)
    if (this->arm_inited)                               // only if initialized
#  else //(!FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_TO_BASE)
    if (pCfg->DEVICE_ID == EE_DEV && this->arm_inited)  // EE only when initialized
#  endif 
# endif
    {
        // ceres::LossFunction *loss_function_arm;
        // //loss_function = NULL;
        // loss_function_arm = new ceres::HuberLoss(0.1); 
        for (int i = 0; i < frame_count-1; i++)
        {
            if (arm_valid[i]) // only valid posees
            {
#           if (FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_NON_UNIFORM)
                ARMFactor* arm_factor;
                if (pCfg->DEVICE_ID == EE_DEV)
                {
                    // TODO: do not use arm factor if base imu is not stable? / weighted factor
                    const double w = ESTIMATOR_ARM_FACTOR_TO_EE;
                    arm_factor = new ARMFactor(arm_Rs[i],arm_Ps[i],w,w,ESTIMATOR_ARM_FACTOR_TO_EE_Z,ESTIMATOR_ARM_FACTOR_TO_EE_Q);
                }
                else
                {
                    const double w = ESTIMATOR_ARM_FACTOR_TO_BASE;
                    arm_factor = new ARMFactor(arm_Rs[i],arm_Ps[i],w,w,ESTIMATOR_ARM_FACTOR_TO_BASE_Z,ESTIMATOR_ARM_FACTOR_TO_BASE_Q);
                }
#           else
                const double weight = (pCfg->DEVICE_ID == EE_DEV)?(ESTIMATOR_ARM_FACTOR_TO_EE):(ESTIMATOR_ARM_FACTOR_TO_BASE);
                // TODO: do not use arm factor if base imu is not stable? / weighted factor
                ARMFactor* arm_factor = new ARMFactor(arm_Rs[i],arm_Ps[i],weight);
#           endif //(FEATURE_ENABLE_ARM_ODOMETRY_FACTOR_NON_UNIFORM)
                // ARMFactor* arm_factor = new ARMFactor(arm_Rs[i], arm_Ps[i]);
                problem.AddResidualBlock(arm_factor, NULL, para_Pose[i]);
            }
                
        }
    }
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_FACTOR)

    TOK(t_prepare);

    /*********************************
     * M A R G I N A L I Z A T I O N *
     * *******************************/
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = pCfg->NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = pCfg->SOLVER_TIME * 0.80; // reduce 20% for old
    else
        options.max_solver_time_in_seconds = pCfg->SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    TOK(t_solver);

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;
    
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)
        this->perf.opt_margin_key ++;
#endif // (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)

        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                {
                    drop_set.push_back(i);
                    // PRINT_DEBUG("drop set add i@%d", i); 
                }
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // if(pCfg->USE_IMU) IMU Support
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1], pCfg->G);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});// drop P0 SB0
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // vision marginalization:
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3}); // drop Pi,  Feat_i
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
// #if (FEATURE_ENABLE_STEREO_SUPPORT) // stereo only
//                     if(pCfg->STEREO && it_per_frame.is_stereo)
//                     {
//                         Vector3d pts_j_right = it_per_frame.pointRight;
//                         if(imu_i != imu_j)
//                         {
//                             ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
//                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
//                             ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
//                                                                                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
//                                                                                            vector<int>{0, 4});
//                             marginalization_info->addResidualBlockInfo(residual_block_info);
//                         }
//                         else
//                         {
//                             ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
//                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
//                             ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
//                                                                                            vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
//                                                                                            vector<int>{2});
//                             marginalization_info->addResidualBlockInfo(residual_block_info);
//                         }
//                     }
// #endif
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        TOK(t_pre_margin);
        
        TicToc t_margin;
        marginalization_info->marginalize();
        TOK(t_margin);

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1]; // IMU Support
        }
        for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else // New Margin:
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
#if (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)
            this->perf.opt_margin_not_key ++;
#endif // (FEATURE_ENABLE_PERFORMANCE_EVAL_ESTIMATOR)

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                    {
                        drop_set.push_back(i);
                        // PRINT_DEBUG("drop set add i@%d", i); 
                    }
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            TOK(t_pre_margin);

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            TOK(t_margin);
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1]; //IMU Support
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i]; //IMU Support
                }
            }
            for (int i = 0; i < pCfg->NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    TOK(t_whole_marginalization);
    TOK(t_whole);
}

void Estimator::slideWindow()
{
    if (marginalization_flag == MARGIN_OLD) // slide window as keyframe
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
                arm_Rs[i].swap(arm_Rs[i + 1]);
                arm_Ps[i].swap(arm_Ps[i + 1]);
                arm_valid[i] = arm_valid[i + 1];
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
                
                // if(pCfg->USE_IMU) IMU Support
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
            arm_Rs[WINDOW_SIZE] = arm_Rs[WINDOW_SIZE - 1];
            arm_Ps[WINDOW_SIZE] = arm_Ps[WINDOW_SIZE - 1];
            arm_valid[WINDOW_SIZE] = arm_valid[WINDOW_SIZE - 1];
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)

            // if(pCfg->USE_IMU) IMU Support
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE], pCfg};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else // carry over to the next frame, not a keyframe
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
#if (FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)
            arm_Rs[frame_count - 1] = arm_Rs[frame_count];
            arm_Ps[frame_count - 1] = arm_Ps[frame_count];
            arm_valid[frame_count - 1] = arm_valid[frame_count];
#endif //(FEATURE_ENABLE_ARM_ODOMETRY_SUPPORT)


            // if(pCfg->USE_IMU) IMU Support: carry over the IMU pre-integration to next frame:
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE], pCfg};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
	const double FOCAL_LENGTH = pCfg->FOCAL_LENGTH;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
// #if (FEATURE_ENABLE_STEREO_SUPPORT) // stereo only
//             // need to rewrite projecton factor.........
//             if(pCfg->STEREO && it_per_frame.is_stereo)
//             {
                
//                 Vector3d pts_j_right = it_per_frame.pointRight;
//                 if(imu_i != imu_j)
//                 {            
//                     double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
//                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
//                                                         depth, pts_i, pts_j_right);
//                     err += tmp_error;
//                     errCnt++;
//                     //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
//                 }
//                 else
//                 {
//                     double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
//                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
//                                                         depth, pts_i, pts_j_right);
//                     err += tmp_error;
//                     errCnt++;
//                     //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
//                 }       
//             }
// #endif
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    // update the latest states
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count]; // set keyframe poses
    latest_R = Rs[frame_count]; // set keyframe poses
    latest_Q = Eigen::Quaterniond(latest_R); 
    latest_V = Vs[frame_count]; // set keyframe poses
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
