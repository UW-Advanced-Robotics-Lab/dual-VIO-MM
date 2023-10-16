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
    pCfgs[BASE_DEV].reset();
    pCfgs[EE_DEV  ].reset();
    pEsts[BASE_DEV].reset();
    pEsts[EE_DEV  ].reset();
}

void EstimatorManager::restartManager()
{
    pEsts[BASE_DEV]->clearState();
    pEsts[EE_DEV  ]->clearState();
    pEsts[BASE_DEV]->setParameter();
    pEsts[EE_DEV  ]->setParameter();
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