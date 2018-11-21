#ifndef _PARSER_H_
#define _PARSER_H_

#include <string>
#include <Eigen/Core>

#include "CommonDef.h"

class SensorConfig
{
  public:
    void init(const std::string &sensorConfigPath);

    // load camera config;
    bool loadSensorConfig(const std::string &sensorConfigPath);

    const CamIntrinsic_t getCamIntrinsic()
    {
        CamIntrinsic_t cam;
        cam.camK = k_;
        cam.discoff = discoff_;

        return cam;
    }

    const ImuIntrinsic_t getImuIntrinsic()
    {
        ImuIntrinsic_t imu;
        imu.na  = na_;
        imu.ng  = ng_;
        imu.nba = nba_;
        imu.nbg = nbg_;

        return imu;
    }

    const Eigen::Matrix4d getTbc()
    {
        return T_imu_cam_;
    }

  public:
    // camera intistics:
    Eigen::Matrix3d k_;
    Eigen::Vector4d discoff_;
    double fx_, fy_, cx_, cy_;
    double k1_, k2_;
    double p1_, p2_;
    double camFps_;

    Eigen::Matrix4d T_imu_cam_; //from cam to imu

    //Imu param:
    double nba_;
    double nbg_;

    //white noise
    double na_;
    double ng_;
};

class AlgoConfig
{
    void init(const std::string &configPath);
};

#endif //_PARSER_H_/
