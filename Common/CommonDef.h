

#ifndef _COMMON_DEF_H_
#define _COMMON_DEF_H_

#include <Eigen/Core>

struct CamIntrinsic_t
{
    Eigen::Matrix3d camK;
    Eigen::Vector4d discoff;
};

struct ImuIntrinsic_t
{
    //Imu biase param:
    double nba;
    double nbg;

    //white noise
    double na;
    double ng;
};





#endif //_COMMON_DEF_H_