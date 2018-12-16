

#ifndef _COMMON_DEF_H_
#define _COMMON_DEF_H_

#include <Eigen/Core>

typedef Eigen::Matrix<double, 5, 1> Vector5d;

struct CamIntrinsic_t
{
    Eigen::Matrix3d camK;
    Vector5d discoff;  //k1, k2, k3, p1, p2
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


struct ImgInfo_s
{
    std::string imgPath;
    int64_t timeStamp;
};

struct ImuInfo_s
{
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    int64_t deltaT;
    int64_t timeStamp;
};

// ground truth
struct GTInfo_s
{
    Eigen::Vector3d position;
    Eigen::Vector4d qRotation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d bw;
    Eigen::Vector3d ba;
    int64_t timeStamp;
};




#endif //_COMMON_DEF_H_