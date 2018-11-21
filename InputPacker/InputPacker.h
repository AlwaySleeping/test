

#ifndef _INPUT_PACKER_H_
#define _INPUT_PACKER_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

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

class InputPacker
{
  public:
    virtual ~InputPacker(){};

    virtual void init(const std::string &caseRootFolder) = 0;

    virtual bool getSensorData(int index, cv::Mat &image, std::vector<ImuInfo_s> &vImuDate) = 0;

    virtual bool packSensorData() = 0;

    virtual bool undistort(cv::Mat srcImg, cv::Mat& distImg) = 0;

    int getImgNum(){ return static_cast<int>(vImgInfos_.size());}

  public:
    std::vector<ImgInfo_s> vImgInfos_;
    std::vector<ImuInfo_s> vImuInfos_;
    std::vector<std::pair<ImgInfo_s, std::vector<ImuInfo_s> > > vImuImgPairs_;
};

#endif //_INPUT_PACKER_H_/