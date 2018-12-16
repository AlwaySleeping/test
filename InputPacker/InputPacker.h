

#ifndef _INPUT_PACKER_H_
#define _INPUT_PACKER_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "CommonDef.h"

class InputPacker
{
  public:
    virtual ~InputPacker(){};

    virtual void init(const std::string &caseRootFolder) = 0;

    virtual bool getSensorData(int index, cv::Mat &image, std::vector<ImuInfo_s> &vImuDate, bool bUndistortImage) = 0;

    virtual bool packSensorData() = 0;

    void setCamIntrinsic(const Eigen::Matrix3d &camK, const Vector5d &camDiscoff)
    {
        cvCameK_ = (cv::Mat_<float>(3, 3) << camK(0, 0), 0, camK(0, 2),
                    0, camK(1, 1), camK(1, 2),
                    0, 0, 1);

        cvDiscoff_ = (cv::Mat_<float>(4, 1) << camDiscoff(0), camDiscoff(1), camDiscoff(3), camDiscoff(4));
    }

    bool undistort(const cv::Mat &srcImg, cv::Mat &distImg)
    {
        if (mapx_.empty() || mapy_.empty())
        { // use map buffer to speed up
            cv::initUndistortRectifyMap(cvCameK_, cvDiscoff_, cv::Mat(),
                                        cvCameK_, srcImg.size(), CV_32FC1, mapx_, mapy_);
        }

        if (distImg.empty())
        {
            distImg.create(srcImg.size(), srcImg.type());
        }

        cv::remap(srcImg, distImg, mapx_, mapy_, cv::INTER_LINEAR);

        return true;
    }

    int getImgNum() { return static_cast<int>(vImgInfos_.size()); }

  public:
    std::vector<ImgInfo_s> vImgInfos_;
    std::vector<ImuInfo_s> vImuInfos_;
    std::vector<std::pair<ImgInfo_s, std::vector<ImuInfo_s>>> vImuImgPairs_;

    cv::Mat cvCameK_;
    cv::Mat cvDiscoff_;

    cv::Mat mapx_, mapy_;
};

#endif //_INPUT_PACKER_H_/