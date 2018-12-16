#include "OpticalFlowTracker.h"

#include <tuple>

namespace SLAM
{

OpticalFlowTracker::OpticalFlowTracker()
{
    // initialize class members
    nTrackerID_ = 0;

    if_ = Eigen::Vector2d::Zero();

    nImageCols_ = 0;
    nImageRows_ = 0;
}

OpticalFlowTracker::~OpticalFlowTracker()
{
    reset();
}

bool OpticalFlowTracker::setPara(const TrackerPara_s &stPara)
{
    if_ = 1.0 / stPara.f.array();
    stTrackPara_.nFeatureNum = stPara.nFeatureNum;
    stTrackPara_.nMaskRadius = stPara.nMaskRadius;
    stTrackPara_.nMaxTrackLenth = stPara.nMaxTrackLenth;
    stTrackPara_.nBorder = stPara.nBorder;
    stTrackPara_.fDistance = stPara.fDistance;
    stTrackPara_.fQuality = stPara.fQuality;
    stTrackPara_.f = stPara.f;
    stTrackPara_.c = stPara.c;
    stTrackPara_.k = stPara.k;
    stTrackPara_.p = stPara.p;
    stTrackPara_.bDoCLAHE = stPara.bDoCLAHE;
    stTrackPara_.bInputUndistortedImg = stPara.bInputUndistortedImg;

    std::cout << stTrackPara_.f << std::endl;
    std::cout << stTrackPara_.c << std::endl;
    std::cout << stTrackPara_.k << std::endl;
    std::cout << stTrackPara_.p << std::endl;

    if (stTrackPara_.bDoCLAHE)
    {
        pClaheObj_ = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    return true;
}

void OpticalFlowTracker::reset()
{
    prvGrayImage_.release();
    curGrayImage_.release();
    vecCurPoints_.clear();
    vecPrvPoints_.clear();
    vecTrackerID_.clear();
    vecPointTrackedCnt_.clear();
    mask_.release();
    nTrackerID_ = 0;
}

void OpticalFlowTracker::preProcess(cv::Mat &grayImage)
{
    // histogram equalization
    if (stTrackPara_.bDoCLAHE)
    {
        pClaheObj_->apply(grayImage, curGrayImage_);
    }
    else
    {
        curGrayImage_ = grayImage.clone();
    }
}

void OpticalFlowTracker::trackCurrFrame(cv::Mat &inputImage)
{
    cv::Mat grayImage;
    if (inputImage.channels() > 1)
        cv::cvtColor(inputImage, grayImage, CV_BGR2GRAY);
    else
        inputImage.copyTo(grayImage);

    if (prvGrayImage_.empty())
    {
        nImageCols_ = grayImage.cols;
        nImageRows_ = grayImage.rows;
        mask_ = cv::Mat(nImageRows_, nImageCols_, CV_8UC1, cv::Scalar(255));
    }

    // clear points
    vecCurPoints_.clear();
    vecUndistortedPoints_.clear();

    // update image
    preProcess(grayImage);

    // tracking previous points
    if (0 < vecPrvPoints_.size())
    {
        std::vector<uchar> vecStatus;
        std::vector<float> vecError;

        cv::calcOpticalFlowPyrLK(prvGrayImage_, curGrayImage_, vecPrvPoints_, vecCurPoints_,
                                 vecStatus, vecError, cv::Size(21, 21), 3);

        int nCurPointsNum = static_cast<int>(vecCurPoints_.size());
        for (int i = 0; i < nCurPointsNum; i++)
        {
            if (!isInBorder(vecCurPoints_[i]) || vecPointTrackedCnt_[i] > stTrackPara_.nMaxTrackLenth)
            {
                vecStatus[i] = 0;
            }
        }

        std::cout << "before reduce1: " << vecCurPoints_.size() << std::endl;
        reduceVector(vecStatus);
        std::cout << "after reduce1: " << vecCurPoints_.size() << std::endl;

        std::cout << "before rejectOutlierWithFMat: " << vecCurPoints_.size() << std::endl;
        // reject outlier with F matrix
        rejectOutlierWithFMat();
        std::cout << "after rejectOutlierWithFMat: " << vecCurPoints_.size() << std::endl;

        // increase tracking count
        for (auto &nCnt : vecPointTrackedCnt_)
        {
            nCnt++;
        }

        // update mask
        setMask();
    }

    const int nNeedNewNum = stTrackPara_.nFeatureNum - static_cast<int>(vecCurPoints_.size());
    std::cout << "nNeedNewNum: " << nNeedNewNum << std::endl;
    ;
    if (nNeedNewNum > 0)
    {
        std::vector<cv::Point2f> vecNewPoints;
        if (vecPrvPoints_.empty())
            cv::goodFeaturesToTrack(curGrayImage_, vecNewPoints, nNeedNewNum, stTrackPara_.fQuality, stTrackPara_.nMaskRadius, mask_);
        else
            extractNewPoints(curGrayImage_, vecCurPoints_, vecNewPoints);

        std::cout << "extract new points num: " << vecNewPoints.size() << std::endl;
        addPoints(vecNewPoints);
    }

    curGrayImage_.copyTo(prvGrayImage_);
    vecPrvPoints_ = vecCurPoints_;
    vecPrvUndistortedPoints_ = vecUndistortedPoints_;
}

void OpticalFlowTracker::getUndistortedPoints(std::vector<cv::Point2f> &vecPoints)
{
    vecPoints.clear();
    // update undistorted points
    int nPrvPointsNum = static_cast<int>(vecUndistortedPoints_.size());
    vecPoints.resize(nPrvPointsNum);
    for (int i = 0; i < nPrvPointsNum; ++i)
    {
        Eigen::Vector2d undistortedPt(vecUndistortedPoints_[i].x, vecUndistortedPoints_[i].y);
        undistortedPt = (undistortedPt - stTrackPara_.c).array() * if_.array();
        vecPoints[i] = cv::Point2f(undistortedPt[0], undistortedPt[1]);
    }
}

void OpticalFlowTracker::getOrgImagePoints(std::vector<cv::Point2f> &vecPoints)
{
    vecPoints = vecCurPoints_;
}

std::vector<cv::Point2f> &OpticalFlowTracker::getTrackedPoints(void)
{
    return vecPrvPoints_;
}

std::vector<int> &OpticalFlowTracker::getIDs(void)
{
    return vecTrackerID_;
}

std::vector<int> &OpticalFlowTracker::getTrackCnt(void)
{
    return vecPointTrackedCnt_;
}

void OpticalFlowTracker::setMask()
{
    const uchar ucLabel = 255;
    memset(mask_.data, ucLabel, nImageRows_ * nImageCols_);

    unsigned int nCurPointNum = static_cast<unsigned int>(vecCurPoints_.size());
    std::vector<std::tuple<int, cv::Point2f, int, cv::Point2f>> vecPtsPair(nCurPointNum);
    for (unsigned int i = 0; i < nCurPointNum; ++i)
    {
        vecPtsPair[i] = std::make_tuple(vecPointTrackedCnt_[i], vecCurPoints_[i], vecTrackerID_[i], vecUndistortedPoints_[i]);
    }

    // sort points
    std::sort(vecPtsPair.begin(), vecPtsPair.end(),
              [](const std::tuple<int, cv::Point2f, int, cv::Point2f> &a,
                 const std::tuple<int, cv::Point2f, int, cv::Point2f> &b) {
                  return std::get<0>(a) > std::get<0>(b);
              });

    vecCurPoints_.clear();
    vecTrackerID_.clear();
    vecPointTrackedCnt_.clear();
    vecUndistortedPoints_.clear();

    for (auto &it : vecPtsPair)
    {
        cv::Point2f &pt = std::get<1>(it);
        if (ucLabel == mask_.at<uchar>(pt))
        {
            vecCurPoints_.push_back(pt);
            vecTrackerID_.push_back(std::get<2>(it));
            vecPointTrackedCnt_.push_back(std::get<0>(it));
            vecUndistortedPoints_.push_back(std::get<3>(it));

            cv::circle(mask_, pt, stTrackPara_.nMaskRadius, 0, -1);
        }
    }
}

void OpticalFlowTracker::addPoints(const std::vector<cv::Point2f> &vecPts)
{
    unsigned int nCurPointsNum = static_cast<unsigned int>(vecCurPoints_.size());
    unsigned int nNewPointsNum = static_cast<unsigned int>(vecPts.size());
    vecCurPoints_.resize(nCurPointsNum + nNewPointsNum);
    vecTrackerID_.resize(nCurPointsNum + nNewPointsNum);
    vecPointTrackedCnt_.resize(nCurPointsNum + nNewPointsNum);
    vecUndistortedPoints_.resize(nCurPointsNum + nNewPointsNum);

    for (auto &pt : vecPts)
    {
        vecCurPoints_[nCurPointsNum] = pt;
        vecTrackerID_[nCurPointsNum] = nTrackerID_;
        vecPointTrackedCnt_[nCurPointsNum] = 1;
        if (stTrackPara_.bInputUndistortedImg == false)
        {
            Eigen::Vector2d tmp = undistortPoint(Eigen::Vector2d(pt.x, pt.y));
            vecUndistortedPoints_[nCurPointsNum] = cv::Point2f(tmp(0), tmp(1));
        }
        else
            vecUndistortedPoints_[nCurPointsNum] = pt;

        nTrackerID_++;
        nCurPointsNum++;
    }

    std::cout << "first undistortedPoint: " << vecUndistortedPoints_[0] << std::endl;
    ;
}

void OpticalFlowTracker::rejectOutlierWithFMat()
{
    const int nCur = static_cast<int>(vecCurPoints_.size());
    if (8 > nCur)
    {
        return;
    }

    // for (int i = 0; i < nCur; ++i)
    // {
    //     const cv::Point2f &pt2 = vecPrvPoints_[i];
    //     const Eigen::Vector2d tmp2 = undistortPoint(Eigen::Vector2d(pt2.x, pt2.y));
    //     std::cout << "tmp2: " << tmp2.transpose() << std::endl;
    //     std::cout << "vecPrvUndistortedPoints_[i]: " << vecPrvUndistortedPoints_[i] << std::endl;
    // }

    vecPrvUndistortedPoints_.clear();
    vecPrvUndistortedPoints_.resize(nCur);
    std::vector<uchar> vecStatus;
    vecUndistortedPoints_.resize(nCur);
    if (stTrackPara_.bInputUndistortedImg == false)
    {
        for (int i = 0; i < nCur; ++i)
        {
            const cv::Point2f &pt2 = vecPrvPoints_[i];
            const Eigen::Vector2d tmp2 = undistortPoint(Eigen::Vector2d(pt2.x, pt2.y));
            vecPrvUndistortedPoints_[i] = cv::Point2f(tmp2.x(), tmp2.y());
        }

        for (int i = 0; i < nCur; ++i)
        {
            const cv::Point2f &pt2 = vecCurPoints_[i];
            const Eigen::Vector2d tmp2 = undistortPoint(Eigen::Vector2d(pt2.x, pt2.y));
            vecUndistortedPoints_[i] = cv::Point2f(tmp2.x(), tmp2.y());
        }
        std::cout << "=== first vecPrvUndistortedPoints_: " << vecPrvUndistortedPoints_[0] << std::endl;
        cv::findFundamentalMat(vecPrvUndistortedPoints_, vecUndistortedPoints_, cv::FM_RANSAC, stTrackPara_.fDistance, 0.99, vecStatus);
    }
    else
    {
        cv::findFundamentalMat(vecPrvPoints_, vecCurPoints_, cv::FM_RANSAC, stTrackPara_.fDistance, 0.99, vecStatus);
    }

    reduceVector(vecStatus);
}

Eigen::Vector2d OpticalFlowTracker::undistortPoint(const Eigen::Vector2d &pt)
{
    const int nIters = 5;
    Eigen::Vector2d p, p0;
    p0 = p = (pt - stTrackPara_.c).array() * if_.array();
    for (int k = 0; k < nIters; ++k)
    {
        const double xx = p(0) * p(0);
        const double yy = p(1) * p(1);
        const double xy = p(0) * p(1);
        const double r2 = xx + yy;

        const double icdist = 1.0f / (1.0f + (stTrackPara_.k(1) * r2 + stTrackPara_.k(0)) * r2);
        const double deltaX = 2.0f * stTrackPara_.p(0) * xy + stTrackPara_.p(1) * (r2 + 2.0f * xx);
        const double deltaY = stTrackPara_.p(0) * (r2 + 2.0f * yy) + 2.0f * stTrackPara_.p(1) * xy;
        p = (p0 - Eigen::Vector2d(deltaX, deltaY)) * icdist;
    }

    return p.array() * stTrackPara_.f.array() + stTrackPara_.c.array();
}

inline bool OpticalFlowTracker::isInBorder(const cv::Point2f &pt)
{
    const int nX = cvRound(pt.x);
    const int nY = cvRound(pt.y);
    return (stTrackPara_.nBorder <= nX) && (nX < nImageCols_ - stTrackPara_.nBorder) &&
           (stTrackPara_.nBorder <= nY) && (nY < nImageRows_ - stTrackPara_.nBorder);
}

void OpticalFlowTracker::reduceVector(const std::vector<uchar> &vecStatus)
{
    reduceVector(vecPrvPoints_, vecStatus);
    reduceVector(vecCurPoints_, vecStatus);
    reduceVector(vecTrackerID_, vecStatus);
    reduceVector(vecPointTrackedCnt_, vecStatus);
    reduceVector(vecPrvUndistortedPoints_, vecStatus);
    reduceVector(vecUndistortedPoints_, vecStatus);
}

template <class Type>
void OpticalFlowTracker::reduceVector(std::vector<Type> &vecData, const std::vector<uchar> &vecStatus) const
{
    int nValid = 0;
    for (int i = 0; i < static_cast<int>(vecData.size()); ++i)
    {
        if (0 < vecStatus[i])
        {
            vecData[nValid++] = vecData[i];
        }
    }
    vecData.resize(nValid);
}

void OpticalFlowTracker::extractNewPoints(cv::Mat &grayImg,
                                          std::vector<cv::Point2f> &vOldPoints,
                                          std::vector<cv::Point2f> &vNewPoints)
{
    vNewPoints.clear();
    int imgW = grayImg.cols;
    int imgH = grayImg.rows;
    int nRowGrid = 5;
    int nColGrid = 8;
    int nPreGrid = stTrackPara_.nFeatureNum / (nRowGrid * nColGrid);
    int widthGrid = floor(1.0 * imgW / nColGrid);
    int hightGrid = floor(1.0 * imgH / nRowGrid);

    std::cout << "old: " << vOldPoints.size() << std::endl;
    std::vector<std::vector<cv::Point2f>> vvOldPoints(nRowGrid * nColGrid);
    int nOldP2d = static_cast<int>(vOldPoints.size());
    for (int i = 0; i < nOldP2d; i++)
    {
        cv::Point2f pt = vOldPoints[i];
        int idx = pt.x / widthGrid;
        int idy = pt.y / hightGrid;
        int index = idx + idy * nColGrid;
        vvOldPoints[index].push_back(pt);
    }

    std::vector<cv::KeyPoint> vFastPoints;
    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(10);
    detector->detect(grayImg, vFastPoints, mask_);
    int nFast = static_cast<int>(vFastPoints.size());
    std::vector<std::vector<cv::KeyPoint>> vvNewPoints(nRowGrid * nColGrid);
    for (int i = 0; i < nFast; i++)
    {
        cv::KeyPoint &kp = vFastPoints[i];
        int idx = kp.pt.x / widthGrid;
        int idy = kp.pt.y / hightGrid;
        int index = idx + idy * nColGrid;
        vvNewPoints[index].push_back(kp);
    }

    std::vector<cv::KeyPoint> vNewKeyPoints;
    for (int i = 0; i < nColGrid; i++)
    {
        for (int j = 0; j < nRowGrid; j++)
        {
            int index = i + j * nColGrid;
            std::vector<cv::KeyPoint> &vTemp = vvNewPoints[index];
            int nMax = static_cast<int>(vTemp.size());
            int nNeed = std::min(nPreGrid - static_cast<int>(vvOldPoints[index].size()), nMax);
            // std::cout << "index: " << index << ", nPreGrid: " << nPreGrid << ", old: " << vvOldPoints[index].size() << ", nNeed: " << nNeed << std::endl;
            if (nNeed > 0)
            {
                std::sort(vTemp.begin(), vTemp.end(),
                          [&](const cv::KeyPoint t1, const cv::KeyPoint t2) -> bool { return t1.response > t2.response; });
                vNewKeyPoints.insert(vNewKeyPoints.end(), vTemp.begin(), vTemp.begin() + nNeed);
            }
        }
    }

    cv::KeyPoint::convert(vNewKeyPoints, vNewPoints);

    if (1)
    {
        cv::Mat imgColor;
        cv::cvtColor(grayImg, imgColor, cv::COLOR_GRAY2BGR);
        for (int i = 0; i < nColGrid; i++)
        {
            cv::Point2d pUp, pDown;
            pUp.x = i * widthGrid;
            pUp.y = 0;
            pDown.x = pUp.x;
            pDown.y = imgH;
            cv::line(imgColor, pUp, pDown, cv::Scalar(0, 0, 0), 2);
        }

        for (int i = 0; i < nRowGrid; i++)
        {
            cv::Point2d pl, pr;
            pl.x = 0;
            pl.y = i * hightGrid;
            pr.x = imgW;
            pr.y = pl.y;
            cv::line(imgColor, pl, pr, cv::Scalar(0, 0, 0), 2);
        }

        for (int i = 0; i < nOldP2d; i++)
        {
            cv::circle(imgColor, vOldPoints[i], 2, cv::Scalar(0, 0, 255), -1);
        }

        for (int i = 0; i < vNewPoints.size(); i++)
        {
            cv::circle(imgColor, vNewPoints[i], 2, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("imgColor", imgColor);
        cv::imshow("mask ", mask_);
        cv::waitKey(1);
    }
}

} // namespace SLAM