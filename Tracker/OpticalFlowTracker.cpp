

#include "OpticalFlowTracker.h"
#include "Frame.h"
#include "Feature.h"
#include "CommonDef.h"

namespace SLAM
{

OpticalFlowTracker::~OpticalFlowTracker()
{
}

void OpticalFlowTracker::setTrackPara(const CamIntrinsic_t &mCamInsic)
{
    pLastFrame_ = NULL;
    nTrackerID_ = 0;

    mCamK_ = mCamInsic.camK;
    mDiscoff_ = mCamInsic.discoff;

    fx_ = mCamK_(0, 0);
    fy_ = mCamK_(1, 1);
    cx_ = mCamK_(0, 2);
    cy_ = mCamK_(1, 2);

    sTrakPara_.nQualityLevel = 0.000001;
    sTrakPara_.nTotalPoints = 500;
    sTrakPara_.nMask = 10;
    sTrakPara_.nMaxLength = 100;
    sTrakPara_.nRowGrid = 5;
    sTrakPara_.nColGrid = 8;
}

void OpticalFlowTracker::trackCurrFrame(const cv::Mat &image, Frame *pCurFrame)
{
    int nTracked = 0;
    if (image.channels() != 1)
        cv::cvtColor(image, mCurrImage_, CV_BGR2GRAY);
    else
        image.copyTo(mCurrImage_);

    if (pLastFrame_ == NULL)
    {
        mImgH_ = mCurrImage_.rows;
        mImgW_ = mCurrImage_.cols;
        mMask_ = cv::Mat(mImgH_, mImgW_, CV_8UC1, cv::Scalar(255));
    }
    else
    {
        std::vector<cv::Point2f> vLastP2ds;
        std::vector<cv::Point2f> vCurrP2ds;
        std::vector<Feature *> vLastFeatures;

        //*** step1.get 2d points from last frame;
        pLastFrame_->getValidFeaPts(vLastFeatures, vLastP2ds);
        std::vector<bool> vInlierFlags(vLastP2ds.size(), false);

        //*** step2.track with optical flow;
        opticalFlowTrack(vLastP2ds, vCurrP2ds, vInlierFlags);

        //*** step3.remove outliers
        removeOutliers(vInlierFlags, vLastFeatures, vCurrP2ds);

        //*** step4 add frame as obs to features, and add feature to frame;
        addObsTracks(vLastFeatures, vCurrP2ds, pCurFrame);

        //*** step5.update the mask
        updateMask(vCurrP2ds);

        nTracked = static_cast<int>(vCurrP2ds.size());
    }

    //*** step6.extract new 2d points to next track;
    std::vector<cv::Point2f> vNewPts;
    extractPoints(nTracked, mCurrImage_, vNewPts);

    //*** step7. create new features and add obs
    addObsTracks(std::vector<Feature *>(), vNewPts, pCurFrame);

    bool debug = true;
    if (debug)
    {
        std::vector<cv::Point2f> vP2ds;
        std::vector<Feature *> vFeatures;
        pCurFrame->getValidFeaPts(vFeatures, vP2ds);
        //draw all:
        cv::Mat I;
        mCurrImage_.copyTo(I);
        cv::cvtColor(I, I, CV_GRAY2BGR);
        for (size_t i = 0; i < vP2ds.size(); i++)
        {
            cv::circle(I, vP2ds[i], 0, cv::Scalar(0, 255, 0), 3);
        }
        for (size_t i = 0; i < vNewPts.size(); i++)
        {
            cv::circle(I, vNewPts[i], 0, cv::Scalar(0, 0, 255), 3);
        }
        cv::imshow("I", I);
        cv::waitKey(1);
    }

    mLastImage_ = mCurrImage_.clone();
    pLastFrame_ = pCurFrame;
}

void OpticalFlowTracker::removeOutliers(std::vector<bool> &vInlierFlags,
                                        std::vector<Feature *> &vLastFeatures,
                                        std::vector<cv::Point2f> &vCurrP2ds)
{
    std::vector<cv::Point2f> vTmpP2ds;
    std::vector<Feature *> vTmpLastFeatures;
    int nFeature = static_cast<int>(vLastFeatures.size());
    for (int i = 0; i < nFeature; i++)
    {
        if (vInlierFlags[i] == true)
        {
            vTmpP2ds.emplace_back(vCurrP2ds[i]);
            vTmpLastFeatures.emplace_back(vLastFeatures[i]);
        }
    }
    vLastFeatures = vTmpLastFeatures;
    vCurrP2ds = vTmpP2ds;
}

void OpticalFlowTracker::opticalFlowTrack(std::vector<cv::Point2f> &vLastP2ds,
                                          std::vector<cv::Point2f> &vCurrP2ds,
                                          std::vector<bool> &vInlierFlags)
{
    std::vector<uchar> vecStatus;
    std::vector<float> vecError;

    // tracking points with optical flow
    cv::calcOpticalFlowPyrLK(mLastImage_, mCurrImage_, vLastP2ds, vCurrP2ds, vecStatus, vecError, cv::Size(15, 15), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    checkInliers(vLastP2ds, vCurrP2ds, vInlierFlags, vecStatus);
}

void OpticalFlowTracker::updateMask(const std::vector<cv::Point2f> &vCurrP2ds)
{
    const uchar ucLabel = 255;
    memset(mMask_.data, ucLabel, mLastImage_.cols * mLastImage_.rows);
    for (const cv::Point2f &it : vCurrP2ds)
    {
        cv::circle(mMask_, cv::Point(it.x, it.y), sTrakPara_.nMask, 0, -1);
    }
}

void OpticalFlowTracker::extractPoints(int nTracked, const cv::Mat &curImg, std::vector<cv::Point2f> &vPoints)
{
    //****Step2: extract 2d points, and add to current frame;
    int needNum = std::max(sTrakPara_.nTotalPoints - nTracked, 0);
    if (needNum <= 0)
    {
        std::cout << "tracked enough point num: " << sTrakPara_.nTotalPoints << ", should not extract new points!" << std::endl;
        return;
    }
    cv::goodFeaturesToTrack(curImg, vPoints, needNum, sTrakPara_.nQualityLevel, sTrakPara_.nMask, mMask_);

    std::cout << "need: " << needNum << ", extract new: " << vPoints.size() << std::endl;
}

void OpticalFlowTracker::addObsTracks(const std::vector<Feature *> &vLastFeatures,
                                      const std::vector<cv::Point2f> &vCurrP2ds,
                                      Frame *pCurFrame)
{
    int nP2ds = static_cast<int>(vCurrP2ds.size());
    int nFeatures = static_cast<int>(vLastFeatures.size());
    Eigen::Vector2d p2d, p2d_norm;
    cv::Point2f cvP2d;
    for (int i = 0; i < nP2ds; i++)
    {
        cvP2d = vCurrP2ds[i];
        p2d << cvP2d.x, cvP2d.y;
        p2d_norm(0) = (p2d(0) - cx_) / fx_;
        p2d_norm(1) = (p2d(1) - cy_) / fy_;
        //undistort p2d_norm();

        Feature *pFeature;
        if (nFeatures != 0)
            pFeature = vLastFeatures[i];
        else
        {
            pFeature = new Feature(nTrackerID_);
            nTrackerID_++;
        }

        pFeature->addObsFrame(pCurFrame, p2d, p2d_norm);
        pCurFrame->addFeature(pFeature);
    }
}

void OpticalFlowTracker::checkInliers(std::vector<cv::Point2f> &vPrePts,
                                      std::vector<cv::Point2f> &vCurPts,
                                      std::vector<bool> &vInliers,
                                      std::vector<uchar> &status)
{
    int border_ = 5;
    std::vector<cv::Point2f> vPreTemp, vCurTemp;
    std::vector<int> vPointIndex;
    for (std::size_t i = 0; i < status.size(); i++)
    {
        if (vCurPts[i].x > mImgW_ - border_ || vCurPts[i].x < border_ ||
            vCurPts[i].y > mImgH_ - border_ || vCurPts[i].y < border_ || status[i] == 0)
            vInliers[i] = false;
        else
        {
            vInliers[i] = true;
            vPreTemp.push_back(vPrePts[i]);
            vCurTemp.push_back(vCurPts[i]);
            vPointIndex.push_back(i);
        }
    }

    std::vector<uchar> vFstatus;
    cv::Mat F = cv::findFundamentalMat(vPreTemp, vCurTemp, cv::FM_RANSAC, 2, 0.99, vFstatus);

    if (F.empty())
        return;

    int nFoutlier = 0;
    for (std::size_t i = 0; i < vFstatus.size(); i++)
    {
        if (vFstatus[i] == 0)
        {
            vInliers[vPointIndex[i]] = false;
            nFoutlier++;
        }
    }
}

} // namespace SLAM