#include "Feature.h"
#include "Frame.h"

namespace SLAM
{

Frame::Frame(int mId)
{
    mId_ = mId;
}

Frame::~Frame()
{
    vFeatures_.clear();
}

void Frame::getValidFeaPts(std::vector<Feature *>& vLastFeatures, std::vector<cv::Point2f>& vP2ds)
{
    for (Feature *pFeature : vFeatures_)
    {
        if (pFeature->isBad())
            continue;

        const std::vector<Obs_t> &vObs = pFeature->getObs();
        int nObs = static_cast<int>(vObs.size());
        for (int i = nObs - 1; i >= 0; i--)
        {
            const Obs_t &obs = vObs[i];
            if (obs.pFrame->getId() == mId_)
            {
                cv::Point2f pt = cv::Point2f(obs.p2d(0), obs.p2d(1));
                vP2ds.emplace_back(pt);
                break;
            }
        }

        vLastFeatures.emplace_back(pFeature);
    }

}

void Frame::addFeature(Feature *pFeature)
{
    vFeatures_.emplace_back(pFeature);
}

} // namespace SLAM
