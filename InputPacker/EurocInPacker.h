

#ifndef _EUROC_IN_PACKER_H_
#define _EUROC_IN_PACKER_H_

#include "InputPacker.h"

class EurocInPacker : public InputPacker
{
  public:
    virtual ~EurocInPacker();

    virtual void init(const std::string &caseRootFolder);

    virtual bool getSensorData(int index, cv::Mat &image, std::vector<ImuInfo_s> &vImuDate);

    virtual bool packSensorData();

    bool readImage(const std::string &caseRootFolder);
    bool readImu(const std::string &caseRootFolder);
    bool readGroundTruth(const std::string &caseRootFolder);

  private:
    std::vector<GTInfo_s> vGtInfos_;
};

#endif //_EUROC_IN_PACKER_H_/