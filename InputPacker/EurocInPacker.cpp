

#include "EurocInPacker.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <assert.h>

EurocInPacker::~EurocInPacker()
{
    vImgInfos_.clear();
    vImuInfos_.clear();
    vGtInfos_.clear();
    vImuImgPairs_.clear();
}

bool EurocInPacker::readImage(const std::string &caseRootFolder)
{
    std::string imgFolder = caseRootFolder + "/cam0/data";
    std::string imgTimeStampPath = caseRootFolder + "/cam0/data.csv";
    std::cout << "load image path: " << imgFolder << std::endl;
    std::ifstream fImg(imgTimeStampPath);
    int linNum = 0;
    if (fImg.is_open())
    {
        while (!fImg.eof())
        {
            linNum++;
            std::string lineData;
            std::getline(fImg, lineData);
            std::stringstream stringin(lineData);
            if (linNum == 1 || 0 == lineData.length())
                continue;

            ImgInfo_s imgInfo;
            std::string s, name;
            stringin >> imgInfo.timeStamp;
            stringin >> name;
            name.erase(name.begin());
            imgInfo.imgPath = imgFolder + "/" + name;
            vImgInfos_.emplace_back(imgInfo);
        }
    }
    else
    {
        std::cout << "Load image timeStamp file faild!" << imgTimeStampPath << std::endl;
        return false;
    }

    fImg.close();
    std::cout << "read image size: " << vImgInfos_.size() << std::endl;

    return true;
}

bool EurocInPacker::readImu(const std::string &caseRootFolder)
{
    std::string imuPath = caseRootFolder + "/imu0/data.csv";
    std::cout << "load imu path: " << imuPath << std::endl;
    std::ifstream fImu(imuPath);
    std::string lineData, item;
    int linNum = 0;
    if (fImu.is_open())
    {
        while (!fImu.eof())
        {
            linNum++;
            std::getline(fImu, lineData);
            std::stringstream stringin(lineData);
            if (linNum == 1 || 0 == lineData.length())
                continue;

            int i = 0;
            double data[6];
            ImuInfo_s imuInfo;
            while (getline(stringin, item, ','))
            {
                std::stringstream ss;
                ss << item;
                if (i == 0)
                    ss >> imuInfo.timeStamp;
                else
                {
                    ss >> data[i - 1];
                }
                i++;
            }

            imuInfo.gyr[0] = data[0];
            imuInfo.gyr[1] = data[1];
            imuInfo.gyr[2] = data[2];
            imuInfo.acc[0] = data[3];
            imuInfo.acc[1] = data[4];
            imuInfo.acc[2] = data[5];
            imuInfo.deltaT = 0;
            vImuInfos_.emplace_back(imuInfo);
        }
    }
    else
    {
        std::cout << "Load imu file faild!" << imuPath << std::endl;
        return false;
    }

    fImu.close();
    std::cout << "read imu size: " << vImuInfos_.size() << std::endl;
    return true;
}

bool EurocInPacker::readGroundTruth(const std::string &caseRootFolder)
{
    std::string GTPath = caseRootFolder + "/state_groundtruth_estimate0/data.csv";
    std::cout << "load ground truth file path: " << GTPath << std::endl;
    std::ifstream fGT(GTPath);
    std::string lineData, item;
    int linNum = 0;
    if (fGT.is_open())
    {
        while (!fGT.eof())
        {
            linNum++;
            std::getline(fGT, lineData);
            std::stringstream stringin(lineData);
            if (linNum == 1 || 0 == lineData.length())
                continue;

            int i = 0;
            double data[16];
            GTInfo_s gtInfo;
            while (getline(stringin, item, ','))
            {
                std::stringstream ss;
                ss << item;
                if (i == 0)
                    ss >> gtInfo.timeStamp;
                else
                {
                    ss >> data[i - 1];
                }
                i++;
            }

            gtInfo.position << data[0], data[1], data[2];
            gtInfo.qRotation << data[3], data[4], data[5], data[6];
            gtInfo.velocity << data[7], data[8], data[9];
            gtInfo.bw << data[10], data[11], data[12];
            ;
            gtInfo.ba << data[13], data[14], data[15];
            vGtInfos_.emplace_back(gtInfo);
        }
    }
    else
    {
        std::cout << "Load ground truth file faild!" << GTPath << std::endl;
        return false;
    }

    fGT.close();
    std::cout << "read ground true size: " << vGtInfos_.size() << std::endl;
    return true;
}

void EurocInPacker::init(const std::string &caseRootFolder)
{
    readImage(caseRootFolder);
    readImu(caseRootFolder);
    readGroundTruth(caseRootFolder);
    packSensorData();
}

bool EurocInPacker::packSensorData()
{
    int nImu = static_cast<int>(vImuInfos_.size());
    int nImg = static_cast<int>(vImgInfos_.size());
    assert(nImu > nImg);

    int64_t lastTimeStamp;
    int imgIndex = 0;
    int imuIndex = 0;

    //find the first imu whoes timestamp is bigger than first image's timestamp;
    for (int imuIndex = 0; imuIndex < nImu; imuIndex++)
    {
        if (vImuInfos_[imuIndex].timeStamp >= vImgInfos_.front().timeStamp)
            break;
    }

    assert(imuIndex != nImu);

    for (ImgInfo_s &img : vImgInfos_)
    {
        std::vector<ImuInfo_s> vImuTmp;
        vImuTmp.clear();
        int64_t timeStamp = img.timeStamp;
        int64_t intervalTime = 0;
        //the first image's imudata should be empty();
        if (imgIndex != 0)
        {
            while (imuIndex + 1 < nImu)
            {
                if (vImuInfos_[imuIndex + 1].timeStamp <= timeStamp)
                {
                    ImuInfo_s &imuData = vImuInfos_[imuIndex];
                    if (vImuTmp.empty())
                        imuData.deltaT = imuData.timeStamp - lastTimeStamp;
                    else
                        imuData.deltaT = imuData.timeStamp - vImuTmp.back().timeStamp;

                    intervalTime += imuData.deltaT;
                    vImuTmp.emplace_back(imuData);
                    imuIndex++;
                }
                else if (!vImuTmp.empty()) //add a new imudata, the timestamp equal to current image timestamp
                {
                    ImuInfo_s lastImuData = vImuTmp.back();
                    lastImuData.deltaT = timeStamp - lastImuData.timeStamp;
                    lastImuData.timeStamp = timeStamp;

                    intervalTime += lastImuData.deltaT;
                    vImuTmp.emplace_back(lastImuData);
                    imuIndex++;
                    break;
                }
            }

            assert(intervalTime == (timeStamp - lastTimeStamp));
        }

        vImuImgPairs_.emplace_back(img, vImuTmp);
        lastTimeStamp = timeStamp;
        imgIndex++;
    }
    std::cout << "read imu image pair size: " << vImuImgPairs_.size() << std::endl;
    return true;
}

bool EurocInPacker::getSensorData(int index, cv::Mat &image, std::vector<ImuInfo_s> &vImuDate)
{
    int nParis = static_cast<int>(vImuImgPairs_.size());
    if(index >= nParis)
    {
        std::cout<<"error: input index > total pairs, input: "<<index<<", total num: "<<nParis;
        return false;
    }
    std::pair<ImgInfo_s, std::vector<ImuInfo_s> >& pair = vImuImgPairs_[index];
    image = cv::imread(pair.first.imgPath);
    vImuDate = pair.second;
}

bool EurocInPacker::undistort(cv::Mat srcImg, cv::Mat& distImg)
{
    cv::Mat camK(3, 3, CV_32FC1);
    cv::Mat discoff(4, 1, CV_32FC1);
    cv::Mat mapx, mapy;
    if (mapx.empty())
    {// use map buffer to speed up
        cv::initUndistortRectifyMap(camK, discoff, cv::Mat(),
                                    camK, srcImg.size(), CV_32FC1, mapx, mapy);
    }
    if(distImg.empty())
    {
        distImg.create(srcImg.size(), srcImg.type());
    }
    
    cv::remap(srcImg, distImg, mapx, mapy, cv::INTER_LINEAR);

    return true;
}