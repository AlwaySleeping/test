
#include "System.h"
#include "EurocInPacker.h"
#include "Parser.h"

int main(int argc, char **argv)
{
    std::string casePath = "/media/psf/Home/Documents/videos/kitti-mav/EuRoCMAVDataset/MH_01_easy/mav0";
    std::string sensorCfgPath = "/media/psf/AllFiles/Volumes/Project/git/test/config/cam_imu.yaml"; 
    std::string AlgoCfgPath = ""; //TODO

    EurocInPacker* pEuPacker = new EurocInPacker();
    SensorConfig*  pSensCfg = new SensorConfig();
    AlgoConfig*    pAlgoCfg = new AlgoConfig();

    pEuPacker->init(casePath);
    pSensCfg->init(sensorCfgPath);

    SLAM::System* pSystem = new SLAM::System();
    pSystem->init(pEuPacker, pAlgoCfg, pSensCfg);

    pSystem->run();

}

