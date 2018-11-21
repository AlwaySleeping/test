#include "Parser.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


void SensorConfig::init(const std::string &sensorConfigPath)
{
    loadSensorConfig(sensorConfigPath);
}

bool SensorConfig::loadSensorConfig(const std::string &sensorConfigPath)
{
    cv::FileStorage fSettings(sensorConfigPath, cv::FileStorage::READ);
    if (fSettings.isOpened())
    {
        // cam cofig params:
        fx_ = fSettings["Camera.fx"];
        fy_ = fSettings["Camera.fy"];
        cx_ = fSettings["Camera.cx"];
        cy_ = fSettings["Camera.cy"];
        
        k1_ = fSettings["Camera.k1"];
        k2_ = fSettings["Camera.k2"];
        p1_ = fSettings["Camera.p1"];
        p2_ = fSettings["Camera.p2"];
        
        camFps_ = fSettings["rate_hz"];
        
        k_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        discoff_<< k1_, k2_, p1_, p2_;
        
        //imu cofig params:
        cv::FileNode fs = fSettings["T_imu_cam"];
        auto it = fs.begin();
        for(std::size_t i=0; i<4; i++)
            for(std::size_t j=0; j<4; j++)
        {
            T_imu_cam_(i,j) = *(it ++);
        }
        
        ng_ = fSettings["gyroscope_noise_density"];
        nbg_ = fSettings["gyroscope_random_walk"];
        na_ = fSettings["accelerometer_noise_density"];
        nba_ = fSettings["accelerometer_random_walk"];
        
        std::cerr<<"********* camera imu param:"<<std::endl;
        std::cerr<<"camera K:\n"<<k_<<std::endl;
        std::cerr<<"T_imu_cam:\n"<<T_imu_cam_<<std::endl;
        std::cerr<<"na_, nba_, ng_, nbg_ :"<<na_<<","<<nba_<<","<<ng_<<","<<nbg_<<std::endl;

        fSettings.release();
    }
    else
    {
        std::cout<<"[config]: load sensor config error! path:"<<sensorConfigPath<<std::endl;
        return false;
    }
    return true;
    
}
