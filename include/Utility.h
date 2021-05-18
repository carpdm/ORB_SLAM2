
#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);

};


class SingleInstance {
public:
    // some public vars and configurations.
    static ORBVocabulary * GetVoc() {
        static ORBVocabulary* pORBVocabulary;
        if (pORBVocabulary == nullptr) {
            pORBVocabulary = new ORBVocabulary ();
        }
        return pORBVocabulary;
    }

};

class Config {
public:

    inline static float fx() {return Config::settings["Camera.fx"];}
    inline static float invfx() {return 1.0 / fx();}
    inline static float invfy() {return 1.0 / fy();}
    inline static float fy() {return Config::settings["Camera.fy"];}
    inline static float cx() {return Config::settings["Camera.cx"];}
    inline static float cy() {return Config::settings["Camera.cy"];}
    inline static float k1() {return Config::settings["Camera.k1"];}
    inline static float k2() {return Config::settings["Camera.k2"];}
    inline static float k3() {return Config::settings["Camera.k3"];}
    inline static float p1() {return Config::settings["Camera.p1"];}
    inline static float p2() {return Config::settings["Camera.p2"];}
    inline static float bf() {return Config::settings["Camera.bf"];}
    inline static float b() {return bf() / fx();}
    inline static float fps() {return Config::settings["Camera.fps"];}
    inline static int rgb() {return Config::settings["Camera.RGB"];}
    inline static float depth_factor() {return Config::settings["DepthMapFactor"];}

    inline static int nFeatures() { return Config::settings["ORBextractor.nFeatures"];}
    inline static float fScaleFactor()  {return Config::settings["ORBextractor.scaleFactor"];}
    inline static int nLevels() {return Config::settings["ORBextractor.nLevels"]; }
    inline static int fIniThFAST() { return Config::settings["ORBextractor.iniThFAST"]; }
    inline static int fMinThFAST() { return Config::settings["ORBextractor.minThFAST"]; }
    inline static float ThDepth() {return bf() * (float) Config::settings["ThDepth"] / fx(); }
    //


    static cv::FileStorage settings;
    static cv::Mat K;
    static cv::Mat DistCoef;

    static void LoadConfig(const string& filename) {
        Config::settings.open(filename, cv::FileStorage::READ);
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx();
        K.at<float>(1,1) = fy();
        K.at<float>(0,2) = cx();
        K.at<float>(1,2) = cy();
        K.copyTo(Config::K);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = k1();
        DistCoef.at<float>(1) = k2();
        DistCoef.at<float>(2) = p1();
        DistCoef.at<float>(3) = p2();
        const float _k3 = k3();
        if(_k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = _k3;
        }
        DistCoef.copyTo(Config::DistCoef);

    }

};





}// namespace ORB_SLAM

#endif // CONVERTER_H
