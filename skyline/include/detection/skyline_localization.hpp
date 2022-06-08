#pragma once
#ifndef ROVER_NO_OPENCV
#include <opencv2/opencv.hpp>
#else
namespace cv
{
    class Mat;
}
#endif
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <cmath>

class SkylineLocalization
{
private:
    std::string m_logdir_prefix;
    //スタート位置の前後画像
    cv::Mat m_frontImg, m_backImg;
    cv::Mat preProcessing(cv::Mat &img, bool debug);

    //gamma処理関連
    //gamma = 1　変化なし gamma < 1　暗くなる gamma > 1　明るくなる
    double m_gamma;
    cv::Mat gammaControl(cv::Mat &img, bool debug);

    //ガウシアンフィルタ管連
    cv::Mat gaussianFilter(cv::Mat &img, bool debug, int filtersize, double sigma);
    
    //グレースケール変換
    cv::Mat convertGray(cv::Mat &img, bool debug);
    
    //canny検出器
    cv::Mat cannyEdge(cv::Mat &img, bool debug, double threshold1, double threshold2);
    
    double m_angle_th;
    double m_min_rate_th;
    double m_max_rate_th;
    int m_count_th;
    //カメラの視野角
    double m_pic_angle;
public:
    //初期値をここにセット
    explicit SkylineLocalization(double gamma = 0.2, double angle_th = 5.0, double min_rate_th = 0.8, double max_rate_th = 1.2, int count_th = 7, double pic_angle = 120.0);
    ~SkylineLocalization() = default;
    //開始地点の画像をとる
    void snapStartPos(cv::Mat &img, std::string mode, bool debug);
     //AKAZEによる確度推定
    double calucurateAngle(cv::Mat &tarImg, std::string mode, bool debug);
};