#pragma once
//TODO コンパイル切り離し
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include "detection/preprocessing_base.hpp"
#include "detection/make_panorama.hpp"

//TODO

class SkylineLocalization
{
private:
    double m_init_angle;
    cv::Mat m_panorama_img;
    PreprocessingBase* m_preprocessinger;
public:
    //初期値をここにセット
    explicit SkylineLocalization(double init_angle, cv::Mat &panorama_img, PreprocessingBase &preprocessinger);
    ~SkylineLocalization() = default;
     //AKAZEによる確度推定
    double calucurateAngle(cv::Mat &img);
};