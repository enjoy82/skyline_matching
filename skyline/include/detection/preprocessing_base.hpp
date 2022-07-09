#pragma once
#include <opencv2/opencv.hpp>

class PreprocessingBase
{
private:
public:
    PreprocessingBase() = default;
    ~PreprocessingBase() = default;
    virtual cv::Mat preprocessing(cv::Mat &img)
    {
        return img;
    }
};