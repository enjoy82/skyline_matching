#include "detection/town_preprocessing.hpp"

TownPreprocessing::TownPreprocessing(double gamma)
    : m_gamma(gamma)
{
}

cv::Mat TownPreprocessing::gammaControl(cv::Mat &img){
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.data;
    for( int i = 0; i < 256; ++i){
        p[i] = pow(1.0*i/255, m_gamma) * 255;
    }
    // この置き換え処理をする関数が用意されている
    cv::LUT(img, lookUpTable, img);

    return img;
}


cv::Mat TownPreprocessing::gaussianFilter(cv::Mat &img, int filtersize = 5, double sigma = 1.5){
    cv::GaussianBlur(img, img, cv::Size(filtersize, filtersize), 5);
    return img;
}

//グレースケール変換
cv::Mat TownPreprocessing::convertGray(cv::Mat &img){
    cv::Mat rgbChannel[3];
    cv::Mat grayImg;
    cv::split(img, rgbChannel);
    grayImg = rgbChannel[2];
    return grayImg;
}


cv::Mat TownPreprocessing::preprocessing(cv::Mat &img)
{
    img = this->gammaControl(img);
    img = this->gaussianFilter(img);
    img = this->convertGray(img);
    return img;
}