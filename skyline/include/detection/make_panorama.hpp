#pragma once
//TODO コンパイル切り分け
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include<memory>
#include "detection/preprocessing_base.hpp"
#include "detection/make_panorama.hpp"


class MakePanorama // パノラマ画像を作るためのdetection
{
private:
    //パノラマ画像 360度回転すると仮定して，角度情報を計算する．
    cv::Ptr<cv::Stitcher> m_stiticher;
    cv::Mat m_panorama_img;
    //パノラマ画像生成用画像群
    std::vector<cv::Mat> m_source_front_imgs, m_source_back_imgs;
    //パノラマ画像が生成されたか
    bool m_finished;
    //初期値の角度情報　(どうする？？)
    double m_init_angle;
    bool finishedPanorama(cv::Mat &img);
    void MakePanoramaImg();
    //前処理は分岐できるように切り離す
    PreprocessingBase* m_preprocessinger;
public:
    explicit MakePanorama(PreprocessingBase &preprocessinger);
    ~MakePanorama() = default;
    //初回設定時のみ角度情報を与える
    void initAngle(double angle);
    bool setImage(cv::Mat &front_img, cv::Mat &back_img);
    void getPanorama(cv::Mat &panorama_img);
    double getAngle();
};