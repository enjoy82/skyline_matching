#include "detection/make_panorama.hpp"

MakePanorama::MakePanorama(PreprocessingBase &preprocessinger)
    : m_init_angle(0.0)
{
    this->m_preprocessinger = &preprocessinger;
}

bool MakePanorama::finishedPanorama(cv::Mat &img)
{
    //パノラマ画像を作成する材料が揃った場合, マッチングして，frontがbackに重なるものがある場合trueを返す
    //TODO next writing
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptor1, descriptor2;
    auto akaze = cv::AKAZE::create();
    akaze->detectAndCompute(img, cv::noArray(), keypoint1, descriptor1);
    akaze->detectAndCompute(m_source_back_imgs[0], cv::noArray(), keypoint2, descriptor2);
    //マッチング
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches, match12, match21;
    matcher->match(descriptor1, descriptor2, match12);
    matcher->match(descriptor2, descriptor1, match21);

    //クロスチェック(1→2と2→1の両方でマッチしたものだけを残して精度を高める)
    for (size_t i = 0; i < match12.size(); i++)
    {
      cv::DMatch forward = match12[i];
      cv::DMatch backward = match21[forward.trainIdx];
      if (backward.trainIdx == forward.queryIdx)
      {
        matches.push_back(forward);
      }
    }
    //from michibiki いけるのか？
    double minDistance = 1e9;
    for (auto && match : matches)
    { 
        double distance = match.distance;
        if (distance < minDistance)
        {
            minDistance = distance;
        }
    }
    // TODO 角度情報を足す
    // 良いペアのみ残す
    std::vector<cv::DMatch> goodMatches;
    for (auto && match : matches)
    {
        if (match.distance < 3.0 * minDistance)
        {
            goodMatches.push_back(match);
        }
    }

    // 十分な対応点がある
    if (goodMatches.size() > 3)
    {
        return true;
    }

    return false;
}

void MakePanorama::initAngle(double angle)
{
    //angleの初期化をする関数
    m_init_angle = angle;
}

void MakePanorama::MakePanoramaImg()
{
    //パノラマ画像を作る関数
    for(unsigned int i = 0; i < m_source_back_imgs.size(); i++){
        m_source_front_imgs.push_back(m_source_back_imgs[i]);
    }
    cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
    m_stiticher = cv::Stitcher::create(mode);
    m_stiticher->stitch(m_source_front_imgs, m_panorama_img);
    m_finished = true;
}

bool MakePanorama::setImage(cv::Mat &front_img, cv::Mat &back_img)
{
    front_img = m_preprocessinger->preprocessing(front_img);
    back_img = m_preprocessinger->preprocessing(back_img);
    if(this->finishedPanorama(front_img)){
        MakePanoramaImg();
        return true;
    }
    m_source_front_imgs.push_back(front_img);
    m_source_back_imgs.push_back(back_img);
    return false;
}

void MakePanorama::getPanorama(cv::Mat &panorama_img)
{
    panorama_img = m_panorama_img;
    return;
}

double MakePanorama::getAngle()
{
    return m_init_angle;
}