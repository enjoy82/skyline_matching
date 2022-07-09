#include "detection/skyline_localization.hpp"
#include <cmath>

SkylineLocalization::SkylineLocalization(double init_angle, cv::Mat &panorama_img, PreprocessingBase &preprocessinger)
    : m_panorama_img(panorama_img)
    , m_init_angle(init_angle)
{
    m_preprocessinger = &preprocessinger;
}

double SkylineLocalization::calucurateAngle(cv::Mat &img)
{
    img = m_preprocessinger->preprocessing(img);
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptor1, descriptor2;
    auto akaze = cv::AKAZE::create();
    akaze->detectAndCompute(img, cv::noArray(), keypoint1, descriptor1);
    akaze->detectAndCompute(m_panorama_img, cv::noArray(), keypoint2, descriptor2);
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
        //TODO あとは移動距離を算出して，相対角度を算出するだけ！！！！！
        return 0.0;
    }
}