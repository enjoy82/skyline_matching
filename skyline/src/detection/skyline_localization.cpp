#include "detection/skyline_localization.hpp"


namespace
{
    double rad2angle(double rad)
    {
        return (180.0 / M_PI) * rad;
    }

    double calc_angle(cv::Point2f temppt, cv::Point2f tarpt)
    {
        double dx = tarpt.x - temppt.x;
        double dy = tarpt.y - temppt.y;
        if(dy != 0)
            return rad2angle(std::atan2(dy, dx));
        else
            return 0.0;
    }
}

SkylineLocalization::SkylineLocalization(double gamma, double angle_th, double min_rate_th, double max_rate_th, int count_th, double pic_angle)
    : m_gamma(gamma)
    , m_angle_th(angle_th)
    , m_min_rate_th(min_rate_th)
    , m_max_rate_th(max_rate_th)
    , m_count_th(count_th)
    , m_pic_angle(pic_angle)
{
    //もしログを丁寧に撮りたいときはここを変える
    m_logdir_prefix = "log/";
}

cv::Mat SkylineLocalization::gammaControl(cv::Mat &img, bool debug = false){
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.data;
    for( int i = 0; i < 256; ++i){
        p[i] = pow(1.0*i/255, m_gamma) * 255;
    }
    // この置き換え処理をする関数が用意されている
    cv::LUT(img, lookUpTable, img);

    if(debug){
        std::string filename = m_logdir_prefix + "_0gamma_img.png";
        cv::imwrite(filename, img);
    }
    return img;
}

cv::Mat SkylineLocalization::gaussianFilter(cv::Mat &img, bool debug = false, int filtersize = 5, double sigma = 1.5){
    cv::GaussianBlur(img, img, cv::Size(filtersize, filtersize), 5);
    if(debug){
        std::string filename = m_logdir_prefix + "_1gaussian_img.png";
        cv::imwrite(filename, img);
    }
    return img;
}

//グレースケール変換
cv::Mat SkylineLocalization::convertGray(cv::Mat &img, bool debug = false){
    cv::Mat rgbChannel[3];
    cv::Mat grayImg;
    cv::split(img, rgbChannel);
    grayImg = rgbChannel[2];
    if(debug){
        std::string filename = m_logdir_prefix + "_2gray_img.png";
        cv::imwrite(filename, grayImg);
    }
    return grayImg;
}

cv::Mat SkylineLocalization::cannyEdge(cv::Mat &img, bool debug = false, double threshold1 = 100, double threshold2 = 200){
    cv::Canny(img, img, threshold1, threshold2, 5);
    if(debug){
        std::string filename = m_logdir_prefix + "_3canny_img.png";
        cv::imwrite(filename, img);
    }
    return img;
}


//FIXME デバッグしやすくする
cv::Mat SkylineLocalization::preProcessing(cv::Mat &img, bool debug = false){
    img = gammaControl(img, debug);
    img = gaussianFilter(img, debug);
    img = convertGray(img, debug);
    //img = cannyEdge(img, debug);
    return img;
}

//ひとまずマッチングでどれだけ精度が出るか調べる
double SkylineLocalization::calucurateAngle(cv::Mat &tarImg, std::string mode, bool debug = false){
    //前処理
    tarImg = this->preProcessing(tarImg, debug);
    //from https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
    cv::Mat tempImg;
    if(mode == "front")
        tempImg = this->m_frontImg;
    else
        tempImg = this->m_backImg;
    // 特徴点抽出 
    //TODO 前計算できる
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptor1, descriptor2;
    auto akaze = cv::AKAZE::create();
    akaze->detectAndCompute(tempImg, cv::noArray(), keypoint1, descriptor1);
    akaze->detectAndCompute(tarImg, cv::noArray(), keypoint2, descriptor2);

    //マッチング
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> match, match12, match21;
    matcher->match(descriptor1, descriptor2, match12);
    matcher->match(descriptor2, descriptor1, match21);

    //クロスチェック(1→2と2→1の両方でマッチしたものだけを残して精度を高める)
    for (size_t i = 0; i < match12.size(); i++)
    {
      cv::DMatch forward = match12[i];
      cv::DMatch backward = match21[forward.trainIdx];
      if (backward.trainIdx == forward.queryIdx)
      {
        match.push_back(forward);
      }
    }

    //有効な特徴量ペアを計算する
    //各特徴量点を+-5度の範囲かつ長さが一定の割合から逸脱していないものをカウントする
    std::vector<int> match_pair_count(match.size(), 0);
    for(int i = 0; i < match.size(); i++)
    {
        cv::Point2f temppt1 = keypoint1[match[i].queryIdx].pt;
        cv::Point2f tarpt1 =  keypoint1[match[i].trainIdx].pt;
        double angle_base = calc_angle(temppt1, tarpt1);
        double distance_base = match[i].distance;
        for(int l = 0; l < match.size(); l++)
        {
            cv::Point2f temppt2 = keypoint1[match[l].queryIdx].pt;
            cv::Point2f tarpt2 =  keypoint1[match[l].trainIdx].pt;
            double angle_tar = calc_angle(temppt2, tarpt2);
            double distance_tar = match[l].distance;
            if(abs(angle_base - angle_tar) < m_angle_th && (distance_base * m_min_rate_th < distance_tar && distance_tar < distance_base * m_max_rate_th))
            {
                match_pair_count[i]++;
            }
        }
    }
    int bestIdx = 0;
    for(int i = 0; i < match_pair_count.size(); i++){
        if(match_pair_count[bestIdx] < match_pair_count[i])
            bestIdx = i;
    }
    if(match_pair_count[bestIdx] < m_count_th){
        return -1;
    }

    cv::Point2f temppt = keypoint1[bestIdx].pt;
    cv::Point2f tarpt =  keypoint1[bestIdx].pt;
    double best_match_dx = temppt.x - tarpt.x;

    //画角とピクセル数から角度を算出する
    double pic_width = tempImg.size().width;
    double match_angle = best_match_dx * m_pic_angle / pic_width;

    if(debug)
    {
        // マッチング結果の描画
        cv::Mat dest;
        cv::drawMatches(tempImg, keypoint1, tarImg, keypoint2, match, dest);
        //マッチング結果の書き出し
        std::string filename = m_logdir_prefix + "_5matching.png";
        cv::imwrite(filename, dest);
    }
    return match_angle;
}


//／前処理を通す
void SkylineLocalization::snapStartPos(cv::Mat &img, std::string mode = "front", bool debug = false){
    if(mode == "front")
        this->m_frontImg = this->preProcessing(img, debug);
    else
        this->m_backImg = this->preProcessing(img, debug);
    if(debug){
        std::string front_filename = m_logdir_prefix + "_4templete_" + mode + "_img.png";
        cv::imwrite(front_filename, img);
    }
}
