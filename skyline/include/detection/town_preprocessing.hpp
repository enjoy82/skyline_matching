#include "detection/preprocessing_base.hpp"

class TownPreprocessing : public PreprocessingBase
{
private:
    double m_gamma;
    cv::Mat gammaControl(cv::Mat &img);

    //ガウシアンフィルタ管連
    cv::Mat gaussianFilter(cv::Mat &img, int filtersize, double sigma);
    
    //グレースケール変換
    cv::Mat convertGray(cv::Mat &img);
public:
    TownPreprocessing(double gamma = 1.2);
    ~TownPreprocessing() = default;
    virtual cv::Mat preprocessing(cv::Mat &img) override;
};