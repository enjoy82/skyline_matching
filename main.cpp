#include <iostream>
#include <string>
#include <glob.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "detection/skyline_localization.hpp"

namespace{
    std::string temp_img_path = "/Users/naoya/code/skyline_matching/pics/albelt2_1.png";
    std::string tar_img_dir = "/Users/naoya/code/skyline_matching/pics/";

    std::vector<std::string> get_file_path(std::string input_dir) {
        glob_t globbuf;
        std::vector<std::string> files;
        glob((input_dir + "*.png").c_str(), 0, NULL, &globbuf);
        for (int i = 0; i < globbuf.gl_pathc; i++) {
            files.push_back(globbuf.gl_pathv[i]);
        }
        globfree(&globbuf);
        return files;
    }
    double img_width = 800.0;
    double img_height = 600.0;
}

int main(){
    std::vector<std::string> tar_img_paths = get_file_path(tar_img_dir);
    SkylineLocalization *skyline = new SkylineLocalization("albelt1");
    cv::Mat temp_img = cv::imread(temp_img_path);
    cv::resize(temp_img, temp_img, cv::Size(), img_width/temp_img.cols ,img_height/temp_img.rows);
    skyline->snapStartPos(temp_img, "front", true);
    for(int i = 0; i < 2; i++){
        std::cout << tar_img_paths[i] << std::endl;
        cv::Mat tar_img = cv::imread(tar_img_paths[i]);
        cv::resize(tar_img, tar_img, cv::Size(), img_width/tar_img.cols ,img_height/tar_img.rows);
        double angle = skyline->calucurateAngle(std::to_string(i), tar_img, "front", true);
        std::cout << "target : " << tar_img_paths[i] << " angle : " << angle << " °" << std::endl;
    }
    return 0;
}