#include <iostream>
#include <string>
#include <glob.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "detection/skyline_localization.hpp"

namespace{
    std::string temp_img_paths = "/Users/naoya/code/skyline_matching/pics/test.png";
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
}

int main(){
    std::vector<std::string> tar_img_paths = get_file_path(tar_img_dir);
    SkylineLocalization *skyline = new SkylineLocalization();
    cv::Mat temp_img = cv::imread(temp_img_paths);
    skyline->snapStartPos(temp_img, "front", true);
    for(int i = 0; i < tar_img_paths.size(); i++){
        cv::Mat tar_img = cv::imread(tar_img_paths[i]);
        double angle = skyline->calucurateAngle(tar_img, "front", true);
        std::cout << "target : " << tar_img_paths[i] << " angle : " << angle << " °" << std::endl;
    }
    return 0;
}