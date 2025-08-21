#pragma once

#include "../../param/param.hpp"
#include <opencv2/opencv.hpp>

class ImageProcess {
private:
    Config _config;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
public:
    ImageProcess();
    ImageProcess(Config config);
    cv::Mat processImage(const cv::Mat & src_img);
    void mapPerspective(float x, float y, float loc[2], uint8_t mode);
    cv::Mat matilluminationChange(cv::Mat src);
};
