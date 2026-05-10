#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "../param/param.hpp"

class ICamera {
public:
    virtual ~ICamera() = default;
    virtual bool getImage(cv::Mat &img) = 0;
};

class IUart {
public:
    virtual ~IUart() = default;
    virtual int open() = 0;
    virtual void carControl(float speed, uint16_t servo, uint16_t flag) = 0;
};

std::shared_ptr<ICamera> createCamera(const Config &config);
std::shared_ptr<IUart> createUart(const std::string &portName);
