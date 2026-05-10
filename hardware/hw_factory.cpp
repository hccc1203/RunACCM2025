#include "../include/hw_interface.hpp"
#include <iostream>

#ifndef ENABLE_X86_MOCK
#include "../capture/capture.h"
#include "../include/uart.hpp"

class CameraAdapter : public ICamera {
public:
    explicit CameraAdapter(const Config &config) : capture_(config) {}
    bool getImage(cv::Mat &img) override { return capture_.getImage(img); }
private:
    Capture capture_;
};

class UartAdapter : public IUart {
public:
    explicit UartAdapter(const std::string &portName) : uart_(portName) {}
    int open() override { return uart_.open(); }
    void carControl(float speed, uint16_t servo, uint16_t flag) override { uart_.carControl(speed, servo, flag); }
private:
    Uart uart_;
};
#else
class MockCamera : public ICamera {
public:
    explicit MockCamera(const Config &) {}
    bool getImage(cv::Mat &img) override {
        static int tick = 0;
        img = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::line(img, cv::Point(tick % 320, 0), cv::Point(tick % 320, 239), cv::Scalar(0, 255, 0), 2);
        cv::putText(img, "x86 mock camera", cv::Point(40, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        ++tick;
        return true;
    }
};

class MockUart : public IUart {
public:
    explicit MockUart(const std::string &portName) : portName_(portName) {}
    int open() override { std::cout << "[MOCK UART] open " << portName_ << std::endl; return 0; }
    void carControl(float speed, uint16_t servo, uint16_t flag) override {
        std::cout << "[MOCK UART] speed=" << speed << " servo=" << servo << " flag=" << flag << std::endl;
    }
private:
    std::string portName_;
};
#endif

std::shared_ptr<ICamera> createCamera(const Config &config)
{
#ifndef ENABLE_X86_MOCK
    return std::make_shared<CameraAdapter>(config);
#else
    return std::make_shared<MockCamera>(config);
#endif
}

std::shared_ptr<IUart> createUart(const std::string &portName)
{
#ifndef ENABLE_X86_MOCK
    return std::make_shared<UartAdapter>(portName);
#else
    return std::make_shared<MockUart>(portName);
#endif
}
