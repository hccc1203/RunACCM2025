/*************************************/
// Author 长安大学
/*************************************/

#include "./capture.h"


#include <unistd.h>
#include <iostream>

Capture::Capture(Config config) {
	this->capture = cv::VideoCapture("/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0",cv::CAP_V4L2);

	if (!capture.isOpened())
    {
        printf("can not open video device!!!\n");
        exit(0);
    }
	std::cout << "Capture connect successfully." << std::endl;
	
	this->capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	this->capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	this->capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	this->capture.set(CV_CAP_PROP_FPS, 120);
	this->capture.set(CV_CAP_PROP_CONTRAST, 0.8); 
	this->capture.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.25);
	this->capture.set(CV_CAP_PROP_EXPOSURE, config.exposure);
	this->capture.set(cv::CAP_PROP_AUTO_WB, 0);
	this->capture.set(cv::CAP_PROP_WB_TEMPERATURE, config.temprature);
	this->capture.set(CV_CAP_PROP_BRIGHTNESS, config.brightness);
	this->capture.set(CV_CAP_PROP_SATURATION, 0.5);
	
}

bool Capture::getImage(cv::Mat & _img) {
	this->capture.read(this->frame);
	if (this->frame.empty()) {
		std::cout << "Read image faild..." << std::endl;
		return false;	
	}
	_img = this->frame.clone();
	// _img = _img(cv::Rect(20 , 35 , 600 , 400));
	// usleep(10000);
	return true;
}