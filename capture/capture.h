/*************************************/
// Author 长安大学
/*************************************/

#ifndef __CAPTURE_H__
#define __CAPTURE_H__


#include <opencv2/opencv.hpp>
#include "../param/param.hpp"

class Capture{
public:
	Capture(Config config);
	bool getImage(cv::Mat & _img);
private:
	cv::VideoCapture capture;
	cv::Mat frame;
};


#endif // __CAPTURE_H__