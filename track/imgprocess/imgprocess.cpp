#include "imgprocess.h"

ImageProcess::ImageProcess() {}

ImageProcess::ImageProcess(Config config) {
    this->_config = config; 
}

//去除高光
cv::Mat ImageProcess::matilluminationChange(cv::Mat src) {
  cv::Mat gray, threshmat, dst;
  //复制出来改为灰度图
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  //直方图均衡化
  cv::equalizeHist(gray, gray);
  imshow("equalizeHist", gray);

  //二值化操作，定义大于210的即为高光
  cv::threshold(gray, threshmat, 210, 255, cv::THRESH_BINARY);

  //查找图片中高亮区域轮廓
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(threshmat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
  for (int i = 0; i < contours.size(); ++i) {
    cv::Rect rect = boundingRect(contours[i]);
    cv::rectangle(mask, rect, cv::Scalar(255), -1);
  }

  imshow("mask", mask);

  //去高光
  cv::illuminationChange(src, mask, dst, 1.0f, 0.1f);

  return dst;
}

cv::Mat ImageProcess::processImage(const cv::Mat & src_img) {
    cv::Mat result_img;
    // cv::resize(src_img, result_img, cv::Size(_config.img_width, _config.img_height));
    cv::cvtColor(src_img, result_img, cv::COLOR_BGR2GRAY);
    
    cv::threshold(result_img, result_img, 100, 255, cv::THRESH_OTSU);
    cv::morphologyEx(result_img, result_img, cv::MORPH_CLOSE, kernel);
    return result_img;
}

// 原图 -> 俯视  透视矩阵
cv::Mat warpMatrix =
    (cv::Mat_<float>(3, 3) << 4.830985915493104, 7.481690140845294,
     -559.9154929577662, 1.3374922084896e-14, 14.74647887323986,
     -725.1690140845323, 7.62235112648736e-17, 0.05070422535211418, 1);

// 俯视 -> 原图  透视矩阵
cv::Mat warpMatrixT =
    (cv::Mat_<float>(3, 3) << 0.7231273024969306, -0.503533906399236,
     39.74299358711961, 4.254746920719163e-16, 0.06781279847182463,
     49.17574021012407, 1.303269874835107e-18, -0.003438395415472779, 1);

// 透视变换 (0:原图 -> 俯视, 1:俯视 -> 原图)
void ImageProcess::mapPerspective(float x, float y, float loc[2], uint8_t mode) {
    float xx, yy, zz;

    if (mode == 0) {
        zz = warpMatrix.at<float>(2, 0) * x +
             warpMatrix.at<float>(2, 1) * y +
             warpMatrix.at<float>(2, 2);
        xx = (warpMatrix.at<float>(0, 0) * x +
              warpMatrix.at<float>(0, 1) * y +
              warpMatrix.at<float>(0, 2)) /
             zz;
        yy = (warpMatrix.at<float>(1, 0) * x +
              warpMatrix.at<float>(1, 1) * y +
              warpMatrix.at<float>(1, 2)) /
             zz;

        loc[0] = xx;
        loc[1] = yy;
    } else {
        zz = warpMatrixT.at<float>(2, 0) * x +
             warpMatrixT.at<float>(2, 1) * y +
             warpMatrixT.at<float>(2, 2);
        xx = (warpMatrixT.at<float>(0, 0) * x +
              warpMatrixT.at<float>(0, 1) * y +
              warpMatrixT.at<float>(0, 2)) /
             zz;
        yy = (warpMatrixT.at<float>(1, 0) * x +
              warpMatrixT.at<float>(1, 1) * y +
              warpMatrixT.at<float>(1, 2)) /
             zz;

        loc[0] = xx;
        loc[1] = yy;
    }
}
