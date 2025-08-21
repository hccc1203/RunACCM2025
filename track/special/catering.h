/*************************************/
// Author 孙酩贺
/*************************************/


#pragma once

#include "../../include/detection.hpp"
#include "../../include/json.hpp"
#include "../../include/logger.hpp"
#include "../../param/param.hpp"
#include "../imgprocess/imgprocess.h"
#include "../standard/general.h"
#include <opencv2/opencv.hpp>
#include <vector>

class Catering {
  public:
    Catering();
    Catering(Config &config);

    enum flag_catering_e {
        CATERING_NONE,
        CATERING_SIZE,
        CATERING_PARK,
        CATERING_ENTER,
        CATERING_EXIT,
    };

    enum turn_direction { TURN_LEFT, TURN_RIGHT };

    flag_catering_e flag_catering;
    turn_direction turn_dir;
    General general;

    void check_catering(std::vector<PredictResult> &predict_result, bool is_L_left_find,bool is_L_right_find);
    void run_catering(std::vector<PredictResult> &predict_result,cv::Mat &src_img, vector<POINT> &pointsEdgeLeft,
                      vector<POINT> &pointsEdgeRight,
                      vector<POINT> &AI_CenterEdge, int pointsEdgeRight_size,
                      int pointEdgeLeft_size,bool is_L_left_find,bool is_L_right_find);

    int timer = 0;
    bool park_enable = false;
    bool is_holding = false;
    bool both_L_flag = false;
    bool exiting_flag = false;
    std::chrono::time_point<std::chrono::steady_clock> hold_start_time; // 保持开始时间
    int  CATERING_STOP_TIMESTAMP = 15;


  private:
    Logger logger = Logger("Catering");

    bool isPointWhite(cv::Mat &image, int x, int y);
    pair<vector<int>, vector<int>> find_left_right_white_pixels(const Mat &mask);
    // std::chrono::steady_clock::time_point first_detection_time;  // 首次检测到汉堡的时间点
    // bool is_detection_started;                                   // 是否已开始计时
    // bool parking_completed;                                      // 停车是否完成
    // static constexpr int FIXED_SECONDS = 3;                      // 固定停车时间
};