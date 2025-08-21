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

class Layby{
public:
    Layby();
    Layby(Config &config);
    enum flag_layby_e{
        LAYBY_NONE,
        LAYBY_ENTER,
        LAYBY_PARKING,
    };
    enum flag_near_e{
        NEAR_RIGHT,
        NEAR_LEFT
    };
    flag_layby_e flag_layby;
    flag_near_e flag_near;
    General general;



    void check_layby(cv::Mat & src_img,std::vector<PredictResult> &predict_result);
    void run_layby(cv::Mat & src_img,std::vector<PredictResult> &predict_result,vector<POINT> & pointsEdgeLeft, vector<POINT> & pointsEdgeRight,
    vector<POINT> & AI_CenterEdge,float &output_speed_rato);

    bool park_enable = false;
    bool is_holding = false;
    std::chrono::time_point<std::chrono::steady_clock> hold_start_time; // 保持开始时间
    int park_timer = 0;
    int LAYBY_SHIFT_LEFT=10;//左边线偏移量
    int LAYBY_SHIFT_RIGHT=20;//右边线偏移量
    int LAYBY_PARKING_TIMER=70;//停车时间
private:
    Logger logger = Logger("layby");

};