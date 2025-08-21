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

class Crosswalk{
    public:
       Crosswalk();
       Crosswalk(Config &config);
       enum flag_crosswalk_e{
            CROSSWALK_NORMAL,
            CROSSWALK_NONE,
            CROSSWALK_STOP
       };
       flag_crosswalk_e flag_crosswalk = CROSSWALK_NONE;
       General general;

       void check_crosswalk(std::vector<PredictResult> &predict_result);
       void run_crosswalk(std::vector<PredictResult> &predict_result);
       bool is_holding = false;
       std::chrono::time_point<std::chrono::steady_clock> hold_start_time; // 保持开始时间
       int park_timer = 0;
       bool park_enable = false;
      int STOP_EXPECT_Y = 150;
   private:
      Logger logger = Logger("Crosswalk");


};