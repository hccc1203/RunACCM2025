/*************************************/
// Author 胡城玮
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



class Bridge{
    public:
        Bridge();
        Bridge(Config &config);

        enum flag_bridge_e {
            BRIDGE_NONE,
            BRIDGE_ENTER,
            BRIDGE_DOWN,
        };

        flag_bridge_e flag_bridge;
        General general;
        

        void check_bridge(std::vector<PredictResult> &predict_result,cv::Mat &src_img,float pitch_angle);
        void run_bridge();
        bool is_holding = false;
        std::chrono::time_point<std::chrono::steady_clock> hold_start_time; // 保持开始时间

        private:
           Logger logger = Logger("bridge");
};
// "aim_distance_far" : 0.6,
// "aim_distance_near" : 0.43,
// "steering_p_k" : 0.89,
// "ring_entering_p_k" :-0.0004,
// "ring_p_k" : -0.0004,
// "steering_p" :0.89,
// "steering_d" : 1.2,
// "curve_p":1,
// "curve_d":1,
// "ring_d":1.0,