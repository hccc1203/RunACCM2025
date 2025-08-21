/*************************************/
// Author 胡城玮
/*************************************/

#include "bridge.h"
#include "../../include/detection.hpp"
#include "../../include/logger.hpp"
#include "../standard/general.h"
#define PREDICT_TIMES 5
#define PREDICT_THRESH 2


Bridge::Bridge(){}
Bridge::Bridge(Config &config){}
static int bridge_count = 0;
static std::queue<bool> recent_results;

void Bridge::check_bridge(std::vector<PredictResult> &predict_result,cv::Mat &src_img,float pitch_angle){
    static int timer = 0;
    for (const auto &result : predict_result) {
        // logger(result.type);
        if (result.type == LABEL_BRIDGE) {
            flag_bridge = BRIDGE_ENTER;
            break;
        }
    }

}

void Bridge::run_bridge(){

}