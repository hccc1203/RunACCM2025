/*************************************/
// Author 孙酩贺
/*************************************/

#include "crosswalk.h"
#include "../standard/general.h"
#include "../../include/logger.hpp"

Crosswalk::Crosswalk() {}

Crosswalk::Crosswalk(Config &config) {

}
static int layby_count = 0;
static std::queue<bool> recent_results;
static int park_timer = 0;
static int start_timer = 0;
void Crosswalk::check_crosswalk(std::vector<PredictResult> &predict_result)
{
    // for (const auto &result : predict_result) {
    //     // logger(result.type);
    //     if (result.type == 7) {
    //         flag_crosswalk = CROSSWALK_NORMAL;
    //         break;
    //     }
    // }
    // auto current_time = std::chrono::steady_clock::now();
    // // 检查是否处于保持期内
    // if (is_holding) {
    //     auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - hold_start_time).count();
    //     if (elapsed < 2) {
    //         flag_crosswalk = CROSSWALK_NORMAL; // 保持期内强制设为临时停车区模式
    //         return;
    //     } else {
    //         is_holding = false; // 保持期结束
    //     }
    // }
    int consecutive_detections = 0;
    auto current_time = std::chrono::steady_clock::now();
    
    // 检查是否处于保持期内
    if (is_holding) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - hold_start_time).count();
        if (elapsed < 4) {
            flag_crosswalk = CROSSWALK_NORMAL; // 保持期内强制设为临时停车区模式
            return;
        } else {
            is_holding = false; // 保持期结束
            consecutive_detections = 0; // 重置连续检测计数器
        }
    }

    // 检查当前帧是否检测到斑马线
    bool detected_current_frame = false;
    for (const auto &result : predict_result) {
        if (result.type == 6 && result.score >= 0.8) {
            flag_crosswalk = CROSSWALK_NORMAL;
            detected_current_frame = true;
            break;
        }
    }

    // // 更新连续检测状态
    // if (detected_current_frame) {
    //     consecutive_detections++; // 增加连续检测计数器
        
    //     // 如果连续三帧都检测到斑马线
    //     if (consecutive_detections >= 2) {
    //         flag_crosswalk = CROSSWALK_NORMAL; // 进入斑马线模式
    //         is_holding = true; // 启动保持期
    //         hold_start_time = current_time; // 记录保持期开始时间
    //         consecutive_detections = 0; // 重置计数器
    //     }
    // } else {
    //     consecutive_detections = 0; // 如果当前帧未检测到，重置计数器
    // }

    // 如果没有检测到斑马线且不在保持期，重置状态
    // if (!is_holding && consecutive_detections == 0) {
    //     flag_crosswalk = CROSSWALK_NONE;
    // }


    
}

void Crosswalk::run_crosswalk(std::vector<PredictResult> &predict_result)
{
    // flag_crosswalk = CROSSWALK_NONE;
    // check_crosswalk(predict_result);
    // 静态变量用于帧计数
    static int no_detect_count = 0;  // 连续未检测到目标的帧数
    static int parkingConfirmCount = 0;  // 确认停车状态的计数器
    
    // 检查当前帧是否检测到目标
    bool current_frame_detected = false;
    for (const auto &result : predict_result) {
        if (result.type == 6) {
            current_frame_detected = true;
            break;
        }
    }
    
    // 更新连续未检测计数器
    if (!current_frame_detected) {
        no_detect_count++;
    } else {
        no_detect_count = 0;  // 重置计数器
    }
    // if(no_detect_count >= 5)
    // {
    //     flag_crosswalk = CROSSWALK_STOP;
    //     printf("1111111111111\n");
    //     park_enable = true;
    // }
    // flag_crosswalk = CROSSWALK_NONE;
    // check_crosswalk(predict_result);
    // if(flag_crosswalk == CROSSWALK_NORMAL)
    // {
    //     park_enable = true;
    // }
    for (const auto &result : predict_result) {
        if (result.type == 6 && result.score >= 0.8 && result.y >= STOP_EXPECT_Y) {
            flag_crosswalk = CROSSWALK_STOP;
            break;
        }
    }

}