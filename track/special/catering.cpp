/*************************************/
// Author 孙酩贺
/*************************************/

#include "catering.h"
#include "../../include/detection.hpp"
#include "../../include/logger.hpp"
#include "../standard/general.h"
#define CATERING_DEBUG 0


#define PREDICT_TIMES 2
#define PREDICT_THRESH 1

Catering::Catering(){}

Catering::Catering(Config &config){}

static int burger_count = 0;
static int start_park_timer = 0;
static std::queue<bool> recent_results;
static int park_timer = 0;
static int start_timer = 0;

void Catering::check_catering(std::vector<PredictResult> &predict_result, bool is_L_left_find,bool is_L_right_find) {

    if (CATERING_DEBUG) {
        logger("Debug mode, skipping check_catering");
        flag_catering = CATERING_ENTER;
        return;
    }

    logger("Detecting");
    static int exit_counter = 0;
    // Check if any predict_result[i].type == 4 in the current result
    bool has_burger = false;
    for (const auto &result : predict_result) {
        // logger(result.type);
        if (result.type == 3) {
            has_burger = true;
            break;
        }
    }
    // has_burger = true;

    // Update the queue of recent results
    if (recent_results.size() == PREDICT_TIMES) {
        if (recent_results.front()) {
            --burger_count;
        }
        recent_results.pop();
    }
    recent_results.push(has_burger);
    if (has_burger) {
        ++burger_count;
    }
    auto current_time = std::chrono::steady_clock::now();
    // 检查是否处于保持期内
    if (is_holding) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - hold_start_time).count();

        if (elapsed < 2) {
            flag_catering = CATERING_ENTER; // 保持期内强制设为汉堡模式
            return;
        } 
        else if(elapsed > 2)
        {
            is_holding = false; // 保持期结束   
        }
    }

    // Check if the threshold is met
    if (burger_count >= PREDICT_THRESH) {
        flag_catering = CATERING_ENTER;
        is_holding = true;
        hold_start_time = current_time; // 记录开始时间
        Catering::logger("Burger detected.");
    } else {
        // logger("catering_none");
        flag_catering = CATERING_NONE;
    }
 
}

void Catering::run_catering(std::vector<PredictResult> &predict_result,cv::Mat &raw_src_img, vector<POINT> &pointsEdgeLeft,
                            vector<POINT> &pointsEdgeRight,
                            vector<POINT> &AI_CenterEdge,
                            int pointsEdgeRight_size, int pointEdgeLeft_size,
                             bool is_L_left_find,bool is_L_right_find) {
    
    if (raw_src_img.empty()) {
        logger("Error: src_img is empty.");
        return;
    }



    logger("Running catering");
    //检测汉堡的位置
    
    for(auto res : predict_result)
    {
       if(res.type == LABEL_BURGER)
       {
        if(res.x < COLSIMAGE / 2)
        turn_dir = TURN_RIGHT;
        else{
        turn_dir = TURN_LEFT;
        }
       }
    }

    
        if(turn_dir == TURN_LEFT)
    {
        if(pointsEdgeRight_size == 0)
        {
            flag_catering = CATERING_SIZE;
            // park_enable = true;
            // park_timer++;
        }
    }
    else
    {
        if(pointEdgeLeft_size == 0)
        {
            flag_catering = CATERING_SIZE;
            // park_enable = true;
            // park_timer++;
        }
    }
    if(flag_catering == CATERING_SIZE)
    {
        if(turn_dir == TURN_LEFT)
        {
            if(pointsEdgeRight_size)
            {
                park_enable = true;
            }
        }
        if(turn_dir == TURN_RIGHT)
        {
            if(pointEdgeLeft_size)
            {
                park_enable = true;
            }
        }
    }
    if(park_enable)
    {
        flag_catering = CATERING_PARK;
        printf("PARKING!!!!!!!!!!!!!\n");
        park_timer++;
    }
    if(park_timer >= 30)
    {
        flag_catering = CATERING_NONE;           ;
    }
    // }
    // if(park_timer >= 1)
    // {
    //     printf("............");
    //     if(pointEdgeLeft_size)
    //     {
    //         start_timer++;
    //         printf("================");
    //         if(start_timer <= 30)
    //         {
    //             // output_speed_rato = 0.0;
    //             flag_catering = CATERING_EXIT;
    //         }
    //     }
    //     else
    //     {
    //         // output_speed_rato = -1.0;
    //     }
    // }

    timer--;
    // printf("timer is %d",timer);

    // // 如果 timer 小于 20，速度设为 0
    // if (abs(timer) >= CATERING_STOP_TIMESTAMP && abs(timer) <= 2 * CATERING_STOP_TIMESTAMP) {
        
    //     output_speed_rato = (1.0 / abs(timer) * CATERING_STOP_TIMESTAMP);
    //     // output_speed_rato = 0.0;
    // } else {
    //     output_speed_rato = -1.0;
    // }


}

// 添加辅助函数
// bool Catering::isPointWhite(cv::Mat &image, int x, int y) {
//     cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);
//     return pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255;
// }

// pair<vector<int>, vector<int>>
// Catering::find_left_right_white_pixels(const Mat &mask) {
//     Mat gray, thresh;
//     if (mask.channels() > 1) {
//         cvtColor(mask, gray, COLOR_BGR2GRAY);
//     } else {
//         gray = mask.clone();
//     }
//     threshold(gray, thresh, 127, 255, THRESH_BINARY);

//     vector<int> left_coords(thresh.rows + 5, -1);
//     vector<int> right_coords(thresh.rows + 5, -1);

//     int pre_x1 = -1, pre_x2 = -1;
//     bool bad_x1 = false, bad_x2 = false;

//     for (int y = thresh.rows - 1; y >= 0; y--) {
//         const uchar *row = thresh.ptr<uchar>(y);
//         // Find left
//         if (!bad_x1)
//             for (int x = 0; x < thresh.cols; ++x) {
//                 if (row[x] == 255) {
//                     if (pre_x1 == -1) {
//                         pre_x1 = x;
//                     }

//                     if (abs(x - pre_x1) > 50) {
//                         bad_x1 = true;
//                         break;
//                     }

//                     left_coords[y] = x;
//                     pre_x1 = x;
//                     break;
//                 }
//             }

//         // Find right
//         if (!bad_x2)
//             for (int x = thresh.cols - 1; x >= 0; --x) {
//                 if (row[x] == 255) {
//                     if (pre_x2 == -1) {
//                         pre_x2 = x;
//                     }

//                     if (abs(x - pre_x2) > 50) {
//                         bad_x2 = true;
//                         break;
//                     }

//                     right_coords[y] = x;
//                     pre_x2 = x;
//                     break;
//                 }
//             }
//     }

//     return {left_coords, right_coords};
// }