/*************************************/
// Author 孙酩贺
/*************************************/

#include "layby.h"
#include "../standard/general.h"
#include "../../include/logger.hpp"
#define PREDICT_TIMES 10
#define PREDICT_THRESH 4

Layby::Layby() {}

Layby::Layby(Config &config) {

}
static int layby_count = 0;
static std::queue<bool> recent_results;
static int park_timer = 0;
static int start_timer = 0;
void Layby::check_layby(cv::Mat & src_img,std::vector<PredictResult> &predict_result)
{
        bool has_layby = false;
    for (const auto &result : predict_result) {
        // logger(result.type);
        if (result.type == 4||result.type == 8) {
            has_layby = true;
            break;
        }
    }

    // Update the queue of recent results
    if (recent_results.size() == PREDICT_TIMES) {
        if (recent_results.front()) {
            --layby_count;
        }
        recent_results.pop();
    }
    recent_results.push(has_layby);
    if (has_layby) {
        ++layby_count;
    }
    auto current_time = std::chrono::steady_clock::now();
    // 检查是否处于保持期内
    if (is_holding) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - hold_start_time).count();
        if (elapsed < 2) {
            flag_layby = LAYBY_ENTER; // 保持期内强制设为临时停车区模式
            return;
        } else {
            is_holding = false; // 保持期结束
        }
    }

    // Check if the threshold is met
    if (layby_count >= PREDICT_THRESH) {
        flag_layby = LAYBY_ENTER;
        is_holding = true;
        hold_start_time = current_time; // 记录开始时间
        Layby::logger("Layby detected.");
    } else {
        flag_layby = LAYBY_NONE;
    }
    
}

void Layby::run_layby(cv::Mat & src_img,std::vector<PredictResult> &predict_result,vector<POINT> & pointsEdgeLeft, vector<POINT> & pointsEdgeRight,
    vector<POINT> & AI_CenterEdge,float &output_speed_rato)
{
        flag_layby = LAYBY_NONE;
        check_layby(src_img, predict_result);

    
    // 静态变量用于帧计数
    static int no_detect_count = 0;  // 连续未检测到目标的帧数
    static int parkingConfirmCount = 0;  // 确认停车状态的计数器
    
    // 检查当前帧是否检测到目标
    bool current_frame_detected = false;
    for (const auto &result : predict_result) {
        if (result.type == 4 || result.type == 8) {
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
    switch(flag_layby)
    {
        case LAYBY_ENTER:{
        vector<Point> control_points;
        for(auto res : predict_result)
    {
        if(res.x < COLSIMAGE / 2)
        {
            flag_near = NEAR_LEFT;
        }
        else{
            flag_near = NEAR_RIGHT;
        }
    }
    //中线规划
    AI_CenterEdge.clear();
    if(flag_near == NEAR_LEFT)
    {
    
        for (size_t i = 0; i < pointsEdgeLeft.size() && i < pointsEdgeRight.size(); ++i) {
            
            
                int mid_x = (pointsEdgeLeft[i].x + pointsEdgeRight[i].x) / 2;
            int shifted_x = mid_x - LAYBY_SHIFT_LEFT;
            shifted_x = std::max(shifted_x, 0); // 确保不超出左边界
            
            // 保持原有y坐标
            AI_CenterEdge.push_back({shifted_x, pointsEdgeLeft[i].y});
            
        }
    
        


    }
    else{
        for (size_t i = 0; i < pointsEdgeLeft.size() && i < pointsEdgeRight.size(); ++i) {
            // 计算左右车道线中点
            int mid_x = (pointsEdgeLeft[i].x + pointsEdgeRight[i].x) / 2;
            
            // 向右偏移定值
            int shifted_x = mid_x + LAYBY_SHIFT_RIGHT;
            shifted_x = std::min(shifted_x, COLSIMAGE-1); // 确保不超出右边界
            
            // 保持原有y坐标
            AI_CenterEdge.push_back({shifted_x, pointsEdgeRight[i].y});
        }
    }
    if (!control_points.empty()) {
        // 转换控制点类型为POINT
        vector<POINT> control_points_p;
        for (const auto& cp : control_points) {
            control_points_p.push_back({cp.x, cp.y});
        }
        vector<POINT> bezierPoints = general.Bezier(0.01, control_points_p);
        AI_CenterEdge.insert(AI_CenterEdge.end(), bezierPoints.begin(), bezierPoints.end());
            
    }
    if (!AI_CenterEdge.empty()) {
        std::vector<cv::Point> cv_points;
        for (const auto& p : AI_CenterEdge) {
            // 确保坐标不超出图像范围
            int x = std::max(0, std::min(p.x, src_img.cols - 1));
            int y = std::max(0, std::min(p.y, src_img.rows - 1));
            cv_points.emplace_back(x, y);
        }
        // cv::polylines(src_img, cv_points, false, cv::Scalar(0, 0, 250), 2); 
    }
    //状态给定
    //识别虚线停车
    //  cv::Mat gray, binary;
    //  cv::cvtColor(src_img, gray, cv::COLOR_BGR2GRAY);
    //  cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY);
     
    //     cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8UC1);
    //     std::vector<cv::Point> polygon_points;

    //     // 构建多边形点集（左车道逆序+右车道正序
    //     for(int i = pointsEdgeLeft.size()-1; i >= 0; --i) {
    //         polygon_points.emplace_back(pointsEdgeLeft[i].x, pointsEdgeLeft[i].y);
    //     }
    //     for(const auto& p : pointsEdgeRight) {
    //         polygon_points.emplace_back(p.x, p.y);
    //     }
    //     // 填充多边形区域
    //     std::vector<std::vector<cv::Point>> contours = {polygon_points};
    //     cv::fillPoly(mask, contours, cv::Scalar(255));

    //     // 应用掩膜
    //     cv::bitwise_and(binary, mask, binary);
    //     // cv::imshow("Masked Binary", binary);  
    //  // 状态转换逻辑
    //      // 1. 对处理后的binary图像进行Canny边缘检测
    //     cv::Mat edges;
    //     cv::Canny(binary, edges, 50, 150);

    //     // 2. 查找轮廓（检测黑色小色块）
    //     // std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //     // 3. 检测有效黑色小色块
    //     bool black_blobs_detected = false;
    //     int blob_count = 0;
    //     const int MIN_BLOB_AREA = 10;   // 最小色块面积
    //     const int MAX_BLOB_AREA = 50;  // 最大色块面积

    //     for (const auto& contour : contours) {
    //         // 计算轮廓面积
    //         double area = cv::contourArea(contour);
    //         if (area < MIN_BLOB_AREA || area > MAX_BLOB_AREA) continue;
            
    //         // 获取边界框
    //         cv::Rect bbox = cv::boundingRect(contour);
    //         if (bbox.y < 80) {  // 行数小于100则跳过
    //             continue;
    //         }
            
    //         // 在原始图像上绘制边界框（绿色）
    //         cv::rectangle(src_img, bbox, cv::Scalar(0, 255, 0), 2);
            
    //         // 标记为有效黑色色块
    //         black_blobs_detected = true;
    //         blob_count++;
            
    //         // 在色块中心标注面积
    //         cv::Point center(bbox.x + bbox.width/2, bbox.y + bbox.height/2);
    //         cv::putText(src_img, std::to_string((int)area), center, 
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
    //     }
        
        static int no_blob_count = 0;  // 连续未检测到色块的帧数

    //     if (black_blobs_detected) {
    //         no_blob_count = 0;  // 重置计数器
    //         cv::putText(src_img, "Blobs: " + std::to_string(blob_count), 
    //                 cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
    //                 cv::Scalar(0, 255, 0), 2);
    //     } else {
    //         no_blob_count++;  // 增加计数器
    //         cv::putText(src_img, "No Blobs: " + std::to_string(no_blob_count), 
    //                 cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
    //                 cv::Scalar(0, 0, 255), 2);
    //     }

        // 5. 连续3帧没有检测到黑色色块时进入停车状态
        if(flag_near == NEAR_LEFT)
        {
            if (!current_frame_detected) {
                no_blob_count++;
                if(no_blob_count >= 5)
                {
                    printf("parkinhg!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                    flag_layby = LAYBY_PARKING;
                }
            }
        }
        else{
            if (!current_frame_detected) {
                no_blob_count++;
                if(no_blob_count >= 5)
                {
                    printf("parkinhg!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                    flag_layby = LAYBY_PARKING;
                }
            }
        }
        }

        // 6. 可视化调试
        // cv::imshow("Canny Edges", edges);

    //  if(parkingConfirmCount >= 20) {
    //         flag_layby = LAYBY_PARKING;
    //         // parkingConfirmCount = 0;
    // }
    //  lastHorizontalLineCount = horizontalLineCount;
    //  // 在图像上显示检测信息
    //  cv::putText(src_img, "Lines: " + std::to_string(horizontalLineCount), 
    //             cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
    //             cv::Scalar(0,0,255), 2);
     
 
//  if(flag_layby == LAYBY_PARKING && park_timer >= 100)
//  {
//     flag_layby = LAYBY_NONE;
//  }

case LAYBY_PARKING:{
    parkingConfirmCount++;
        
        if (parkingConfirmCount >= LAYBY_PARKING_TIMER) {  
            flag_layby = LAYBY_NONE;
            no_detect_count = 0;
            parkingConfirmCount = 0;
        }
}
    }
    // //速度模块
    // if(flag_layby == LAYBY_ENTER)
    // {
    //     output_speed_rato = 0.55;
    // }
    // else if(flag_layby == LAYBY_PARKING)
    // {
    //     output_speed_rato = 0;
    // }
}