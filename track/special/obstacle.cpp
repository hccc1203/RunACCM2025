
/*************************************/
// Author 孙酩贺
/*************************************/


//////////////////////////////////////////////////////////////////////
/////////贝塞尔算法///////////////////////////////////////////////////
#include "obstacle.h"
#include "../standard/general.h"
#include <algorithm>
// #include <opencv2/highgui.hpp>
// #include <opencv2/opencv.hpp>



#define PREDICT_TIMES 10
#define PREDICT_THRESH 2 

General image_writer;

Obstacle::Obstacle() {}

Obstacle::Obstacle(Config &config)
{
}
static int type4_count = 0;
static std::queue<bool> recent_results;

void Obstacle::check_obstacle(cv::Mat &src_img, vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight, std::vector<PredictResult> &predict_result,bool is_straight_left,bool is_straight_right)
{
                            // 辅助函数：查找指定y坐标的边缘点x值
                            auto interpolate_edge_x = [](const vector<POINT> &edge, int y) -> int {
                                if (edge.empty()) return -1;
    
                                // 寻找目标y附近的上下两个点
                                int above_idx = -1, below_idx = -1;
                                for (size_t i = 0; i < edge.size(); ++i) {
                                    if (edge[i].y == y) return edge[i].x;
                                    else if (edge[i].y < y) { // 下方点
                                        if (below_idx == -1 || edge[i].y > edge[below_idx].y)
                                            below_idx = i;
                                    } else { // 上方点
                                        if (above_idx == -1 || edge[i].y < edge[above_idx].y)
                                            above_idx = i;
                                    }
                                }
                                // 边界处理
                                if (above_idx == -1) return edge[below_idx].x;
                                if (below_idx == -1) return edge[above_idx].x;
    
                                // 线性插值计算x
                                const POINT &a = edge[above_idx];
                                const POINT &b = edge[below_idx];
                                double t = (double)(y - b.y) / (a.y - b.y);
                                return (int)(b.x + t * (a.x - b.x));
                            };
 
    if (pointsEdgeLeft.empty() || pointsEdgeRight.empty())
    {
        // 若赛道边缘点不存在，直接返回
        return;
    }


   
    // 初始化标志和最低位置
    bool has_cone = false;
    bool has_pedestrian = false;
    bool has_black_brick = false;
    int lowest_cone_y = -1;      // 锥桶最低点的y坐标（最大值）
    int lowest_pedestrian_y = -1; // 行人最低点的y坐标（最大值）
    int lowest_black_brick_y = -1;
    int min_area = 50;

    // 遍历所有检测结果
    for (auto &res : predict_result)
    {
        if (res.type == 5)
        {
            // 计算锥桶底部y坐标（矩形框底部 = y + height）
            int bottom_y = res.y + res.height;
            if (bottom_y > lowest_cone_y) {
                lowest_cone_y = bottom_y;
            }
            if(res.x <  interpolate_edge_x(pointsEdgeLeft, res.y)|| res.x > interpolate_edge_x(pointsEdgeRight, res.x))
            {
                has_cone = false;
            }
            else
            {
                has_cone = true;
            }
        }
        if (res.type == 7)
        {
            // if(res.x <  interpolate_edge_x(pointsEdgeLeft, res.y)|| res.x > interpolate_edge_x(pointsEdgeRight, res.x))
            // {
            //     has_pedestrian = false;
            // }
            // else{
            //     has_pedestrian = true;
            // }
            has_pedestrian = true;
            // 计算行人底部y坐标
            int bottom_y = res.y + res.height;
            if (bottom_y > lowest_pedestrian_y) {
                lowest_pedestrian_y = bottom_y;
            }
        }
        else if (res.type == 1)
        {
            if(res.x <  interpolate_edge_x(pointsEdgeLeft, res.y)|| res.x > interpolate_edge_x(pointsEdgeRight, res.x))
            {
                has_black_brick = false;
            }
            else{
                has_black_brick = true;
            }
            has_black_brick = true;
            int bottom_y = res.y + res.height;
            if (bottom_y > lowest_black_brick_y) {
                lowest_black_brick_y = bottom_y;
            }
        }
    }
 
    // 决策逻辑：优先选择图像下方（y坐标更大）的物体
    if (has_cone && has_pedestrian)
    {
        if (lowest_cone_y >= lowest_pedestrian_y) 
        {
            printf("识别为锥桶区（更靠近底部）");
            flag_obstacle = OBSTACLE_CONICAL;
        }
        else
        {
            printf("识别为行人区（更靠近底部）");
            flag_obstacle = OBSTACLE_PEDESTRAIN;
        }
        
    }
    if (has_cone && has_black_brick)
    {
        if (lowest_cone_y >= lowest_black_brick_y) 
        {
            printf("识别为锥桶区（更靠近底部）");
            flag_obstacle = OBSTACLE_CONICAL;
        }
        else
        {
            printf("识别为黑色障碍区（更靠近底部）");
            flag_obstacle = OBSTACLE_BLACK_BRICK;
        }
        
    }
    if (has_pedestrian && has_black_brick)
    {
        if (lowest_pedestrian_y >= lowest_black_brick_y) 
        {
            printf("识别为行人区（更靠近底部）");
            flag_obstacle = OBSTACLE_PEDESTRAIN;
        }
        else
        {
            printf("识别为黑色障碍区（更靠近底部）");
            flag_obstacle = OBSTACLE_BLACK_BRICK;
        }
        
    }
    else if (has_cone)
    {
        printf("识别为锥桶区");
        flag_obstacle = OBSTACLE_CONICAL;
    }
    else if (has_pedestrian)
    {
        printf("识别为行人区");
        flag_obstacle = OBSTACLE_PEDESTRAIN;
    }
    else if(has_black_brick)
    {
        printf("识别为黑块区");
        flag_obstacle = OBSTACLE_BLACK_BRICK;
    }
   

}

void Obstacle::run_obstacle(
    cv::Mat &src_img,
    vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight,
    vector<POINT> &AI_CenterEdge,
    int pointsEdgeRight_size, int pointEdgeLeft_size,std::vector<PredictResult> &predict_result,cv::Mat imgBinary,std::vector<POINT> &t_pointsEdgeLeft,int &t_pointsEdgeLeft_size,
    std::vector<POINT> &t_pointsEdgeRight,int &t_pointsEdgeRight_size,bool is_straight_left,bool is_straight_right)
{
    flag_obstacle = OBSTACLE_NONE;
    check_obstacle(src_img,pointsEdgeLeft,pointsEdgeRight, predict_result,is_straight_left,is_straight_right);
    switch (flag_obstacle)
    {
    case OBSTACLE_CONICAL:
    {
        printf("左边线长度为%d,右边线长度为%d", pointEdgeLeft_size, pointsEdgeRight_size);

        
                    if (pointsEdgeRight_size != 0 && pointEdgeLeft_size != 0)
                {
                    // 1. 从模型预测结果中获取锥桶底部位置
                    vector<cv::Point> obstacle_bottoms;
                    for (auto &res : predict_result) {
                        if (res.type == 5 || res.type == 7 || res.type == 1) {
                            // 计算锥桶底部中心点
                            int x_bottom = res.x;
                            int y_bottom = res.y + res.height;
                            obstacle_bottoms.emplace_back(x_bottom, y_bottom);
                        }
                    }
                    // 1. 重新检测黄色障碍物最低点
                // vector<cv::Point> obstacle_bottoms;
                // {
                //     // Mat hsv, mask;
                //     // cvtColor(src_img, hsv, COLOR_BGR2HSV);
                //     // inRange(hsv, Scalar(40, 100, 100), Scalar(60, 255, 255), mask);
                //     // Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
                //     // morphologyEx(mask, mask, MORPH_OPEN, kernel);
                //     // morphologyEx(mask, mask, MORPH_CLOSE, kernel);
        
                //     // vector<vector<Point>> contours;
                //     // findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
                //     // for (const auto& contour : contours) {
                //     //     if (contourArea(contour) > 100) {
                //     //         Moments m = moments(contour);
                //     //         obstacle_bottoms.emplace_back(m.m10/m.m00, m.m01/m.m00);
                //     //     }
                //     // }
                //     int min_area = 50;
                //     Mat hsv, mask;
                //     cvtColor(src_img, hsv, COLOR_BGR2HSV);
                //     inRange(hsv, Scalar(40, 100, 100), Scalar(60, 255, 255), mask);
                    
                //     Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
                //     morphologyEx(mask, mask, MORPH_OPEN, kernel);
                //     morphologyEx(mask, mask, MORPH_CLOSE, kernel);
                    
                //     vector<vector<Point>> contours;
                //     findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                    
                //     for (const auto& contour : contours) {
                //         if (contourArea(contour) > min_area) {
                //             // 找到轮廓的最低点（y坐标最大）
                //             auto bottom_it = std::max_element(contour.begin(), contour.end(),
                //                 [](const Point& a, const Point& b) { return a.y < b.y; });
                            
                //             // 计算底部中心点（取底部附近多个点的x平均值）
                //             int sum_x = 0, count = 0;
                //             for (const auto& pt : contour) {
                //                 if (abs(pt.y - bottom_it->y) <= 5) { // 底部5像素范围内
                //                     sum_x += pt.x;
                //                     count++;
                //                 }
                //             }
                //             if (count > 0) {
                //                 obstacle_bottoms.emplace_back(sum_x/count, bottom_it->y);
                //             }
                //         }
                //     }
                // }
                //  // 2. 生成贝塞尔曲线控制点
                //  vector<Point> control_points;
                //  if(pointsEdgeLeft.empty() || pointsEdgeRight.empty())
                //  {
                //      flag_obstacle = OBSTACLE_NONE;
                //  }
                 
                //  if (!obstacle_bottoms.empty()) 
                //  {
                //      // 辅助函数：查找指定y坐标的边缘点x值
                //      auto interpolate_edge_x = [](const vector<POINT>& edge, int y) -> int {
                //          if (edge.empty()) return -1;
                         
                //          // 寻找目标y附近的上下两个点
                //          int above_idx = -1, below_idx = -1;
                //          for (size_t i = 0; i < edge.size(); ++i) {
                //              if (edge[i].y == y) return edge[i].x;
                //              else if (edge[i].y < y) { // 下方点
                //                  if (below_idx == -1 || edge[i].y > edge[below_idx].y) 
                //                      below_idx = i;
                //              } else { // 上方点
                //                  if (above_idx == -1 || edge[i].y < edge[above_idx].y) 
                //                      above_idx = i;
                //              }
                //          }                 
                //          // 边界处理
                //          if (above_idx == -1) return edge[below_idx].x;
                //          if (below_idx == -1) return edge[above_idx].x;
                         
                //          // 线性插值计算x
                //          const POINT& a = edge[above_idx];
                //          const POINT& b = edge[below_idx];
                //          double t = (double)(y - b.y) / (a.y - b.y);
                //          return (int)(b.x + t * (a.x - b.x));
                //      };
         
                //      // if (obstacle_bottoms.size() >= 2) {
                //          //图像底部的边缘线的中点
                //          int bottom_left_x = interpolate_edge_x(pointsEdgeLeft, src_img.rows - 1);
                //          int bottom_right_x = interpolate_edge_x(pointsEdgeRight, src_img.rows - 1);
                //          int bottom_mid_x = (bottom_left_x + bottom_right_x) / 2;
                //          //图像顶部边缘线的中点
                //          int top_left_x = interpolate_edge_x(pointsEdgeLeft,0);
                //          int top_right_x = interpolate_edge_x(pointsEdgeRight,0);
                //          int top_mid_x = (top_left_x + top_right_x) / 2;
                //          // 添加图像底部边缘线中心作为起点
                //          control_points.emplace_back(bottom_mid_x, src_img.rows - 1);
 
                //          // // 按x坐标排序区分左右
                //          // sort(obstacle_bottoms.begin(), obstacle_bottoms.end(),
                //          //      [](const Point& a, const Point& b) { return a.x < b.x; });
         
                //          // // 左锥桶与右边缘线中点
                //          // Point left_cone = obstacle_bottoms[0];
                //          // int right_x = interpolate_edge_x(pointsEdgeRight, left_cone.y);
                //          // control_points.emplace_back((left_cone.x + right_x)/2+width_conical, left_cone.y);
         
                //          // // 右锥桶与左边缘线中点
                //          // Point right_cone = obstacle_bottoms[1];
                //          // int left_x = interpolate_edge_x(pointsEdgeLeft, right_cone.y);
                //          // control_points.emplace_back((right_cone.x + left_x)/2-width_conical, right_cone.y);
         
                //          // // 较下方的锥桶与底部中点
                //          // Point lower_cone = (left_cone.y > right_cone.y) ? left_cone : right_cone;
                //          // control_points.emplace_back(lower_cone.x, (lower_cone.y + src_img.rows)/2);
                //          sort(obstacle_bottoms.begin(), obstacle_bottoms.end(),
                //              [](const Point& a, const Point& b) { return a.y > b.y; });
                //          for(size_t i = 0; i < obstacle_bottoms.size(); ++i)
                //          {
                //              if(obstacle_bottoms[i].x < (interpolate_edge_x(pointsEdgeLeft,obstacle_bottoms[i].y)+interpolate_edge_x(pointsEdgeRight,obstacle_bottoms[i].y))/2)
                //              {//在左侧
                //                  control_points.emplace_back((obstacle_bottoms[i].x+interpolate_edge_x(pointsEdgeRight,obstacle_bottoms[i].y))/2 + OBSTACLE_width_conical,obstacle_bottoms[i].y);
                //                  control_points.emplace_back(top_mid_x - OBSTACLE_width_conical, 0);
                //              }
                //              if(obstacle_bottoms[i].x > (interpolate_edge_x(pointsEdgeLeft,obstacle_bottoms[i].y)+interpolate_edge_x(pointsEdgeRight,obstacle_bottoms[i].y))/2)
                //              {//在右侧
                //                  control_points.emplace_back((obstacle_bottoms[i].x+interpolate_edge_x(pointsEdgeLeft,obstacle_bottoms[i].y))/2 - OBSTACLE_width_conical,obstacle_bottoms[i].y);
                //                  control_points.emplace_back(top_mid_x + OBSTACLE_width_conical, 0);
                //              }
                //          // }
                //          //添加图像顶部赛道边缘线中点
                     
                         
                //      } 
                //      // else { // 单个障碍物
                //      //     //图像顶部赛道线中点
                //      //     int top_left_x = interpolate_edge_x(pointsEdgeLeft,0);
                //      //     int top_right_x = interpolate_edge_x(pointsEdgeRight,0);
                //      //     int top_mid_x = (top_left_x + top_right_x) / 2;
                //      //     //图像底部赛道线中点
                //      //     int bottom_left_x = interpolate_edge_x(pointsEdgeLeft, src_img.rows - 1);
                //      //     int bottom_right_x = interpolate_edge_x(pointsEdgeRight, src_img.rows - 1);
                //      //     int bottom_mid_x = (bottom_left_x + bottom_right_x) / 2;
                //      //     control_points.emplace_back(bottom_mid_x, src_img.rows - 1);
                //      //     Point cone = obstacle_bottoms[0];
                //      //     if (cone.x < (interpolate_edge_x(pointsEdgeLeft,cone.y)+interpolate_edge_x(pointsEdgeRight,cone.y))/2) { // 左侧
                //      //         int right_x = interpolate_edge_x(pointsEdgeRight, cone.y);
                //      //         control_points.emplace_back((cone.x + right_x)/2 + width_conical, cone.y);
                //      //         // control_points.emplace_back((interpolate_edge_x(pointsEdgeLeft,(cone.y + src_img.rows)/2)+interpolate_edge_x(pointsEdgeRight,(cone.y + src_img.rows)/2)), (cone.y + src_img.rows)/2);
                //      //     } else if (cone.x > (interpolate_edge_x(pointsEdgeLeft,cone.y)+interpolate_edge_x(pointsEdgeRight,cone.y))/2){ // 右侧
                //      //         int left_x = interpolate_edge_x(pointsEdgeLeft, cone.y);
                //      //         control_points.emplace_back((cone.x + left_x)/2 - width_conical, cone.y);
                //      //         // control_points.emplace_back((interpolate_edge_x(pointsEdgeLeft,(cone.y + src_img.rows)/2)+interpolate_edge_x(pointsEdgeRight,(cone.y + src_img.rows)/2)), (cone.y + src_img.rows)/2);
                //      //     }
                //      //     else{//中部统一规定往右打
                //      //         int right_x = interpolate_edge_x(pointsEdgeRight, cone.y);
                //      //         control_points.emplace_back((cone.x + right_x)/2, cone.y);//还有改进空间（后续处理）
                //      //     }
 
                //      //     control_points.emplace_back(top_mid_x, 0);
                //      //     // // 补充终点
                //      //     // control_points.emplace_back(img_center_x, 0);
                //      // }
         
                //      // 3. 生成三阶贝塞尔曲线
                //      AI_CenterEdge.clear();
                //      if (!control_points.empty()) {
                //          // 转换控制点类型为POINT
                //          vector<POINT> control_points_p;
                //          for (const auto& cp : control_points) {
                //              control_points_p.push_back({cp.x, cp.y});
                //          }
                //          vector<POINT> bezierPoints = general.Bezier(0.01, control_points_p);
                //          AI_CenterEdge.insert(AI_CenterEdge.end(), bezierPoints.begin(), bezierPoints.end());
                             
                //      }
                //      if (!AI_CenterEdge.empty()) {
                //          std::vector<cv::Point> cv_points;
                //          for (const auto& p : AI_CenterEdge) {
                //              // 确保坐标不超出图像范围
                //              int x = std::max(0, std::min(p.x, src_img.cols - 1));
                //              int y = std::max(0, std::min(p.y, src_img.rows - 1));
                //              cv_points.emplace_back(x, y);
                //          }
                //          cv::polylines(src_img, cv_points, false, cv::Scalar(0, 0, 250), 2); 
                //      }
                //  }

                    // 2. 生成贝塞尔曲线控制点
                    vector<Point> control_points;
                    if (pointsEdgeLeft.empty() || pointsEdgeRight.empty()) {
                        flag_obstacle = OBSTACLE_NONE;
                        break;
                    }

                    if (!obstacle_bottoms.empty()) {
                        // 辅助函数：查找指定y坐标的边缘点x值
                        auto interpolate_edge_x = [](const vector<POINT> &edge, int y) -> int {
                            if (edge.empty()) return -1;

                            // 寻找目标y附近的上下两个点
                            int above_idx = -1, below_idx = -1;
                            for (size_t i = 0; i < edge.size(); ++i) {
                                if (edge[i].y == y) return edge[i].x;
                                else if (edge[i].y < y) { // 下方点
                                    if (below_idx == -1 || edge[i].y > edge[below_idx].y)
                                        below_idx = i;
                                } else { // 上方点
                                    if (above_idx == -1 || edge[i].y < edge[above_idx].y)
                                        above_idx = i;
                                }
                            }
                            // 边界处理
                            if (above_idx == -1) return edge[below_idx].x;
                            if (below_idx == -1) return edge[above_idx].x;

                            // 线性插值计算x
                            const POINT &a = edge[above_idx];
                            const POINT &b = edge[below_idx];
                            double t = (double)(y - b.y) / (a.y - b.y);
                            return (int)(b.x + t * (a.x - b.x));
                        };

                        // 图像底部的边缘线的中点
                        int bottom_left_x = interpolate_edge_x(pointsEdgeLeft, src_img.rows - 1);
                        int bottom_right_x = interpolate_edge_x(pointsEdgeRight, src_img.rows - 1);
                        int bottom_mid_x = (bottom_left_x + bottom_right_x) / 2;
                        
                        // 图像顶部边缘线的中点
                        int top_left_x = interpolate_edge_x(pointsEdgeLeft, 0);
                        int top_right_x = interpolate_edge_x(pointsEdgeRight, 0);
                        int top_mid_x = (top_left_x + top_right_x) / 2;
                        
                        // 添加图像底部边缘线中心作为起点
                        control_points.emplace_back(bottom_mid_x, src_img.rows - 1);

                        // 按y坐标排序（从下到上）
                        sort(obstacle_bottoms.begin(), obstacle_bottoms.end(),
                            [](const Point &a, const Point &b) { return a.y > b.y; });

                        // 为每个锥桶生成控制点
                        for (size_t i = 0; i < obstacle_bottoms.size(); ++i) {
                            int cone_x = obstacle_bottoms[i].x;
                            int cone_y = obstacle_bottoms[i].y;
                            
                            // 计算锥桶所在位置的赛道中线
                            int midline_x = (interpolate_edge_x(pointsEdgeLeft, cone_y) + 
                                            interpolate_edge_x(pointsEdgeRight, cone_y)) / 2;
                            
                            // 确定锥桶位置（左/右/中）
                            if (cone_x < midline_x - 20) { // 左侧锥桶
                                int right_x = interpolate_edge_x(pointsEdgeRight, cone_y);
                                control_points.emplace_back((cone_x + right_x) / 2 + OBSTACLE_width_conical, cone_y);
                            } 
                            else if (cone_x > midline_x + 20) { // 右侧锥桶
                                int left_x = interpolate_edge_x(pointsEdgeLeft, cone_y);
                                control_points.emplace_back((cone_x + left_x) / 2 - OBSTACLE_width_conical - 40, cone_y);
                            } 
                            else { // 中部锥桶（默认向右绕行）
                                int right_x = interpolate_edge_x(pointsEdgeRight, cone_y);
                                control_points.emplace_back((cone_x + right_x) / 2, cone_y);
                            }
                        }
                        
                        // 添加图像顶部赛道边缘线中点作为终点
                        control_points.emplace_back(top_mid_x, 0);
                    }

                    // 3. 生成三阶贝塞尔曲线
                    AI_CenterEdge.clear();
                    if (!control_points.empty()) {
                        // 转换控制点类型为POINT
                        vector<POINT> control_points_p;
                        for (const auto &cp : control_points) {
                            control_points_p.push_back({cp.x, cp.y});
                        }
                        vector<POINT> bezierPoints = general.Bezier(0.01, control_points_p);
                        AI_CenterEdge.insert(AI_CenterEdge.end(), bezierPoints.begin(), bezierPoints.end());
                    }
                    
                    // 4. 绘制生成的路径
                    if (!AI_CenterEdge.empty()) {
                        std::vector<cv::Point> cv_points;
                        for (const auto &p : AI_CenterEdge) {
                            // 确保坐标不超出图像范围
                            int x = std::max(0, std::min(p.x, src_img.cols - 1));
                            int y = std::max(0, std::min(p.y, src_img.rows - 1));
                            cv_points.emplace_back(x, y);
                        }
                        cv::polylines(src_img, cv_points, false, cv::Scalar(0, 0, 250), 2);
                    }
                }
                
                // 5. 检查本次处理是否实际检测到锥桶
                bool cone_detected = false;
                for (auto res : predict_result) {
                    if (res.type == 5) {
                        cone_detected = true;
                        break;
                    }
                }
                
                // 如果没有检测到锥桶，则重置标志
                if (!cone_detected) {
                    flag_obstacle = OBSTACLE_NONE;
                }
                
                break;
                
            
    }

    case OBSTACLE_PEDESTRAIN:
    {

        // 2. 生成贝塞尔控制点
        vector<Point> control_points;
        // 辅助函数：查找指定y坐标的边缘点x值
        auto interpolate_edge_x = [](const vector<POINT> &edge, int y) -> int
        {
            if (edge.empty())
                return -1;

            // 寻找目标y附近的上下两个点
            int above_idx = -1, below_idx = -1;
            for (size_t i = 0; i < edge.size(); ++i)
            {
                if (edge[i].y == y)
                    return edge[i].x;
                else if (edge[i].y < y)
                { // 下方点
                    if (below_idx == -1 || edge[i].y > edge[below_idx].y)
                        below_idx = i;
                }
                else
                { // 上方点
                    if (above_idx == -1 || edge[i].y < edge[above_idx].y)
                        above_idx = i;
                }
            }
            // 边界处理
            if (above_idx == -1)
                return edge[below_idx].x;
            if (below_idx == -1)
                return edge[above_idx].x;

            // 线性插值计算x
            const POINT &a = edge[above_idx];
            const POINT &b = edge[below_idx];
            double t = (double)(y - b.y) / (a.y - b.y);
            return (int)(b.x + t * (a.x - b.x));
        };
        // 图像底部的边缘线的中点
        int bottom_left_x = interpolate_edge_x(pointsEdgeLeft, src_img.rows - 1);
        int bottom_right_x = interpolate_edge_x(pointsEdgeRight, src_img.rows - 1);
        int bottom_mid_x = (bottom_left_x + bottom_right_x) / 2;
        // 图像顶部边缘线的中点
        int top_left_x = interpolate_edge_x(pointsEdgeLeft, 0);
        int top_right_x = interpolate_edge_x(pointsEdgeRight, 0);
        int top_mid_x = (top_left_x + top_right_x) / 2;
        // 添加图像底部边缘线中心作为起点
        control_points.emplace_back(bottom_mid_x, src_img.rows - 1);
        // 遍历行人检测结果进行路径规划
        for (const auto &res : predict_result)
        {
            if (res.type == 7 || res.type == 5 || res.type == 1)
            {
                // 行人底部y坐标
                int pedestrian_bottom_y = res.y + res.height;
                // 行人在图像中的中心x坐标
                int pedestrian_center_x = res.x + res.width / 2;
                // 行人所在位置的赛道中线x坐标
                int midline_x = (interpolate_edge_x(pointsEdgeLeft, pedestrian_bottom_y) +
                                 interpolate_edge_x(pointsEdgeRight, pedestrian_bottom_y)) /
                                2;

                // 判断行人在赛道中线的左侧还是右侧
                bool is_left_side = pedestrian_center_x < midline_x;
                // 更远边线对应行人底部y坐标的x值
                int far_side_x = interpolate_edge_x(is_left_side ? pointsEdgeRight : pointsEdgeLeft, pedestrian_bottom_y);
                // 行人与更远边线底部混合点x坐标
                int bottom_mixed_x = (pedestrian_center_x + far_side_x) / 2;
                // 更远边线对应行人顶部y坐标（假设行人高度一半为顶部）的x值
                int far_top_x = interpolate_edge_x(is_left_side ? pointsEdgeRight : pointsEdgeLeft, res.y + res.height / 2);
                // 行人与更远边线顶部混合点x坐标
                int top_mixed_x = (pedestrian_center_x + far_top_x) / 2;

                if (is_left_side)
                {
                    control_points.emplace_back(bottom_mixed_x + OBSTACLE_width_pedestrian + 40 , pedestrian_bottom_y);
                    control_points.emplace_back(top_mixed_x + OBSTACLE_width_pedestrian + 40, res.y + res.height / 2);
                    control_points.emplace_back(top_mid_x - OBSTACLE_width_pedestrian, 0);
                }
                else
                {
                    control_points.emplace_back(bottom_mixed_x - OBSTACLE_width_pedestrian, pedestrian_bottom_y);
                    control_points.emplace_back(top_mixed_x - OBSTACLE_width_pedestrian, res.y + res.height / 2);
                    control_points.emplace_back(top_mid_x + OBSTACLE_width_pedestrian, 0);
                }
            }
        }

        // 3. 生成贝塞尔曲线
        AI_CenterEdge.clear();
        if (control_points.size() >= 1)
        {
            vector<POINT> control_points_p;
            for (const auto &cp : control_points)
            {
                control_points_p.push_back({cp.x, cp.y});
            }
            vector<POINT> bezierPoints = general.Bezier(0.01, control_points_p);
            AI_CenterEdge.insert(AI_CenterEdge.end(), bezierPoints.begin(), bezierPoints.end());

            // 绘制路径
            if (!AI_CenterEdge.empty())
            {
                std::vector<cv::Point> cv_points;
                for (const auto &p : AI_CenterEdge)
                {
                    // 确保坐标不超出图像范围
                    int x = std::max(0, std::min(p.x, src_img.cols - 1));
                    int y = std::max(0, std::min(p.y, src_img.rows - 1));
                    cv_points.emplace_back(x, y);
                }
                cv::polylines(src_img, cv_points, false, cv::Scalar(0, 0, 250), 2);
            }
        }
        static bool change_pedestrian_enable = true;
         for (auto res : predict_result)
        {
            if (res.type == 7)
            {
                change_pedestrian_enable = false;
            }
        }
        if(change_pedestrian_enable)
        {
            flag_obstacle = OBSTACLE_NONE;
        }
    }
    case OBSTACLE_BLACK_BRICK:
    {
        // 2. 生成贝塞尔控制点
        vector<Point> control_points;
        // 辅助函数：查找指定y坐标的边缘点x值
        auto interpolate_edge_x = [](const vector<POINT> &edge, int y) -> int
        {
            if (edge.empty())
                return -1;

            // 寻找目标y附近的上下两个点
            int above_idx = -1, below_idx = -1;
            for (size_t i = 0; i < edge.size(); ++i)
            {
                if (edge[i].y == y)
                    return edge[i].x;
                else if (edge[i].y < y)
                { // 下方点
                    if (below_idx == -1 || edge[i].y > edge[below_idx].y)
                        below_idx = i;
                }
                else
                { // 上方点
                    if (above_idx == -1 || edge[i].y < edge[above_idx].y)
                        above_idx = i;
                }
            }
            // 边界处理
            if (above_idx == -1)
                return edge[below_idx].x;
            if (below_idx == -1)
                return edge[above_idx].x;

            // 线性插值计算x
            const POINT &a = edge[above_idx];
            const POINT &b = edge[below_idx];
            double t = (double)(y - b.y) / (a.y - b.y);
            return (int)(b.x + t * (a.x - b.x));
        };
        // 图像底部的边缘线的中点
        int bottom_left_x = interpolate_edge_x(pointsEdgeLeft, src_img.rows - 1);
        int bottom_right_x = interpolate_edge_x(pointsEdgeRight, src_img.rows - 1);
        int bottom_mid_x = (bottom_left_x + bottom_right_x) / 2;
        // 图像顶部边缘线的中点
        int top_left_x = interpolate_edge_x(pointsEdgeLeft, 0);
        int top_right_x = interpolate_edge_x(pointsEdgeRight, 0);
        int top_mid_x = (top_left_x + top_right_x) / 2;
        // 添加图像底部边缘线中心作为起点
        control_points.emplace_back(bottom_mid_x, src_img.rows - 1);
        // 遍历行人检测结果进行路径规划
        for (const auto &res : predict_result)
        {
            if (res.type == 5 || res.type == 7 || res.type == 1)
            {
                // 行人底部y坐标
                int block_bottom_y = res.y + res.height;
                // 行人在图像中的中心x坐标
                int block_center_x = res.x + res.width / 2;
                // 行人所在位置的赛道中线x坐标
                int midline_x = (interpolate_edge_x(pointsEdgeLeft, block_bottom_y) +
                                 interpolate_edge_x(pointsEdgeRight, block_bottom_y)) /
                                2;

                // 判断行人在赛道中线的左侧还是右侧
                bool is_left_side = block_center_x < midline_x;
                // 更远边线对应行人底部y坐标的x值
                int far_side_x = interpolate_edge_x(is_left_side ? pointsEdgeRight : pointsEdgeLeft, block_bottom_y);
                // 行人与更远边线底部混合点x坐标
                int bottom_mixed_x = (block_center_x + far_side_x) / 2;
                // 更远边线对应行人顶部y坐标（假设行人高度一半为顶部）的x值
                int far_top_x = interpolate_edge_x(is_left_side ? pointsEdgeRight : pointsEdgeLeft, res.y + res.height / 2);
                // 行人与更远边线顶部混合点x坐标
                int top_mixed_x = (block_center_x + far_top_x) / 2;

                if (is_left_side)
                {
                    control_points.emplace_back(bottom_mixed_x + OBSTACLE_width_block, block_bottom_y);
                    control_points.emplace_back(top_mixed_x + OBSTACLE_width_block, res.y + res.height / 2);
                    control_points.emplace_back(top_mid_x - OBSTACLE_width_block, 0);
                }
                else
                {
                    control_points.emplace_back(bottom_mixed_x - OBSTACLE_width_block, block_bottom_y);
                    control_points.emplace_back(top_mixed_x - OBSTACLE_width_block, res.y + res.height / 2);
                    control_points.emplace_back(top_mid_x + OBSTACLE_width_block, 0);
                }
            }
        }

        // 3. 生成贝塞尔曲线
        AI_CenterEdge.clear();
        if (control_points.size() >= 1)
        {
            vector<POINT> control_points_p;
            for (const auto &cp : control_points)
            {
                control_points_p.push_back({cp.x, cp.y});
            }
            vector<POINT> bezierPoints = general.Bezier(0.01, control_points_p);
            AI_CenterEdge.insert(AI_CenterEdge.end(), bezierPoints.begin(), bezierPoints.end());

            // 绘制路径
            if (!AI_CenterEdge.empty())
            {
                std::vector<cv::Point> cv_points;
                for (const auto &p : AI_CenterEdge)
                {
                    // 确保坐标不超出图像范围
                    int x = std::max(0, std::min(p.x, src_img.cols - 1));
                    int y = std::max(0, std::min(p.y, src_img.rows - 1));
                    cv_points.emplace_back(x, y);
                }
                cv::polylines(src_img, cv_points, false, cv::Scalar(0, 0, 250), 2);
            }
        }
        static bool change_block_enable = true;
         for (auto res : predict_result)
        {
            if (res.type == 1)
            {
                change_block_enable = false;
            }
        }
        for (const auto &result : predict_result) {
            if (result.type == 1 && result.y >= 160) {
                flag_obstacle = OBSTACLE_BLACK_BRICK;
                break;
            }
        }
    }
    }
}

// // 速度 40：
// // p = 1.2
// // k = 1
// // error_far * 0.05
// // error_near * 0.95

// // 速度 50：
// // p = 1.16
// // k = 1
// // error_far * 0.05
// // error_near * 0.95

// // 速度 60：
// // p = 1.2
// // k = 1
// // error_far * 0.05
// // error_near * 0.95

