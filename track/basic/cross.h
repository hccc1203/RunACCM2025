/*************************************/
// Author 胡城玮
/*************************************/


#pragma once

#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../imgprocess/imgprocess.h"
#include "../standard/general.h"

#define cross_debug 0

class Cross
{
public:
    General general;
    Cross();
    Cross(Config _config);

    enum flag_Cross_e
    {
        Cross_None,
        Cross_Begin,
        Cross_Out
    };
    flag_Cross_e flag_cross;

    enum state_Cross_e
    {
        Cross_Left, // 从左入
        Cross_Right, // 从右入
        Cross_Both // 正入
    };
    state_Cross_e state_cross;

    void Cross_Check(bool is_L_left_found, bool is_L_right_found, cv::Mat img, Point t_L_pointLeft, Point t_L_pointRight,int t_L_pointLeft_id, int t_L_pointRight_id,int t_pointsEdgeLeft_size,int t_pointsEdgeRight_size);
    void Cross_Run(std::vector<POINT> &t_pointsEdgeLeft, std::vector<POINT> &t_pointsEdgeRight, cv::Mat img,
                    bool is_L_left_found, bool is_L_right_found, int t_L_pointLeft_id, int t_L_pointRight_id,
                    int &t_pointsEdgeLeft_size,int &t_pointsEdgeRight_size);

    int Cross_counter = 0;

    vector<POINT> far_pointsEdgeLeft;   // 赛道左边缘点集
    vector<POINT> far_pointsEdgeRight;  // 赛道右边缘点集
    vector<POINT> far_t_pointsEdgeLeft; // 透视变换后点集
    vector<POINT> far_t_pointsEdgeRight;
    vector<POINT> far_b_t_pointsEdgeLeft; // 滤波后点集
    vector<POINT> far_b_t_pointsEdgeRight;
    vector<POINT> far_s_b_t_pointsEdgeLeft; // 等距采样后点集
    vector<POINT> far_s_b_t_pointsEdgeRight;
    vector<POINT> far_a_t_pointsEdgeLeft; // 计算角度后点集
    vector<POINT> far_a_t_pointsEdgeRight;
    vector<POINT> far_n_a_t_pointsEdgeLeft; // 非极大值抑制
    vector<POINT> far_n_a_t_pointsEdgeRight;

    int far_pointsEdgeLeft_size;
    int far_pointsEdgeRight_size;
    int far_t_pointsEdgeLeft_size;
    int far_t_pointsEdgeRight_size;
    int far_b_t_pointsEdgeLeft_size; // 滤波后点集
    int far_b_t_pointsEdgeRight_size;
    int far_s_b_t_pointsEdgeLeft_size; // 等距采样后点集
    int far_s_b_t_pointsEdgeRight_size;
    int far_a_t_pointsEdgeLeft_size; // 边线局部角度变化量后点集
    int far_a_t_pointsEdgeRight_size;
    int far_n_a_t_pointsEdgeLeft_size; // 非极大值抑制
    int far_n_a_t_pointsEdgeRight_size;

    int far_t_L_pointLeft_id;
    int far_t_L_pointRight_id;

    int Lconf_Min = 70;
    int Lconf_Max = 130;

    int both_L_find_counter = 0;
    int left_cross_in_counter = 0; 
    int right_cross_in_counter = 0;

    int far_left_x0 = 80;
    int far_left_y0 = 160;
    int far_right_x0 = 240;
    int far_right_y0 = 160;
    int approx_num = 3;
    int block_size = 9; // 自适应阈值block大小
    int clip_value = 3; // 自适应阈值的阈值剪裁量
    double pixel_per_meter = 222.222; // 一米在画面中所占的像素数
    double SAMPLE_DIST = 0.02;        // 采样间距 单位m
    double ROAD_WIDTH = 0.45;         // 道路真实宽度 单位m
    double dist = pixel_per_meter * ROAD_WIDTH / 2.0f;

    bool is_far_t_L_pointLeft_find;
    bool is_far_t_L_pointRight_find;

    bool is_make_line;

    int L_left_down_id;
    int L_left_up_id;
    int L_right_down_id;
    int L_right_up_id;

    int EdgeLeft_size;
    int EdgeRight_size;
    int t_EdgeLeft_size;
    int t_EdgeRight_size;
    int CenterEdge_size;
    
    int no_line_counter = 0;
    int pro_rowCutDown = 180; // 处理时图像下切
    int pro_rowCutUp = 60;    // 处理时图像上切

    bool L_left_found;
    bool L_right_found;

    float Slope_Calculate(int begin, int end, std::vector<POINT> &pointsEdge);

    void calculate_s_i(int start, int end, vector<POINT> &pointsEdge, float &slope_rate, float &intercept);

    void make_up_line();



    const int dir_front[4][2] = {{0, -1},
                                 {1, 0},
                                 {0, 1},
                                 {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1},
                                     {1, -1},
                                     {1, 1},
                                     {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1},
                                      {1, 1},
                                      {-1, 1},
                                      {-1, -1}};

private:
    // 基础巡线
    void cross_find_farline(cv::Mat &img,bool is_L_left_found, bool is_L_right_found, std::vector<POINT> t_pointsEdgeLeft, std::vector<POINT> t_pointsEdgeRight,int t_L_pointLeft_id,int t_L_pointRight_id);

    // 左手巡线
    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeLeft, int &pointsEdgeLeft_size);

    // 右手巡线
    void findline_righthand_adaptive(Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeRight, int &pointsEdgeRight_size);

    // 滤波
    void blur_points(int side, int kernel);

    // 等距采样
    void resample_points(vector<POINT> &in, int in_size, vector<POINT> &out, int &out_size, float dist);

    void find_corners();

    // 计算角度
    void local_angle_points(vector<POINT> pointsEdgeIn, int size, vector<POINT> &pointsEdgeOut, int dist);

    // 非极大值抑制
    void nms_angle(vector<POINT> &in, int in_size, vector<POINT> &out, int kernel);
};