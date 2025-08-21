/*************************************/
// Author 胡城玮
/*************************************/


#pragma once

#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../imgprocess/imgprocess.h"
#include "../standard/general.h"

class Ring
{
public:
    Ring();
    Ring(Config config);
    General general;

    void reset();

    enum flag_Ring_e {
        Ring_None,
        Left_Ring_pre_Entering,
        Left_Ring_Entering,
        Left_Ring_Inside,
        Left_Ring_Exiting,
        Left_Ring_Finish,
        Right_Ring_pre_Entering,
        Right_Ring_Entering,
        Right_Ring_Inside,
        Right_Ring_Exiting,
        Right_Ring_Finish,
    };

    flag_Ring_e flag_ring;

    void Ring_Check(Mat &imgBinary,bool is_left_straight,bool is_right_straight,bool L_left_found,bool L_right_found,
                    int t_pointsEdgeLeft_size,int t_pointsEdgeRight_size,bool is_L_left_found, bool is_L_right_found, int t_L_pointLeft_id, int t_L_pointRight_id,std::vector<POINT> &t_pointsEdgeLeft,std::vector<POINT> &t_pointsEdgeRight);
    void Ring_Run(std::vector<POINT> &t_pointsEdgeLeft,std::vector<POINT> &t_pointsEdgeRight,
                    int &t_pointsEdgeLeft_size,int &t_pointsEdgeRight_size,
                    bool is_L_left_found, bool is_L_right_found,
                    int t_L_pointLeft_id, int t_L_pointRight_id,
                    cv::Mat imgBinary);

    
    int rowstart=220;
    int rowup=30;
    int entering_x0 = 0; // 进环巡远线起始点
    int entering_y0 = 0;
    int exiting_x0 = 0;  // 出环巡远线起始点
    int exiting_y0 = 0;


    std::vector<POINT> pointsLeft;
    std::vector<POINT> pointsRight;
    std::vector<POINT> pointsMid;
    std::vector<POINT> ring_points;
    std::vector<POINT> last_inner_side;
    std::vector<POINT> last_outer_side;

    std::vector<POINT> far_entering_edge;
    std::vector<POINT> far_exiting_edge;
    std::vector<POINT> t_far_entering_edge;
    std::vector<POINT> t_far_exiting_edge;
    std::vector<POINT> b_t_far_entering_edge;
    std::vector<POINT> b_t_far_exiting_edge;
    std::vector<POINT> s_b_t_far_entering_edge;
    std::vector<POINT> s_b_t_far_exiting_edge;

    int far_entering_edge_size;
    int far_exiting_edge_size;
    int t_far_entering_edge_size;
    int t_far_exiting_edge_size;
    int b_t_far_entering_edge_size;
    int b_t_far_exiting_edge_size;
    int s_b_t_far_entering_edge_size;
    int s_b_t_far_exiting_edge_size;



    Point exit_left_mid;
    Point exit_left_up;
    Point exit_right_up;
    Point exit_right_mid;

    Point last_Left_up;
    bool L_left_down_found;
    bool L_left_mid_found;
    bool L_left_up_found;
    bool L_right_down_found;
    bool L_right_mid_found;
    bool L_right_up_found;
    int L_id_left_down;
    int L_id_left_mid;
    int L_id_left_up;
    int L_id_right_down;
    int L_id_right_mid;
    int L_id_right_up;
    int ring_inside_counter = 0;
    int pre_counter = 0;
    int exitingnum = 0;
    int ring_entering_counter = 0;
    int inside_exit_flag = 0;
    int pre_entering_flag = 0 ;

    int left_no_size = 0;
    int right_no_size = 0;

    int mid_up_dis = 0 ;
    int mid_block = 0 ;
    int ring_points_flag = 0 ;

    int pointsLeft_size = 0;
    int pointsRight_size = 0;
    int pointsMid_size = 0;
    int ring_points_size = 0;
    int inside_ring_points_size = 0 ;
    int thresOTSU = 128;
    

    void ring_find_line(Mat &img,int y_start);

    Point find_left_down();//寻左下拐点
    Point find_right_down();//寻右下拐点
    Point find_left_mid(int start);//int start
    Point find_right_mid(int start);
    Point find_left_up();//寻左上拐点
    Point find_right_up();//寻右上拐点

    bool entering_lose_line_flag = false;


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

    // 左手巡线
    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeLeft, int &pointsEdgeLeft_size);

    // 右手巡线
    void findline_righthand_adaptive(Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeRight, int &pointsEdgeRight_size);

    int block_size = 9; // 自适应阈值block大小
    int clip_value = 3; // 自适应阈值的阈值剪裁量
    double pixel_per_meter = 222.222; // 一米在画面中所占的像素数
    double SAMPLE_DIST = 0.02;        // 采样间距 单位m
    double ROAD_WIDTH = 0.45;         // 道路真实宽度 单位m

    bool exiting_loseline_flag = false;

    // 滤波
    void blur_points(int side, int kernel);

    // 等距采样
    void resample_points(vector<POINT> &in, int in_size, vector<POINT> &out, int &out_size, float dist);

    void entering_track_far_line(cv::Mat imgBinary);

    void exiting_track_far_line(cv::Mat imgBinary);


};