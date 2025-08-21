/*************************************/
// Author 胡城玮
/*************************************/


#pragma once
#include "../imgprocess/imgprocess.h"
#include "../../param/param.hpp"
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../../include/json.hpp"
#include "../../include/logger.hpp"
// #include "../basic/garage.h"
#include "../basic/ring.h"
#include "../basic/cross.h"
#include "../special/obstacle.h"
#include "../special/catering.h"
#include "../special/layby.h"
#include "../special/bridge.h"
#include "../special/charging.h"
#include "../special/crosswalk.h"    
#include "general.h"
#include <string.h>
#include <fstream>
// #include "../special/ramp.h"
// #include "../special/find_sign.hpp"
// #include "../special/rescue.h"

#include <vector>
#include <string>
#include <iostream>


struct TaskData
{
    cv::Mat img;
    std::chrono::time_point<std::chrono::_V2::steady_clock, std::chrono::duration<long int, std::ratio<1, 1000000000>>> timestamp;
    bool buzzer_enable = 0;
    int pwm = PWMSERVOMID; // middle
    double speed = 0;      // m/s * 100

};

class Standard
{
public:
    enum TrackState
    {
        TRACK_LEFT,
        TRACK_BOTH,
        TRACK_RIGHT,
        TRAK_AI_MIDDLE
    };
    TrackState trackState;
    VideoWriter videoWriter;

    enum Scene
    {
        NormalScene = 0, // 基础赛道
        CrossScene,      // 十字道路
        RingScene,       // 环岛道路
        BridgeScene,     // 坡道区
        ObstacleScene,   // 障碍区
        CateringScene,   // 快餐店
        LaybyScene,      // 临时停车区
        ChargingScene,   // 充电停车区
        StopScene,        // 停车（结束）
        CrosswalkScene   //斑马线
    };
    Scene scene = Scene::NormalScene;

public:
    ImageProcess _imgprocess;
    General general;
    Obstacle obstacle;
    Catering catering;
    Layby layby;
    Bridge bridge;
    Charging charging;
    Ring ring;
    Cross cross;
    fstream fst;
    Crosswalk crosswalk;
    bool Stop_flag=false;

public:
    Standard();
    Standard(Config config);

    // 主处理 巡线+拟合
    TaskData run(cv::Mat src_img, std::vector<PredictResult> &predict_result,float pitch_angle);
    Config _config;

    vector<POINT> pointsEdgeLeft;   // 赛道左边缘点集
    vector<POINT> pointsEdgeRight;  // 赛道右边缘点集
    vector<POINT> t_pointsEdgeLeft; // 透视变换后点集
    vector<POINT> t_pointsEdgeRight;
    vector<POINT> b_t_pointsEdgeLeft; // 滤波后点集
    vector<POINT> b_t_pointsEdgeRight;
    vector<POINT> s_b_t_pointsEdgeLeft; // 等距采样后点集
    vector<POINT> s_b_t_pointsEdgeRight;
    vector<POINT> a_t_pointsEdgeLeft; // 计算角度后点集
    vector<POINT> a_t_pointsEdgeRight;
    vector<POINT> n_a_t_pointsEdgeLeft; // 非极大值抑制
    vector<POINT> n_a_t_pointsEdgeRight;
    vector<POINT> t_right_CenterEdge;
    vector<POINT> t_left_CenterEdge;
    vector<POINT> t_both_CenterEdge;
    vector<POINT> CenterEdge;   // 透视变换前中线
    vector<POINT> t_CenterEdge;     //透视变换后中线
    vector<POINT> AI_CenterEdge;    //透视变换前AI中线
    vector<POINT> AI_t_CenterEdge;  //透视变换后AI中线

    int pointsEdgeLeft_size;
    int pointsEdgeRight_size;
    int t_pointsEdgeLeft_size;
    int t_pointsEdgeRight_size;
    int b_t_pointsEdgeLeft_size; // 滤波后点集
    int b_t_pointsEdgeRight_size;
    int s_b_t_pointsEdgeLeft_size; // 等距采样后点集
    int s_b_t_pointsEdgeRight_size;
    int a_t_pointsEdgeLeft_size; // 边线局部角度变化量后点集
    int a_t_pointsEdgeRight_size;
    int n_a_t_pointsEdgeLeft_size; // 非极大值抑制
    int n_a_t_pointsEdgeRight_size;
    int CenterEdge_size;
    int t_CenterEdge_size;
    int AI_CenterEdge_size;
    int AI_t_CenterEdge_size;
    int t_right_CenterEdge_size;
    int t_left_CenterEdge_size;
    int t_both_CenterEdge_size;

    double thresOTSU; // 大津法阈值
    int countRows = ROWSIMAGE;
    int countCols = COLSIMAGE;
    int rowCutUp = 20;
    int rowCutBottom = 170;
    int block_size = 9; // 自适应阈值block大小
    int clip_value = 3; // 自适应阈值的阈值剪裁量
    int WIDTH = 320;
    int HEIGHT = 240;
    int x0; // 寻线起始点_左
    int y0;
    int L_count = -1;
    int x1; // 寻线起始点_右
    int y1;
    int R_count = -1;
    int x_offset = 20;  // 起始点距离图像中心的左右偏移量
    int y_offset = 180; // 起始点距离图像中心的上下偏移量（纵坐标）
    int Lconf_Min = 70;
    int Lconf_Max = 120;

    int t_L_pointLeft_id;
    int t_L_pointRight_id;
    Point t_L_pointLeft;
    Point t_L_pointRight;
    bool is_t_L_pointLeft_find;
    bool is_t_L_pointRight_find;

    bool is_left_straight;  // 左边直道标志位
    bool is_right_straight; // 右边直道标志位
    bool is_left_curve;
    bool is_right_curve;

    cv::Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

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

    std::vector<int> _elem_order;
    int order_index = 0;
    std::vector<float> _aim_dis_n_order;
    std::vector<float> _aim_dis_f_order;
    std::vector<int> _speed_order;
    std::vector<float>_aim_angle_p_order;
    std::vector<float>_aim_angle_d_order;
    std::vector<int> obstacel_order_index;


private:
    // 基础巡线
    void trackRecognition(cv::Mat &imageGray, cv::Mat &imageBinary,cv::Mat & imgShow);

    // 左手巡线
    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeLeft, int &pointsEdgeLeft_size);

    // 右手巡线
    void findline_righthand_adaptive(Mat &img, int block_size, int clip_value, int x, int y, vector<POINT> &pointsEdgeRight, int &pointsEdgeRight_size);


    // 直道判断
    void line_straight_detection();

    // 寻找拐点
    void find_corners();

    // 滤波
    void blur_points(int side, int kernel);

    // 等距采样
    void resample_points(vector<POINT> &in, int in_size, vector<POINT> &out, int &out_size, float dist);

    // 计算角度
    void local_angle_points(vector<POINT> pointsEdgeIn, int size, vector<POINT> &pointsEdgeOut, int dist);

    // 非极大值抑制
    void nms_angle(vector<POINT> &in, int in_size, vector<POINT> &out, int kernel);

    // 拟合中线
    void fitting();

    // 生成中线
    void centerCompute(vector<POINT> pointsEdge, int size, int side);

    // 两边拟合中线
    void track_both_edge();

    // 动态预瞄点计算
    double DynamicAimDisCal();

    struct PointsCurve{
        double x,y;
        double dx,dy;
        double ddx,ddy;
        double curve;
    };

private:
    float cx = COLSIMAGE / 2.0f; // 车轮对应点 (纯跟踪起始点)
    float cy = ROWSIMAGE * 0.999f;
    int aim_index_far;
    int aim_index_near;
    int approx_num = 5;              // 算中线时抽样间距
    double pixel_per_meter = 222.222; // 一米在画面中所占的像素数
    double SAMPLE_DIST = 0.02;        // 采样间距 单位m
    double ROAD_WIDTH = 0.45;         // 道路真实宽度 单位m
    double dist = pixel_per_meter * ROAD_WIDTH / 2.0f;
    // double wheelbase_val = 0.2;    // 车轴距
    double car_length = 0.316; // 车身长

    float aim_distance_f;
    float aim_distance_n;
    float aim_angle_p_k;
    float ring_entering_p_k;
    float ring_aim_p_k;
    float aim_angle_p;
    float aim_angle_d;

public:
    float aim_speed = 0.f;
    float aim_angle = 0.0f;      // 偏差量
    float aim_angle_last = 0.0f; // 偏差量 上一帧
    float aim_sigma = 0.0f;      // 偏差方差、

    int layby_end_counter = 0;
    
    int burger_end_counter = 0;


    Point2f src_points[4];
    Point2f dst_points[4];
    cv::Point seedPoint = cv::Point(160,239);
    
    std::vector<POINT> bezier_line;

    Logger logger = Logger("Standard");

};
