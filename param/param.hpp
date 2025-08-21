#pragma once

#include <string>
#include "../include/json.hpp"
struct Config
{
    std::string jsonPth;

    int wifi_start = 0; //1为无线发车 0为网线发车
    int ttyUsb = 0;
    bool en_show = 0; // 控制窗口显示
    bool en_AI_show = false;

    float speedLow = 0.8;    // 智能车最低速
    float speedHigh = 0.8;   // 智能车最高速
    float speedBridge = 0.6; // 坡道速度
    float speedRing = 0.2;  //圆环速度
    float speedCross = 0.2; // 十字速度
    float speedDown = 0.5;   // 特殊区域降速速度

    float aim_distance_far = 0.70;
    float aim_distance_near = 0.30;
    float aim_distance_ring = 0.60;

    float ring_entering_p_k = 0.01;
    float ring_p_k = 0.05;
    float steering_p = 1.555;
    float steering_d = 6.666;
    float curve_p = 0;
    float curve_d = 0;
    float ring_d = 1.18;


    double exposure = 0.0125;
    int temprature = 4600;
    double brightness = 0.5;

    bool debug = false;         // 调试模式使能
    bool saveImg = false;       // 存图使能
    bool enAI = false;
    // uint16_t rowCutUp = 10;     // 图像顶部切行
    // uint16_t rowCutBottom = 10; // 图像顶部切行

    std::vector<int> elem_order; // 元素顺序表
    std::vector<int> speed_order;
    std::vector<float> aim_dis_n_order;
    std::vector<float> aim_angle_p_order;
    std::vector<float> aim_angle_d_order;
    std::vector<int> obstacle_order_index;


    // string video = "../res/samples/output_video.mp4";          // 视频路径
    std::string video = "/home/edgeboard/ACCM2025/output.mp4"; // 视频路径
    std::string model = "/home/edgeboard/ACCM2025/icar/model/yolov3_mobilenet_v1";
    float score = 0.4;

    

    int CATERING_STOP_TIMESTAMP = 0;



    int CHARGING_LEFT_FIRST_GOIN_TIME = 0; //左一车库进库时间
    int CHARGING_LEFT_SECOND_GOIN_TIME; //左二车库进库时间
    int CHARGING_RIGHT_FIRST_GOIN_TIME; //右一车库进库时间
    int CHARGING_RIGHT_SECOND_GOIN_TIME; //右二车库进库时间
    int CHARGING_LEFT_FIRST_OUT_TIME_ADD; //左一车库进库时间
    int CHARGING_LEFT_SECOND_OUT_TIME_ADD; //
    int CHARGING_RIGHT_FIRST_OUT_TIME_ADD; 
    int CHARGING_RIGHT_SECOND_OUT_TIME_ADD; 
    int CHARGING_GOIN_AIM_SPEED;

    int STOP_EXPECT_Y;

    int LAYBY_SHIFT_LEFT=10;//左边线偏移量
    int LAYBY_SHIFT_RIGHT=20;//右边线偏移量
    int LAYBY_PARKING_TIMER=70;//停车时间
    int LAYBY_AIM_SPEED = 50;

    int OBSTACLE_width_conical;
    int OBSTACLE_width_pedestrian; 
    int OBSTACLE_width_block;
    int block_aim_speed;
    int cone_aim_speed;

    float cone_p = 0;
    float cone_d = 0;
    float pe_p = 0;
    float pe_d = 0;
    float block_p = 0;
    float block_d = 0;


    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, jsonPth,wifi_start,ttyUsb, en_show, en_AI_show,speedLow, speedHigh, speedBridge,speedRing, speedCross,speedDown,aim_distance_far,aim_distance_near,aim_distance_ring,
                                       ring_entering_p_k,ring_p_k,steering_p,steering_d,curve_p,curve_d, ring_d,exposure,temprature,brightness, debug, saveImg, enAI,
                                       elem_order,speed_order,aim_dis_n_order,aim_angle_p_order,aim_angle_d_order,obstacle_order_index, model, video, score,CATERING_STOP_TIMESTAMP,
                                        CHARGING_LEFT_FIRST_GOIN_TIME,CHARGING_LEFT_SECOND_GOIN_TIME,CHARGING_RIGHT_FIRST_GOIN_TIME,CHARGING_RIGHT_SECOND_GOIN_TIME,CHARGING_LEFT_FIRST_OUT_TIME_ADD,CHARGING_LEFT_SECOND_OUT_TIME_ADD,CHARGING_RIGHT_FIRST_OUT_TIME_ADD,CHARGING_RIGHT_SECOND_OUT_TIME_ADD,CHARGING_GOIN_AIM_SPEED,
                                        STOP_EXPECT_Y,LAYBY_SHIFT_LEFT,LAYBY_SHIFT_RIGHT,LAYBY_PARKING_TIMER,LAYBY_AIM_SPEED,OBSTACLE_width_conical,OBSTACLE_width_pedestrian,OBSTACLE_width_block,block_aim_speed,cone_aim_speed,
                                        cone_p,cone_d,pe_p,pe_d,block_p,block_d
                                        ); // 添加构造函数    };
    
};