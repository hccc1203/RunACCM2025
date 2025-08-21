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
#include <cmath>


class Charging{
    public:
        Charging();
        Charging(Config &config);
        enum ChargingStep {
            Charging_None,
            Charging_FindGarage,
            Charging_GoIn,
            Charging_stop,
            Charging_GoOut,
            Charging_Leave
        };
        enum Garage_Type {
            Left_First,
            Left_Second,
            Right_First,
            Right_Second
        };
        ChargingStep chargingStep = Charging_None;
        Garage_Type garage_type;
        General general;
        

        void check_charging(std::vector<PredictResult> &predict_result,cv::Mat &src_img,int type);
        void run_charging(cv::Mat imgBinary,std::vector<POINT> &t_pointsEdgeLeft,int &t_pointsEdgeLeft_size,
                                            std::vector<POINT> &t_pointsEdgeRight,int &t_pointsEdgeRight_size,
                                            bool is_straight_left,bool is_straight_right,bool is_t_L_left_found,bool is_t_L_right_found,
                                            int t_L_left_id,int t_L_right_id);

        Logger logger = Logger("Charging");
        std::vector<POINT> navigationPath;  // 记录正向导航路径
        size_t exitPathIndex = 0;           // 反向路径执行索引

        vector<POINT> pointsEdgeLeft;       //原图边线及长度
        vector<POINT> pointsEdgeRight;
        int pointsEdgeLeft_size = 0;
        int pointsEdgeRight_size = 0;

        POINT Start_Point;
        vector<POINT> GoIn_Line;
        int GoIn_Line_size;
        int GoIn_Counter = 0;

        int CHARGING_LEFT_FIRST_GOIN_TIME; //左一车库进库时间
        int CHARGING_LEFT_SECOND_GOIN_TIME; //左二车库进库时间
        int CHARGING_RIGHT_FIRST_GOIN_TIME; //右一车库进库时间
        int CHARGING_RIGHT_SECOND_GOIN_TIME; //右二车库进库时间
        int CHARGING_LEFT_FIRST_OUT_TIME_ADD; //左一车库进库时间
        int CHARGING_LEFT_SECOND_OUT_TIME_ADD; //
        int CHARGING_RIGHT_FIRST_OUT_TIME_ADD; 
        int CHARGING_RIGHT_SECOND_OUT_TIME_ADD; 
};