/*************************************/
// Author 胡城玮
/*************************************/


#include "charging.h"
#include "../../include/detection.hpp"
#include "../../include/logger.hpp"
#include "../standard/general.h"
#include <opencv2/opencv.hpp> // 确保包含OpenCV头文件
#include <vector>
#include <string>


#define PREDICT_TIMES 4
#define PREDICT_THRESH 2

static int timer_exit = 0;
static int timer_back_normal = 0;
static int timer_to_correct = 0;
static int count_parking = 0;
static int navigating_count = 0;
static int no_blob_count = 0;

Charging::Charging(){}
Charging::Charging(Config &config){}





void Charging::check_charging(std::vector<PredictResult> &predict_result, cv::Mat &src_img,int type) {
    static int counter = 0;
    bool detected = false;
    for (const auto& res : predict_result) {
        if (res.type == 0) {
            detected = true;
            break;
        }
    }
    // 状态转换逻辑
    if (chargingStep == Charging::Charging_None) {
        if (detected) {
            if (++counter >= PREDICT_THRESH) {
                chargingStep = Charging::Charging_FindGarage;
                logger.info("Charging detected!"); // 确保logger已正确初始化
                counter = 0;
                switch(type%8){
                    case 0:
                        garage_type = Left_First;
                        break;
                    case 1:
                        garage_type = Left_Second;
                        break;
                    case 2:
                        garage_type = Right_First;
                        break;
                    case 3:
                        garage_type = Right_Second;
                }
            }
        } else {
            counter = 0;
        }
    }
}

void Charging::run_charging(cv::Mat imgBinary,std::vector<POINT> &t_pointsEdgeLeft,int &t_pointsEdgeLeft_size,
                                            std::vector<POINT> &t_pointsEdgeRight,int &t_pointsEdgeRight_size,
                                            bool is_straight_left,bool is_straight_right,
                                        bool is_t_L_left_found,bool is_t_L_right_found,
                                        int t_L_left_id,int t_L_right_id) 
{   
    pointsEdgeLeft.clear();
    pointsEdgeRight.clear();

    for(int i = 0;i < t_pointsEdgeLeft_size;i++)
    {
        int a,b;
        general.Reverse_transf(a,b,t_pointsEdgeLeft[i].x,t_pointsEdgeLeft[i].y);
        pointsEdgeLeft.emplace_back(a,b);
    }
    for(int i = 0;i < t_pointsEdgeRight_size;i++)
    {
        int a,b;
        general.Reverse_transf(a,b,t_pointsEdgeRight[i].x,t_pointsEdgeRight[i].y);
        pointsEdgeRight.emplace_back(a,b);
    }
    pointsEdgeLeft_size = pointsEdgeLeft.size();
    pointsEdgeRight_size = pointsEdgeRight.size();
    

    if(garage_type == Right_First) // 右一车库
    {
        if(((is_t_L_right_found && t_L_right_id < 5)) && chargingStep == Charging_FindGarage) // 丢线及找到车库
        {
            logger("find garage");
            chargingStep = Charging_GoIn;
            GoIn_Line.clear();
            GoIn_Line_size = 0;
        }
        else if(chargingStep == Charging_FindGarage && t_pointsEdgeRight_size > 15) // 确定拉线终点搜寻起始点
        {
            logger("find garage");
            if(is_t_L_right_found)
            {
                Start_Point.x = t_pointsEdgeRight[t_L_right_id].x;
                Start_Point.y = t_pointsEdgeRight[t_L_right_id].y;
            }
            else if(t_pointsEdgeRight[15].x > 160){
            Start_Point.x = t_pointsEdgeRight[15].x;
            Start_Point.y = t_pointsEdgeRight[15].y;
            }
        }
        else if(chargingStep == Charging_GoIn) // 入库
        {
            logger("Goin");
            printf("GoIn counter %d\n",GoIn_Counter);
            if(GoIn_Line_size == 0) // 只拉一次线 确保稳定性
            {
                for(int y0 = Start_Point.y;y0 > 140;y0--)
                {
                    if(imgBinary.at<char>(y0,Start_Point.x) == 0 || y0 == 141)
                    {
                        Start_Point.y = y0;
                        break;
                    }
                }
                std::vector<POINT> v_center(2); // 贝塞尔拉线
                v_center[0] = t_pointsEdgeLeft[0];
                v_center[1] = Start_Point;
                
                GoIn_Line = general.Bezier(0.01,v_center);
                GoIn_Line_size = GoIn_Line.size();
            }
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++) // 拉线
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
            GoIn_Counter++;
            int Stop_y0;
            for(Stop_y0 = 240;Stop_y0 > 200;Stop_y0--)
            {
                if(imgBinary.at<char>(Stop_y0,210) == 0)
                {
                    break;
                }
            }
            if(GoIn_Counter > CHARGING_RIGHT_FIRST_GOIN_TIME)
            {
                chargingStep = Charging_stop;
            }

        }
        else if(chargingStep == Charging_stop) // 固定时间停车并保持当前打角
        {
            logger("stop");
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
            static int counter = 0;
            counter ++;
            if(counter > 30)
            {
                chargingStep = Charging_GoOut;
                GoIn_Counter += CHARGING_RIGHT_FIRST_OUT_TIME_ADD;
                counter = 0;
            }  
        }
        else if(chargingStep == Charging_GoOut) // 出库 退出相同时间
        {
            logger("GoOut");
            GoIn_Counter--;          
            if(GoIn_Counter < 0)
            {
                if(1)
                {
                    chargingStep = Charging_Leave;
                    return;
                }
            }
            // if(GoIn_Counter <= -20)
            // {
            //     chargingStep = Charging_Leave;
            //     return;
                
            // }
            
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
        }
        else if(chargingStep == Charging_Leave) // 保持寻完整边线一段时间
        {
            static int leave_counter = 0;
            leave_counter ++;
            if(leave_counter > 50)
            {
                leave_counter = 0;
                chargingStep = Charging_None;
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////
    else if(garage_type == Right_Second)
    {
        if(chargingStep == Charging_FindGarage)
        {
            static int no_line_counter;
            no_line_counter = 0;
            int x0;
            if(t_pointsEdgeRight_size > 30)
            {
                Start_Point.x = pointsEdgeRight[10].x;
                Start_Point.y = pointsEdgeRight[10].y;
            }
            for(int y0 = 210;y0 > 140;y0--)
            {
                for(x0 = 230;x0 < 280;x0++)
                {
                    if(imgBinary.at<uchar>(y0,x0) < 128)
                        break;
                }
                printf("%d\n",x0);
                if(x0 == 280)
                {
                    no_line_counter++;
                }
                if(no_line_counter > 50)
                {
                    chargingStep = Charging_GoIn;
                    no_line_counter = 0;
                    GoIn_Line.clear();
                    GoIn_Line_size = 0;
                    break;
                }
            }
        }
        else if(chargingStep == Charging_GoIn)
        {
            logger("Charging_Goin");
            if(GoIn_Line_size == 0) // 只拉一次线 确保稳定性
            {
                for(int y0 = Start_Point.y;y0 > 140;y0--)
                {
                    if(imgBinary.at<char>(y0,Start_Point.x) == 0 || y0 == 141)
                    {
                        Start_Point.y = y0;
                        break;
                    }
                }
                std::vector<POINT> v_center(2); // 贝塞尔拉线
                v_center[0] = POINT(pointsEdgeLeft[0].x,pointsEdgeLeft[0].y + 10);
                v_center[1] = Start_Point;
                
                GoIn_Line = general.Bezier(0.01,v_center);
                GoIn_Line_size = GoIn_Line.size();
                for(int i = 0;i < GoIn_Line_size;i++)
                {
                    general.transf(GoIn_Line[i].x,GoIn_Line[i].y,GoIn_Line[i].x,GoIn_Line[i].y);
                }
            }
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++) // 拉线
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
            GoIn_Counter++;
            if(GoIn_Counter > CHARGING_RIGHT_SECOND_GOIN_TIME)
            {
                chargingStep = Charging_stop;
            }
        }
        else if(chargingStep == Charging_stop) // 固定时间停车并保持当前打角
        {
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
            static int counter = 0;
            counter ++;
            if(counter > 30)
            {
                chargingStep = Charging_GoOut;
                GoIn_Counter+=CHARGING_RIGHT_SECOND_OUT_TIME_ADD;
                counter = 0;
            }
        }
        else if(chargingStep == Charging_GoOut) // 出库 退出相同时间
        {
            GoIn_Counter--;
            if(GoIn_Counter == 0)
            {
                chargingStep = Charging_Leave;
                return;
            }
            t_pointsEdgeLeft.clear();
            t_pointsEdgeRight_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeLeft_size = GoIn_Line_size;
        }
        else if(chargingStep == Charging_Leave) // 保持寻完整边线一段时间
        {
            static int leave_counter = 0;
            leave_counter ++;
            if(leave_counter > 20)
            {
                leave_counter = 0;
                chargingStep = Charging_None;
            }
        }
    }
    ////////////////////////////////////////////////
    if(garage_type == Left_First) // 左一车库
    {
        if(is_t_L_left_found && t_L_left_id < 7 && chargingStep == Charging_FindGarage) // 丢线及找到车库
        {
            chargingStep = Charging_GoIn;
            GoIn_Line.clear();
            GoIn_Line_size = 0;
        }
        else if(chargingStep == Charging_FindGarage && t_pointsEdgeLeft_size > 15) // 确定拉线终点搜寻起始点
        {
            Start_Point.x = t_pointsEdgeLeft[15].x;
            Start_Point.y = t_pointsEdgeLeft[15].y;
        }
        else if(chargingStep == Charging_GoIn) // 入库
        {
            if(GoIn_Line_size == 0) // 只拉一次线 确保稳定性
            {
                for(int y0 = Start_Point.y;y0 > 140;y0--)
                {
                    if(imgBinary.at<char>(y0,Start_Point.x) == 0 || y0 == 141)
                    {
                        Start_Point.y = y0;
                        break;
                    }
                }
                std::vector<POINT> v_center(2); // 贝塞尔拉线
                v_center[0] = pointsEdgeRight[0];
                v_center[1] = Start_Point;
                
                GoIn_Line = general.Bezier(0.01,v_center);
                GoIn_Line_size = GoIn_Line.size();
                for(int i = 0;i < GoIn_Line_size;i++)
                {
                    general.transf(GoIn_Line[i].x,GoIn_Line[i].y,GoIn_Line[i].x,GoIn_Line[i].y);
                }
            }
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++) // 拉线
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
            GoIn_Counter++;
            int Stop_y0;
            for(Stop_y0 = 240;Stop_y0 > 200;Stop_y0--)
            {
                if(imgBinary.at<char>(Stop_y0,110) == 0)
                {
                    break;
                }
            }
            if(GoIn_Counter > CHARGING_LEFT_FIRST_GOIN_TIME)
            {
                chargingStep = Charging_stop;
            }

        }
        else if(chargingStep == Charging_stop) // 固定时间停车并保持当前打角
        {
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
            static int counter = 0;
            counter ++;
            if(counter > 30)
            {
                chargingStep = Charging_GoOut;
                GoIn_Counter += CHARGING_LEFT_FIRST_OUT_TIME_ADD;
                counter = 0;
            }
        }
        else if(chargingStep == Charging_GoOut) // 出库 退出相同时间
        {
            GoIn_Counter--;
            if(GoIn_Counter == 0)
            {
                chargingStep = Charging_Leave;
                return;
            }
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
        }
        else if(chargingStep == Charging_Leave) // 保持寻完整边线一段时间
        {
            static int leave_counter = 0;
            leave_counter ++;
            if(leave_counter > 50)
            {
                leave_counter = 0;
                chargingStep = Charging_None;
            }
        }
    }
    //////////////////////////////////////////////////////////
    //左二车库
    else if(garage_type == Left_Second)
    {
        if(chargingStep == Charging_FindGarage)
        {
            static int no_line_counter;
            no_line_counter = 0;
            int x0;
            if(t_pointsEdgeLeft_size > 30)
            {
                Start_Point.x = pointsEdgeLeft[10].x;
                Start_Point.y = pointsEdgeLeft[10].y;
            }
            for(int y0 = 210;y0 > 130;y0--)
            {
                for(x0 =20;x0 < 90;x0++)
                {
                    if(imgBinary.at<uchar>(y0,x0) < 128)
                        break;
                }
                printf("%d\n",x0);
                if(x0 == 90)
                {
                    no_line_counter++;
                }
                if(no_line_counter > 45)
                {
                    chargingStep = Charging_GoIn;
                    no_line_counter = 0;
                    GoIn_Line.clear();
                    GoIn_Line_size = 0;
                    break;
                }
            }
        }
        else if(chargingStep == Charging_GoIn)
        {
            logger("Charging_Goin");
            if(GoIn_Line_size == 0) // 只拉一次线 确保稳定性
            {
                for(int y0 = Start_Point.y;y0 > 140;y0--)
                {
                    if(imgBinary.at<char>(y0,Start_Point.x) == 0 || y0 == 141)
                    {
                        Start_Point.y = y0;
                        break;
                    }
                }
                std::vector<POINT> v_center(2); // 贝塞尔拉线
                v_center[0] = POINT(pointsEdgeRight[0].x,pointsEdgeRight[0].y + 10);
                v_center[1] = Start_Point;
                
                GoIn_Line = general.Bezier(0.01,v_center);
                GoIn_Line_size = GoIn_Line.size();
                for(int i = 0;i < GoIn_Line_size;i++)
                {
                    general.transf(GoIn_Line[i].x,GoIn_Line[i].y,GoIn_Line[i].x,GoIn_Line[i].y);
                }
            }
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++) // 拉线
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
            GoIn_Counter++;
            // int Stop_y0;
            // for(Stop_y0 = 240;Stop_y0 > 200;Stop_y0--)
            // {
            //     if(imgBinary.at<char>(Stop_y0,110) == 0)
            //     {
            //         break;
            //     }
            // }
            if(GoIn_Counter > CHARGING_LEFT_SECOND_GOIN_TIME)
            {
                chargingStep = Charging_stop;
            }
        }
        else if(chargingStep == Charging_stop) // 固定时间停车并保持当前打角
        {
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
            static int counter = 0;
            counter ++;
            if(counter > 30)
            {
                chargingStep = Charging_GoOut;
                GoIn_Counter+=CHARGING_LEFT_SECOND_OUT_TIME_ADD;
                counter = 0;
            }
        }
        else if(chargingStep == Charging_GoOut) // 出库 退出相同时间
        {
            GoIn_Counter--;
            if(GoIn_Counter == 0)
            {
                chargingStep = Charging_Leave;
                return;
            }
            t_pointsEdgeRight.clear();
            t_pointsEdgeLeft_size = 0;
            for(int i = 0;i < GoIn_Line_size;i++)
            {
                t_pointsEdgeRight.emplace_back(GoIn_Line[i].x,GoIn_Line[i].y);
            }
            t_pointsEdgeRight_size = GoIn_Line_size;
        }
        else if(chargingStep == Charging_Leave) // 保持寻完整边线一段时间
        {
            static int leave_counter = 0;
            leave_counter ++;
            if(leave_counter > 20)
            {
                leave_counter = 0;
                chargingStep = Charging_None;
            }
        }
    }
}
