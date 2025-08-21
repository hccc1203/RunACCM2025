/*************************************/
// Author 孙酩贺
/*************************************/

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "../../include/json.hpp"
#include "../standard/general.h"
#include "../../param/param.hpp"
#include "../../include/detection.hpp"
#include "../../include/logger.hpp"
#include "../imgprocess/imgprocess.h"

class Obstacle 
{
public:
    Obstacle();
    Obstacle(Config &config);

    enum flag_obstacle_e {
        OBSTACLE_NONE,
        OBSTACLE_CONICAL,
        OBSTACLE_BLACK_BRICK,
        OBSTACLE_PEDESTRAIN
    };
    enum flag_tab_e{
        TAB_LEFT,
        TAB_RIGHT
    };
    flag_obstacle_e flag_obstacle;
    flag_tab_e flag_tab;
    General general;

    int OBSTACLE_width_conical;
    int OBSTACLE_width_pedestrian; 
    int OBSTACLE_width_block;


    void check_obstacle(cv::Mat & src_img,vector<POINT> & pointsEdgeLeft,vector<POINT> & pointsEdgeRight, std::vector<PredictResult> &predict_result,bool is_straight_left,bool is_straight_right);
    void run_obstacle(cv::Mat & src_img,vector<POINT> & pointsEdgeLeft,vector<POINT> & pointsEdgeRight,vector<POINT>& AI_CenterEdge,int pointsEdgeRight_size,int pointsEdgeLeft_size, std::vector<PredictResult> &predict_result,cv::Mat imgBinary,std::vector<POINT> &t_pointsEdgeLeft,int &t_pointsEdgeLeft_size,
        std::vector<POINT> &t_pointsEdgeRight,int &t_pointsEdgeRight_size,bool is_straight_left,bool is_straight_right);

    Logger logger = Logger("Obstacle");

};