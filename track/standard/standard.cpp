#include "standard.h"
#include "../../include/logger.hpp"

/*************************************/
// Author 胡城玮
/*************************************/


Standard::Standard() {}

// 顺序定义
// 1.汉堡
// 2.虚线停车
// 4.障碍
// 5.圆环
// 6.十字
// 7.桥
// 0.终止位
// 8.左1充电 
// 9.左2充电 
// 10.右1充电 
// 11.右2充电
Standard::Standard(Config config)
{

    // videoWriter.open("../output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(COLSIMAGE, ROWSIMAGE), true);

    this->rowCutBottom = 170;
    this->rowCutUp = 20;

    this->_config = config;
    
    for(int i = 0;i<config.elem_order.size();i++)
    {
        _elem_order.emplace_back(config.elem_order[i]);
    }
    for(int i = 0;i<config.aim_dis_n_order.size();i++)
    {
        _aim_dis_n_order.emplace_back(config.aim_dis_n_order[i]);
    }
    for(int i = 0;i<config.speed_order.size();i++)
    {
        _speed_order.emplace_back(config.speed_order[i]);
    }
    for(int i = 0;i<config.aim_angle_p_order.size();i++)
    {
        _aim_angle_p_order.emplace_back(config.aim_angle_p_order[i]);
    }
    for(int i = 0;i<config.aim_angle_d_order.size();i++)
    {
        _aim_angle_d_order.emplace_back(config.aim_angle_d_order[i]);
    }



    catering.CATERING_STOP_TIMESTAMP = config.CATERING_STOP_TIMESTAMP;

    charging.CHARGING_LEFT_FIRST_GOIN_TIME = config.CHARGING_LEFT_FIRST_GOIN_TIME;
    charging.CHARGING_LEFT_SECOND_GOIN_TIME = config.CHARGING_LEFT_SECOND_GOIN_TIME;
    charging.CHARGING_RIGHT_FIRST_GOIN_TIME = config.CHARGING_RIGHT_FIRST_GOIN_TIME;
    charging.CHARGING_RIGHT_SECOND_GOIN_TIME = config.CHARGING_RIGHT_SECOND_GOIN_TIME;
    charging.CHARGING_LEFT_FIRST_OUT_TIME_ADD = config.CHARGING_LEFT_FIRST_OUT_TIME_ADD;
    charging.CHARGING_LEFT_SECOND_OUT_TIME_ADD = config.CHARGING_LEFT_SECOND_OUT_TIME_ADD;
    charging.CHARGING_RIGHT_FIRST_OUT_TIME_ADD = config.CHARGING_RIGHT_FIRST_OUT_TIME_ADD;
    charging.CHARGING_RIGHT_SECOND_OUT_TIME_ADD = config.CHARGING_RIGHT_SECOND_OUT_TIME_ADD;

    crosswalk.STOP_EXPECT_Y = config.STOP_EXPECT_Y;

    layby.LAYBY_SHIFT_LEFT = config.LAYBY_SHIFT_LEFT;
    layby.LAYBY_SHIFT_RIGHT = config.LAYBY_SHIFT_RIGHT;
    layby.LAYBY_PARKING_TIMER = config.LAYBY_PARKING_TIMER;

    obstacle.OBSTACLE_width_block = config.OBSTACLE_width_block;
    obstacle.OBSTACLE_width_conical = config.OBSTACLE_width_conical;
    obstacle.OBSTACLE_width_pedestrian = config.OBSTACLE_width_pedestrian;

    fst.open("/home/edgeboard/RUN_ACCM2025_icar/track/standard/data.txt",ios::out);
    fst << "";
    fst.close();
    
}

struct SceneStatus
{
    bool ObstacleScene;
    bool CateringScene;
    bool LaybyScene;
    bool BridgeScene;
    bool ChargingScene;
    bool CrosswalkScene;

    SceneStatus()
    {
        this->CateringScene = false;
        this->ObstacleScene = false;
        this->LaybyScene = false;
        this->BridgeScene = false;
        this->ChargingScene = false;
        this->CrosswalkScene = false;
    }

    bool all()
    {
        return this->CateringScene || this->ObstacleScene || this->LaybyScene || this->BridgeScene || this->ChargingScene || this->CrosswalkScene;
    }
} scene_status;

TaskData Standard::run(cv::Mat src_img,
                       std::vector<PredictResult> &predict_result,
                       float pitch_angle)
{
    logger(pitch_angle);
    TaskData dst;
    cv::Mat imgGray;
    cv::Mat imgShow = src_img.clone();
    cv::cvtColor(src_img, imgGray, cv::COLOR_BGR2GRAY);
    cv::Mat imgBinary;
    thresOTSU = cv::threshold(imgGray, imgBinary, 0, 255, cv::THRESH_OTSU);

    bool beep_flag = 0;
    // 处理图像
    trackRecognition(imgGray, imgBinary,imgShow);

    fitting();

    /* **************************************************************** */
    /* **************************** 选定巡线类型 *********************** */
    /* *********************** 单侧线少 切换巡线方向 ******************** */
    if ((t_pointsEdgeRight_size - t_pointsEdgeLeft_size) > 1)
    {
        trackState = TrackState::TRACK_RIGHT;
    }
    else if ((t_pointsEdgeLeft_size - t_pointsEdgeRight_size) > 1)
    {
        trackState = TrackState::TRACK_LEFT;
    }
    else if (t_pointsEdgeLeft_size != 0 && t_pointsEdgeRight_size != 0)
    {
        trackState = TrackState::TRACK_BOTH;
    }

    int same_points = 0;
    if(trackState == TrackState::TRACK_BOTH)
    {
        for(int i = 0;i < pointsEdgeLeft_size / 2;i++)
        {
            for(int j = pointsEdgeRight_size / 2;j < pointsEdgeRight_size;j++)
            {
                if(pointsEdgeLeft[i].x == pointsEdgeRight[j].x && pointsEdgeLeft[i].y == pointsEdgeRight[j].y)
                {
                    same_points++;
                    printf("same\n");
                }
                if(same_points > 3)
                {
                    break;
                }
            }
            if(same_points > 3)
            {
                break;
            }
        }
        if(same_points > 3)
        {
            if(aim_angle_last < 0)
                trackState = TrackState::TRACK_LEFT;
            else
                trackState = TrackState::TRACK_RIGHT;
        }
    }

    // if (is_left_straight && is_right_straight)
    //     trackState = TrackState::TRACK_BOTH;
    // else if (is_left_straight)
    //     trackState = TrackState::TRACK_LEFT;
    // else if (is_right_straight)
    //     trackState = TrackState::TRACK_RIGHT;
    // else if (!is_left_straight && !is_right_straight &&
    // t_both_CenterEdge[t_both_CenterEdge_size - 1].x > COLSIMAGE / 2 &&
    // t_pointsEdgeLeft_size > 40 && t_pointsEdgeRight_size > 40)
    //     trackState = TrackState::TRACK_LEFT;
    // else if (!is_left_straight && !is_right_straight &&
    // t_both_CenterEdge[t_both_CenterEdge_size - 1].x < COLSIMAGE / 2 &&
    // t_pointsEdgeLeft_size > 40 && t_pointsEdgeRight_size > 40)
    //     trackState = TrackState::TRACK_RIGHT;
    // else if (t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size != 0)
    //     trackState = TrackState::TRACK_RIGHT;
    // else if (t_pointsEdgeLeft_size != 0 && t_pointsEdgeRight_size == 0)
    //     trackState = TrackState::TRACK_LEFT;
    // else if (t_pointsEdgeLeft_size < t_pointsEdgeRight_size / 2)
    //     trackState = TrackState::TRACK_RIGHT;
    // else if (t_pointsEdgeLeft_size / 2 > t_pointsEdgeRight_size)
    //     trackState = TrackState::TRACK_LEFT;
    // else if (t_pointsEdgeLeft_size < 10 && t_pointsEdgeLeft_size <
    // t_pointsEdgeRight_size)
    //     trackState = TrackState::TRACK_RIGHT;
    // else if (t_pointsEdgeRight_size < 10 && t_pointsEdgeLeft_size >
    // t_pointsEdgeRight_size)
    //     trackState = TrackState::TRACK_LEFT;
    // else
    //     trackState = TrackState::TRACK_BOTH;

    /* ***************************************************************** */
    /* **************************** 元素检测 **************************** */
    /* ***************************************************************** */

    if (scene == Scene::NormalScene && _config.enAI)
    {
        //////////////////////
        // 障碍 //////////////
        /////////////////////
        if (obstacle.flag_obstacle == Obstacle::OBSTACLE_NONE)
        {
            if(_config.obstacle_order_index[order_index])
            {
                obstacle.check_obstacle(imgShow, pointsEdgeLeft, pointsEdgeRight, predict_result,is_left_straight,is_right_straight);
            }
            if (obstacle.flag_obstacle != Obstacle::OBSTACLE_NONE)
            {
                scene = Scene::ObstacleScene;
            }
        }

        //////////////////////
        // 汉堡 //////////////
        /////////////////////
        if ((catering.flag_catering == Catering::CATERING_NONE && _elem_order[order_index] == 1) )
        {
            catering.check_catering(predict_result,is_t_L_pointLeft_find,is_t_L_pointRight_find);
            if (catering.flag_catering != Catering::CATERING_NONE)
            {
                scene = Scene::CateringScene;
            }
        }

        ///////////////////////
        ///临时停车区//////////
        // /////////////////////

        if (layby.flag_layby == Layby::LAYBY_NONE && _elem_order[order_index] == 2)
        {
            layby.check_layby(imgShow,predict_result);
            if (layby.flag_layby != Layby::LAYBY_NONE)
            {
                scene = Scene::LaybyScene;
            }
        }

        /////////////
        ///桥////////
        /////////////
        if (bridge.flag_bridge == Bridge::BRIDGE_NONE && _elem_order[order_index] == 7)
        {
            bridge.check_bridge(predict_result,src_img,pitch_angle);
            if (bridge.flag_bridge != Bridge::BRIDGE_NONE)
            {
                scene = Scene::BridgeScene;
            }
            // if(burger_end_counter >= 100 && burger_end_counter <= 110)
            // {
            //     scene = Scene::BridgeScene;
            // }
        }
        ///////////////
        ///充电场//////
        //////////////
        if (charging.chargingStep == Charging::Charging_None && (_elem_order[order_index] >= 8 && _elem_order[order_index] <= 11))
        {
            charging.check_charging(predict_result,imgBinary,_elem_order[order_index]);
            if (charging.chargingStep != Charging::Charging_None)
            {
                scene = Scene::ChargingScene;
            }
        }
    }
    //////////////////////
    //斑马线//////////////
    /////////////////////
    // 如您看到此注释且此代码是通过购买得来请立刻联系QQ2447981178
    if (crosswalk.flag_crosswalk == Crosswalk::CROSSWALK_NONE && _elem_order[order_index] == 0)
    {
        crosswalk.check_crosswalk(predict_result);
        if (crosswalk.flag_crosswalk == Crosswalk::CROSSWALK_NORMAL)
        {
            scene = Scene::CrosswalkScene;
        }
    }




    //////////////////////
    // 圆环 //////////////
    /////////////////////
    
    if (scene == Scene::NormalScene  &&  _elem_order[order_index] == 5 && burger_end_counter>50)
    {
        ring.Ring_Check(imgBinary, is_left_straight,is_right_straight, is_t_L_pointLeft_find, is_t_L_pointRight_find,t_pointsEdgeLeft_size,t_pointsEdgeRight_size,is_t_L_pointLeft_find,is_t_L_pointRight_find,t_L_pointLeft_id,t_L_pointRight_id,t_pointsEdgeLeft,t_pointsEdgeRight);
        if (ring.flag_ring != Ring::Ring_None)
        {
            scene = Scene::RingScene;
        }
    }
    //////////////////////
    // 十字 //////////////
    /////////////////////
    if (scene == Scene::NormalScene  && burger_end_counter > 100  && _elem_order[order_index] == 6)
    {
        cross.Cross_Check(is_t_L_pointLeft_find, is_t_L_pointRight_find, imgBinary,t_L_pointLeft,t_L_pointRight,t_L_pointLeft_id,t_L_pointRight_id,t_pointsEdgeLeft_size,t_pointsEdgeRight_size);
        if (cross.flag_cross != Cross::Cross_None)
        {
            scene = Scene::CrossScene;
        }
    }

    int custom_speed = -1;
    float custom_speed_rato = -1.0;

    /* ***************************************************************** */
    /* **************************** 元素处理 **************************** */
    /* ***************************************************************** */
    burger_end_counter ++;
    if(burger_end_counter > 600)
    {
        burger_end_counter = 599;
    }
    if (scene == Scene::ObstacleScene)
    {
        obstacle.run_obstacle(imgShow,pointsEdgeLeft,pointsEdgeRight,AI_CenterEdge,pointsEdgeRight_size,pointsEdgeLeft_size,predict_result,imgBinary,t_pointsEdgeLeft,t_pointsEdgeLeft_size,
                              t_pointsEdgeRight,t_pointsEdgeRight_size,is_left_straight,is_right_straight);
        logger("ObstacleScene");
        AI_CenterEdge_size = AI_CenterEdge.size();
        if (obstacle.flag_obstacle == Obstacle::OBSTACLE_NONE)
        {
            scene = Scene::NormalScene;
            // order_index++;
        // }
        // if(obstacle.flag_tab == Obstacle::TAB_LEFT)
        // {
        //     trackState = TRACK_LEFT;
        // }
        // if(obstacle.flag_tab == Obstacle::TAB_RIGHT)
        // {
        //     trackState = TRACK_RIGHT;
        // }
        }
        // pointsEdgeLeft_size = pointsEdgeLeft.size();
        // pointsEdgeRight_size = pointsEdgeRight.size();
        // if(obstacle.flag_tab == Obstacle::TAB_LEFT)
        // {
        //     trackState = TRACK_LEFT;
        // }
        // else if(obstacle.flag_tab == Obstacle::TAB_RIGHT)
        // {
        //     trackState = TRACK_RIGHT;
        // }
        trackState =TrackState::TRAK_AI_MIDDLE;
    }
    else if (scene == Scene::CateringScene)
    {
        catering.run_catering(predict_result,imgShow, pointsEdgeLeft, pointsEdgeRight,
                              AI_CenterEdge, pointsEdgeRight_size,
                              pointsEdgeLeft_size, 
                              is_t_L_pointLeft_find,is_t_L_pointRight_find);
        logger("CateringScene");
        if (catering.flag_catering == Catering::CATERING_NONE)
        {
            scene = Scene::NormalScene;
            order_index++;
            burger_end_counter = 0;
        }
        
        if(catering.turn_dir == Catering::TURN_LEFT)
        {
            printf("left!!!!!!!!!!!!!!!!!!!!!!!\n\n\n");
            trackState = TRACK_LEFT;
        }
        if(catering.turn_dir == Catering::TURN_RIGHT)
        {
            printf("right!!!!!!!!!!!!!!!!!!!!!!!\n\n\n");
            trackState = TRACK_RIGHT;
        }
    }
    else if (scene == Scene::LaybyScene)
    {
        layby.run_layby(imgShow,predict_result, pointsEdgeLeft, pointsEdgeRight,AI_CenterEdge,custom_speed_rato);
            logger("LaybyScene");
            AI_CenterEdge_size = AI_CenterEdge.size();
            if (layby.flag_layby == Layby::LAYBY_NONE)
            {
                scene = Scene::NormalScene;
                order_index++;
            }
            trackState = TrackState::TRAK_AI_MIDDLE;
    }
    else if (scene == Scene::CrosswalkScene)
    {
        
        crosswalk.run_crosswalk(predict_result);
        logger("crosswalk");
        // if (crosswalk.flag_crosswalk == Crosswalk::CROSSWALK_NONE)
        // {
        //     scene = Scene::NormalScene;///////////////////////////////////////////////////////////////////////
        //     order_index++;
        // }
        trackState = TrackState::TRACK_BOTH;
        
    }
    else if(scene == Scene::BridgeScene)
    {
        logger("bridge");
        bridge.run_bridge();
        static int bridge_counter = 0;
        if(bridge_counter > 48 && t_pointsEdgeRight[t_pointsEdgeRight_size/3*2].x - t_pointsEdgeLeft[t_pointsEdgeLeft_size / 3*2].x > 60)
        {
            burger_end_counter = 30;
            scene = Scene::NormalScene;
            bridge.flag_bridge = Bridge::BRIDGE_NONE;
            order_index++;

        }
        else if (t_pointsEdgeLeft_size > 20 && t_pointsEdgeRight_size > 20)
        {
            t_pointsEdgeLeft_size /= 3;
            t_pointsEdgeRight_size /= 3;
            trackState = TRACK_BOTH;
            track_both_edge();
            bridge_counter++;
        }
        
        // 如您看到此注释且此代码是通过购买得来请立刻联系QQ2447981178
        //     AI_CenterEdge_size = AI_CenterEdge.size();
        //     if (bridge.flag_bridge == Bridge::BRIDGE_NONE)
        //     {
        //         scene = Scene::NormalScene;
        //            order_index++;
        //     }
        //     trackState = TrackState::TRAK_AI_MIDDLE;
    }
    else if (scene == Scene::ChargingScene)
    {
        charging.run_charging(imgBinary,t_pointsEdgeLeft,t_pointsEdgeLeft_size,
                                        t_pointsEdgeRight,t_pointsEdgeRight_size,
                                        is_left_straight,is_right_straight,
                                        is_t_L_pointLeft_find,is_t_L_pointRight_find,
                                        t_L_pointLeft_id,t_L_pointRight_id);
        logger("ChargingScene");
        if(charging.chargingStep == Charging::Charging_None)
        {
            scene = Scene::NormalScene;
            order_index++;
        }
        centerCompute(t_pointsEdgeLeft, t_pointsEdgeLeft_size, 0);
        t_left_CenterEdge_size = t_left_CenterEdge.size();

        centerCompute(t_pointsEdgeRight, t_pointsEdgeRight_size, 1);
        t_right_CenterEdge_size = t_right_CenterEdge.size();

        if(charging.chargingStep != Charging::Charging_None && (charging.garage_type == Charging::Right_First || charging.garage_type == Charging::Right_Second))
        {
            trackState = TRACK_LEFT;
        }
        if(charging.chargingStep != Charging::Charging_None && (charging.garage_type == Charging::Left_First || charging.garage_type == Charging::Left_Second))
        {
            trackState = TRACK_RIGHT;
        }
    }
    else if (scene == Scene::RingScene)
    {
        ring.Ring_Check(imgBinary, is_left_straight,is_right_straight, 
                        is_t_L_pointLeft_find, is_t_L_pointRight_find,
                        t_pointsEdgeLeft_size,t_pointsEdgeRight_size,
                        is_t_L_pointLeft_find,is_t_L_pointRight_find,
                        t_L_pointLeft_id,t_L_pointRight_id,
                        t_pointsEdgeLeft,t_pointsEdgeRight); // 圆环状态更新
        ring.Ring_Run(t_pointsEdgeLeft,t_pointsEdgeRight,t_pointsEdgeLeft_size,t_pointsEdgeRight_size,
                        is_t_L_pointLeft_find,is_t_L_pointRight_find,t_L_pointLeft_id,t_L_pointRight_id,
                        imgBinary);

        logger("RingScene");
        if ((t_pointsEdgeRight_size - t_pointsEdgeLeft_size) > 1)
        {
            trackState = TrackState::TRACK_RIGHT;
        }
        else if ((t_pointsEdgeLeft_size - t_pointsEdgeRight_size) > 1)
        {
            trackState = TrackState::TRACK_LEFT;
        }

        centerCompute(t_pointsEdgeLeft, t_pointsEdgeLeft_size, 0);
        t_left_CenterEdge_size = t_left_CenterEdge.size();

        centerCompute(t_pointsEdgeRight, t_pointsEdgeRight_size, 1);
        t_right_CenterEdge_size = t_right_CenterEdge.size();

        if (ring.flag_ring == Ring::Ring_None)
        {
            scene = Scene::NormalScene;
            order_index++;

        }
    }
    else if (scene == Scene::CrossScene)
    {
        cross.Cross_Run(t_pointsEdgeLeft, t_pointsEdgeRight, imgGray,is_t_L_pointLeft_find, is_t_L_pointRight_find,t_L_pointLeft_id, t_L_pointRight_id,t_pointsEdgeLeft_size,t_pointsEdgeRight_size);
        t_pointsEdgeLeft_size = t_pointsEdgeLeft.size();
        t_pointsEdgeRight_size = t_pointsEdgeRight.size();

        if ((t_pointsEdgeRight_size - t_pointsEdgeLeft_size) > 1)
        {
            trackState = TrackState::TRACK_RIGHT;
        }
        else if ((t_pointsEdgeLeft_size - t_pointsEdgeRight_size) > 1)
        {
            trackState = TrackState::TRACK_LEFT;
        }
        centerCompute(t_pointsEdgeLeft, t_pointsEdgeLeft_size, 0);
        t_left_CenterEdge_size = t_left_CenterEdge.size();

        centerCompute(t_pointsEdgeRight, t_pointsEdgeRight_size, 1);
        t_right_CenterEdge_size = t_right_CenterEdge.size();
        
        if (cross.flag_cross == Cross::Cross_None)
        {
            
            scene = Scene::NormalScene;
            order_index++;

        }
    }

    /* ***************************************************************** */
    /* **************************** 中线选择 **************************** */
    /* ***************************************************************** */
    // 如您看到此注释且此代码是通过购买得来请立刻联系QQ2447981178
    if (trackState == TrackState::TRACK_LEFT)
    {
        printf("track_left\n");
        for (int i = 0; i < t_left_CenterEdge_size; i++)
        {
            t_CenterEdge.emplace_back(t_left_CenterEdge[i].x,
                                      t_left_CenterEdge[i].y);
        }
        t_CenterEdge_size = t_CenterEdge.size();
    }
    else if (trackState == TrackState::TRACK_RIGHT)
    {
        printf("track_right\n");
        for (int i = 0; i < t_right_CenterEdge_size; i++)
        {
            t_CenterEdge.emplace_back(t_right_CenterEdge[i].x,
                                      t_right_CenterEdge[i].y);
        }
        t_CenterEdge_size = t_CenterEdge.size();
    }
    else if (trackState == TrackState::TRACK_BOTH)
    {
        printf("track_both\n");
        for (int i = 0; i < t_both_CenterEdge_size; i++)
        {
            t_CenterEdge.emplace_back(t_both_CenterEdge[i].x,
                                      t_both_CenterEdge[i].y);
        }
        t_CenterEdge_size = t_CenterEdge.size();
    }
    else if (trackState == TrackState::TRAK_AI_MIDDLE)
    {
        printf("Track_ai_middle\n");
        for (int i = 0; i < AI_CenterEdge_size; i++)
        {
            int a, b;
            general.transf(a, b, AI_CenterEdge[i].x, AI_CenterEdge[i].y);
            t_CenterEdge.emplace_back(a, b);
        }
        t_CenterEdge_size = t_CenterEdge.size();
        logger("t_CenterEdge_size: ", t_CenterEdge_size);
    }



    /* ***************************************************************** */
    /* **************************** 偏差计算 **************************** */
    /* ***************************************************************** */
    this->aim_distance_f = _config.aim_distance_far;
    this->aim_distance_n = _config.aim_distance_near;
    this->ring_entering_p_k = _config.ring_entering_p_k;
    this->ring_aim_p_k = _config.ring_p_k;
    this->aim_angle_p = _config.steering_p;
    this->aim_angle_d = _config.steering_d;

    // DynamicAimDisCal();

    if(scene == Scene::NormalScene) 
    {
        aim_distance_n = _aim_dis_n_order[order_index];
    }



    if(scene == Scene::RingScene)
    {
        aim_distance_n = _config.aim_distance_ring;
    }
    else if(scene == Scene::CrossScene)
    {
        aim_distance_n = 0.2;
    }




    float min_dist = 9999;
    int begin_id = -1;
    bool center_effective_flag = false; // 中线有效标志

    cx = COLSIMAGE / 2.0f; // 车轮对应点 (纯跟踪起始点)
    cy = ROWSIMAGE * 0.95f;

    // 找最近点(起始点中线归一化)
    for (int i = 0; i < t_CenterEdge_size; i++)
    {
        float dx = t_CenterEdge[i].x - cx;
        float dy = t_CenterEdge[i].y - cy;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            begin_id = i;
        }
    }

    begin_id = general.clip(begin_id, 0, t_CenterEdge_size - 1);

    std::vector<POINT> temp_center;

    int temp_center_size;
    printf("begin id%d size %d\n", begin_id, t_CenterEdge_size);
    if ((begin_id >= 0 && t_CenterEdge_size - begin_id >= 3))
    {
        center_effective_flag = true;
        if (begin_id > 0)
        {
            cx = t_CenterEdge[begin_id].x;
            cy = t_CenterEdge[begin_id].y;
        }
        for (int i = begin_id; i < t_CenterEdge_size; i++)
        {
            temp_center.emplace_back(t_CenterEdge[i].x, t_CenterEdge[i].y);
        }

        temp_center_size = temp_center.size();
        t_CenterEdge.clear();
        t_CenterEdge_size = 0;
        // logger("temp:size", temp_center_size);
        resample_points(temp_center, temp_center_size, t_CenterEdge,
                        t_CenterEdge_size, SAMPLE_DIST * pixel_per_meter);

        // float near_min_dist = 9999;
        // float far_min_dist = 9999;
        // for(int i = 0;i < t_CenterEdge_size;i++)
        // {
        //     double dx = t_CenterEdge[i].x - cx;
        //     double dy = t_CenterEdge[i].y - cy;
        //     double dn = sqrt(dx*dx + dy*dy) / pixel_per_meter;
        //     if(fabs(dn - aim_distance_n) < near_min_dist)
        //     {
        //         aim_index_near = i;
        //         near_min_dist = fabs(dn - aim_distance_n);
        //     }
        //     if(fabs(dn - aim_distance_f) < far_min_dist)
        //     {
        //         aim_index_far = i;
        //         far_min_dist = fabs(dn - aim_distance_f);
        //     }
        // }

        // printf("   %d %d \n",aim_index_near,aim_index_far);


        // aim_index_far = general.clip(round(aim_distance_f / SAMPLE_DIST), 0,
        //                              t_CenterEdge_size - 1);
        // aim_index_near = general.clip(round(aim_distance_n / SAMPLE_DIST), 0,
        //                               t_CenterEdge_size - 1);

        double min_dis = 1000000;

        for(int i = 1;i < t_CenterEdge_size;i++)
        {
            double dx = t_CenterEdge[i].x - cx;
            double dy = cy - t_CenterEdge[i].y;
            double dn = sqrt(dx*dx + dy*dy);

            double dis = aim_distance_f * pixel_per_meter - dn;
            if(dis < 0)
            {
                dis *= -1;
            }
            if(dis  < min_dis)
            {
                aim_index_far = i;
                min_dis = dis;
            }
        }

        min_dis = 1000000;
        for(int i = 1;i < t_CenterEdge_size;i++)
        {
            double dx = t_CenterEdge[i].x - cx;
            double dy = cy - t_CenterEdge[i].y;
            double dn = sqrt(dx*dx + dy*dy);

            double dis = aim_distance_n * pixel_per_meter - dn;
            if(dis < 0)
            {
                dis *= -1;
            }
            if(dis  < min_dis)
            {
                aim_index_near = i;
                min_dis = dis;
            }
        }


        // std::vector<POINT> v_center(4); // 三阶贝塞尔曲线
        // v_center[0] = {(int)t_CenterEdge[begin_id].x,
        // (int)t_CenterEdge[begin_id].y}; v_center[1] =
        // {(int)t_CenterEdge[aim_index_near].x, (int)(ROWSIMAGE * (1 -
        // aim_distance_n))}; v_center[2] =
        // {(int)t_CenterEdge[(int)((aim_index_far + aim_index_near) / 2)].x,
        //                (int)(ROWSIMAGE * (1 - (aim_distance_f +
        //                aim_distance_n) / 2))};
        // v_center[3] = {(int)t_CenterEdge[aim_index_far].x, (int)(ROWSIMAGE *
        // (1 - aim_distance_f))}; bezier_line = Bezier(0.03, v_center);

        // float dx = bezier_line[bezier_line.size() - 1].x - cx; //
        // rptsn[aim_idx__far][0] - cx; float dy = cy -
        // bezier_line[bezier_line.size() - 1].y; // cy -
        // rptsn[aim_idx__far][1]; float error_far = (-atan2f(dx, dy) * 180 /
        // PI); assert(!isnan(error_far));

        // // 计算近锚点偏差值
        // float dx_near = bezier_line[bezier_line.size() / 2].x - cx; //
        // rptsn[aim_idx_near][0] - cx; float dy_near = cy -
        // bezier_line[bezier_line.size() / 2].y; // cy -
        // rptsn[aim_idx_near][1]; float error_near = (-atan2f(dx_near, dy_near)
        // * 180 / PI); assert(!isnan(error_near));

        float dx =
            t_CenterEdge[aim_index_far].x - cx; // rptsn[aim_idx__far][0] - cx;
        float dy = cy - t_CenterEdge[aim_index_far].y +
                   car_length * pixel_per_meter; // cy - rptsn[aim_idx__far][1];
        float dn = sqrt(dx * dx + dy * dy);
        float error_far =
            (-atanf(pixel_per_meter * 2 * car_length * dx / dn / dn) * 180 /
             PI);
        assert(!isnan(error_far));
        printf("far_dx:%f,far_dy %f\n",dx,dy);
        printf("cx %f cy %f\n",cx,cy);
        // 计算近锚点偏差值
        float dx_near =
            t_CenterEdge[aim_index_near].x - cx; // rptsn[aim_idx_near][0] - cx;
        float dy_near =
            cy - t_CenterEdge[aim_index_near].y +
            car_length * pixel_per_meter; // cy - rptsn[aim_idx_near][1];
        float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
        float error_near = (-atanf(pixel_per_meter * 2 * car_length * dx_near /
                                   dn_near / dn_near) *
                            180 / PI);
        // 如您看到此注释且此代码是通过购买得来请立刻联系QQ2447981178
        assert(!isnan(error_near));

        // if ((is_left_straight && is_right_straight) &&
        //     scene == Scene::NormalScene)
        // {
        //     // aim_angle = error_far * 0.2 + error_near * 0.8;
        //     aim_angle = error_near;
        // }
        if (scene == Scene::ObstacleScene)
        {
            aim_angle = 0.001 * error_far + 0.999 * error_near;
        }
        else if (scene == Scene::NormalScene)
        {
            // aim_angle = error_far * 0.70 + error_near * 0.30;
            aim_angle = error_near;
        }
        else if (scene == Scene::CateringScene)
        {
            aim_angle = error_far * 0.40 + error_near * 0.60;
        }

        else if (scene == Scene::LaybyScene)
        {
            aim_angle = error_far * 0.05 + error_near * 0.95;
        }
        else if (scene == Scene::ChargingScene)
        {
            aim_angle = error_near * 0.95;
        }
        else if (scene == Scene::CrosswalkScene)
        {
            aim_angle = error_far * 0.95 + error_near * 0.05;
        }
        else if (scene == Scene::CrossScene)
        {
            aim_angle = error_near;
        }
        else if (scene == Scene::RingScene)
        {
            aim_angle = error_near;
        }
        else if(scene == Scene::BridgeScene)
        {
            aim_angle = error_near;
        }
        aim_sigma = general.sigma(bezier_line, aim_index_near, aim_index_far);
        if(pointsEdgeLeft_size && !pointsEdgeRight_size)
        {
            seedPoint = cv::Point(general.clip(pointsEdgeLeft[0].x + 5,0,319),pointsEdgeLeft[0].y + 10);
        }
        else if(!pointsEdgeLeft_size && pointsEdgeRight_size)
        {
            seedPoint = cv::Point(general.clip(pointsEdgeRight[0].x - 5,0,319),pointsEdgeRight[0].y + 10);
        }
        else if(pointsEdgeLeft_size && pointsEdgeRight_size)
        {
            seedPoint = cv::Point(general.clip((pointsEdgeRight[0].x + pointsEdgeRight[0].x)/2,0,319),(pointsEdgeRight[0].y + pointsEdgeRight[0].y)/2 + 10);
        }
    }
    else
    {
        center_effective_flag = false;
    }

    /* ***************************************************************** */
    /* **************************** 速度判定 **************************** */
    /* ***************************************************************** */

    aim_speed = _config.speedHigh;
    if (((is_left_curve || is_right_curve) || (!is_left_straight || !is_right_straight)) &&
        scene == Scene::NormalScene)
    {
        aim_speed = _speed_order[order_index];
    }

    
    if (scene == Scene::CateringScene)
    {
        aim_speed = _config.speedLow;
        if(catering.flag_catering == Catering::CATERING_PARK)
        {
            aim_speed = 20;
        }
    }

    if(scene == Scene::CrosswalkScene)
    {
        aim_speed = _config.speedLow;
    }
    if(scene == Scene::CrossScene)
    {
        aim_speed = _config.speedCross;
    }
    if(scene == Scene::RingScene)
    {
        aim_speed = _config.speedRing;
    }
    if(scene == Scene::LaybyScene)
    {
        aim_speed = _config.LAYBY_AIM_SPEED;
        if(layby.flag_layby == Layby::LAYBY_PARKING)
        {
            aim_speed = 0;
        }

    }
    if(scene == Scene::CrosswalkScene)
    {
        printf("222222222222222222/n");
        if(crosswalk.flag_crosswalk == Crosswalk::CROSSWALK_STOP)
        {
            Stop_flag = true;
        }
    }
    // 如您看到此注释且此代码是通过购买得来请立刻联系QQ2447981178

    if(scene == Scene::BridgeScene)
    {
        aim_speed = _config.speedBridge;
    }
    if(scene == Scene::ObstacleScene)
    {
        if(obstacle.flag_obstacle == Obstacle::OBSTACLE_BLACK_BRICK)
        {
            aim_speed = _config.block_aim_speed;
        }
        else{
            aim_speed = _config.cone_aim_speed;
        }
    }
    if(scene == Scene::ChargingScene)
    {
        aim_speed = _config.CHARGING_GOIN_AIM_SPEED;
        if(charging.chargingStep == Charging::Charging_stop)
        {
            aim_speed = 0;
        }
        if(charging.chargingStep == Charging::Charging_GoOut)
        {
            aim_speed = -1 * _config.CHARGING_GOIN_AIM_SPEED;
        }

    }


    /* ***************************************************************** */
    /* **************************** 运行控制 **************************** */
    /* ***************************************************************** */

    if (!center_effective_flag)
    {
        aim_angle = aim_angle_last;
    }
    // float aim_angle_filter = general.filter(aim_angle);
    float aim_angle_filter = aim_angle;
    aim_angle_last = aim_angle_filter;



    if (((is_left_curve || is_right_curve) || (!is_left_straight || !is_right_straight)) &&scene == Scene::NormalScene)
    {
        // aim_angle_p += fabs(aim_angle_filter) * aim_angle_p_k;
        // aim_angle_p = aim_angle_p > _config.steering_p * 5.0f
        //                   ? _config.steering_p * 5.0f
        //                   : aim_angle_p;
        // aim_angle_p = _config.curve_p;
        // aim_angle_d = _config.curve_d;

        aim_angle_p = _aim_angle_p_order[order_index];
        aim_angle_d = _aim_angle_d_order[order_index];
        // aim_angle_p = _config.steering_p_k;
    }
    // //两十字间环
    // else if(_elem_order[order_index] == 6 && _elem_order[order_index - 1] == 6)
    // {
    //     aim_angle_p = 1.8;
    // }
    else if (scene == Scene::ObstacleScene)
    {
        if(obstacle.flag_obstacle == Obstacle::OBSTACLE_BLACK_BRICK)
        {
            aim_angle_p = _config.block_p;
            aim_angle_d = _config.block_d; 
        }
        else if(obstacle.flag_obstacle == Obstacle::OBSTACLE_PEDESTRAIN)
        {
            aim_angle_p = _config.pe_p;
            aim_angle_d = _config.pe_p;
        }
        else{
            aim_angle_p = _config.cone_p;
            aim_angle_d = _config.cone_d;
        }
    }
    else if(scene == Scene::CrosswalkScene)
    {
        aim_angle_p = 0.5;
    }
    else if (scene == Scene::LaybyScene)
    {
        aim_angle_p = 0.7;
        aim_angle_d = 0;
    }
    else if (scene == Scene::CrossScene)
    {
        aim_angle_p = 0.75;
        aim_angle_d = 0.2;
    }
    else if (scene == Scene::ChargingScene)
    {
        aim_angle_p = 0.9;
        aim_angle_d = 0;
    }
    
    else if (scene == Scene::CrossScene && cross.flag_cross == Cross::Cross_Out)
    {
        aim_angle_p = 0.8;
        aim_angle_d = 0.6;
    }
    else if (scene == Scene::RingScene)
    {
        if(ring.flag_ring == Ring::Left_Ring_Entering || ring.flag_ring == Ring::Right_Ring_Entering)
        {
            aim_angle_p += fabs(aim_angle_filter) * ring_entering_p_k;
            aim_angle_p = aim_angle_p > _config.steering_p * 3.0f
                            ? _config.steering_p * 3.0f
                            : aim_angle_p;
        }
        else if(abs(aim_angle_filter) < 20)
        {
            aim_angle_p += fabs(aim_angle_filter) * ring_aim_p_k;
            aim_angle_p = aim_angle_p > _config.steering_p * 3.0f
                            ? _config.steering_p * 3.0f
                            : aim_angle_p;
        }
        else 
        {
            aim_angle_p += fabs(aim_angle_filter) * ring_aim_p_k;
            aim_angle_p = aim_angle_p > _config.steering_p * 3.0f
                            ? _config.steering_p * 3.0f
                            : aim_angle_p;
        }
        aim_angle_d = _config.ring_d;
    }

    


    double aim_angle_pwm = 0;

    aim_angle_pwm = (general.pid_realize_a(aim_angle_filter, 0.0f,
                                                aim_angle_p, aim_angle_d));

    // aim_angle_pwm = general.clip(aim_angle_pwm,-39,39);
    // if(aim_angle_pwm >= 0)
    //     aim_angle_pwm = (int)(-0.03292*aim_angle_pwm*aim_angle_pwm*aim_angle_pwm + 1.5258*aim_angle_pwm*aim_angle_pwm + 14.4723*aim_angle_pwm);
    // else
    // {
    //     double x = -aim_angle_pwm;
    //     aim_angle_pwm = (int)(-(-0.03292*x*x*x + 1.5258*x*x + 14.4723*x));
    // }    
    aim_angle_pwm*=30;

    aim_angle_pwm = PWMSERVOMID - aim_angle_pwm;

    


    aim_angle_pwm = general.clip(aim_angle_pwm, PWMSERVOMIN, PWMSERVOMAX);
    // aim_angle_pwm -= aim_angle_last;
    logger("error",aim_angle_filter);
    logger("p", aim_angle_p, "d", aim_angle_d);
    logger("pwm", aim_angle_pwm);

    /* ***************************************************************** */
    /* **************************** 绘图 ******************************* */
    /* ***************************************************************** */
    cv::Mat imgT;
    if(scene == Scene::CrossScene)
    {
        logger("CrossScene");
    }
    if (_config.en_show || _config.saveImg)
    {
        warpPerspective(src_img, imgT, general.rotation, src_img.size());
        cv::putText(imgT,"error"+std::to_string(aim_angle_filter),Point(20,20),cv::FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255));
        // imgT = src_img.clone();
        // for(int i = 0;i < pointsEdgeLeft_size;i++)
        // {
        //     circle(imgShow,Point(pointsEdgeLeft[i].x,pointsEdgeLeft[i].y),1,
        //         Scalar(0,255,0));
        // }
        // for(int i = 0;i < pointsEdgeRight_size;i++)
        // {
        //     circle(imgShow,Point(pointsEdgeRight[i].x,pointsEdgeRight[i].y),1,
        //         Scalar(0,0,255));
        // }

        for (int i = 0; i < t_CenterEdge_size; i++)
        {
            circle(imgT, Point(t_CenterEdge[i].x, t_CenterEdge[i].y), 1,
                   Scalar(0, 0, 255), -1); // 红色点
        }
        // for (int i = 0; i < bezier_line.size(); i++)
        // {
        //     circle(imgT, Point(bezier_line[i].x, bezier_line[i].y), 1,
        //            Scalar(0, 0, 255), -1); // 红色点
        // }

        for (int i = 0; i < t_pointsEdgeLeft_size; i++)
        {
            circle(imgT, Point(t_pointsEdgeLeft[i].x, t_pointsEdgeLeft[i].y), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }


        for (int i = 0; i < t_pointsEdgeRight_size; i++)
        {
            circle(imgT, Point(t_pointsEdgeRight[i].x, t_pointsEdgeRight[i].y),
                   1, Scalar(0, 255, 255), -1); // 绿色点
        }
        if (is_t_L_pointLeft_find)
        {
            circle(imgT, Point(t_L_pointLeft.x, t_L_pointLeft.y), 5,
                   Scalar(255, 0, 0), -1); // 蓝色点
        }
        if (is_t_L_pointRight_find)
        {
            circle(imgT, Point(t_L_pointRight.x, t_L_pointRight.y), 5,
                   Scalar(255, 0, 0), -1); // 蓝色点
        }

        if (center_effective_flag)
        {

            circle(imgT,
                   Point(t_CenterEdge[aim_index_near].x,
                         t_CenterEdge[aim_index_near].y),
                   5, Scalar(0, 255, 255), -1);
            circle(imgT,
                   Point(t_CenterEdge[aim_index_far].x,
                         t_CenterEdge[aim_index_far].y),
                   5, Scalar(0, 0, 255), -1);
        }
        if (scene == Scene::CrossScene && cross.flag_cross == Cross::Cross_Out)
        {
            circle(imgShow,
                   Point(cross.far_left_x0,cross.far_left_y0),
                   5, Scalar(0, 255, 0), -1);
            for(int i = 0;i < cross.far_pointsEdgeLeft_size;i++)
            {
                circle(imgShow,
                   Point(cross.far_pointsEdgeLeft[i].x,cross.far_pointsEdgeLeft[i].y),
                   1, Scalar(0, 255, 0), -1);
            }
            for(int i = 0;i < cross.far_pointsEdgeRight_size;i++)
            {
                circle(imgShow,
                   Point(cross.far_pointsEdgeRight[i].x,cross.far_pointsEdgeRight[i].y),
                   5, Scalar(0, 255, 255), -1);
            }
        }
        if (scene == Scene::CrossScene && cross.flag_cross == Cross::Cross_Out)
        {
            circle(imgShow,
                   Point(cross.far_right_x0,cross.far_right_y0),
                   5, Scalar(0, 255, 0), -1);
        }

        // if (scene == RingScene && (ring.flag_ring == Ring::Left_Ring_Entering || ring.flag_ring == Ring::Right_Ring_Entering))
        // {
        //     circle(imgShow,Point(ring.entering_x0,ring.entering_y0),5,Scalar(255,0,0),-1);

        //     for(int i = 0;i < ring.far_entering_edge_size;i++)
        //     {
        //         circle(imgShow,Point(ring.far_entering_edge[i].x,ring.far_entering_edge[i].y),1,Scalar(255,0,0),-1);
        //     }
        // }
        // if(scene == RingScene)
        // 
        for(int i = 0;i < ring.pointsLeft_size;i++)
        {
            circle(imgShow,Point(ring.pointsLeft[i].x,ring.pointsLeft[i].y),1,Scalar(255,0,0),-1);
            circle(imgShow,Point(ring.pointsRight[i].x,ring.pointsRight[i].y),1,Scalar(255,255,0),-1);
        }
        // }

        
        if (scene == RingScene && (ring.flag_ring == Ring::Left_Ring_Exiting || ring.flag_ring == Ring::Right_Ring_Exiting))
        {
            circle(imgShow,Point(ring.exiting_x0,ring.exiting_y0),5,Scalar(255,0,0),-1);

            for(int i = 0;i < ring.far_exiting_edge_size;i++)
            {
                circle(imgShow,Point(ring.far_exiting_edge[i].x,ring.far_exiting_edge[i].y),1,Scalar(255,0,0),-1);
            }
        }

        if(scene == ChargingScene)
        {
            int a,b;
            general.transf(a,b,charging.Start_Point.x,charging.Start_Point.y);
            circle(imgT,Point(a,b),5,Scalar(255,0,255),-1);
        }

        

        if(1)
        {

            circle(imgShow,Point(seedPoint.x,seedPoint.y),5,Scalar(255,255,0),-1);
        }
    }
    if (_config.saveImg)
    {
        general.savePicture(imgT);
        // general.savePicture(imgBinary);
        // general.savePicture(imgLayby);          
        // general.savePicture(src_img);
        // general.savePicture(imgShow);
        // videoWriter << imgT;
    }

    fst.open("/home/edgeboard/ACCM2025/RUN_ACCM2025_icar/track/standard/data.txt",ios::out|ios::app);
    fst << aim_angle_filter << "," << aim_angle_pwm << endl;
    fst.close();
    beep_flag = (scene != Scene::NormalScene);
    return TaskData{imgT, std::chrono::steady_clock::now(),
                    beep_flag, (int)(aim_angle_pwm), aim_speed};
}

//////////////////////////////////////////////////////////////////////////////////
//                                分割线                                         //
//////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Description    : 基础巡线
 *******************************************************************************/
void Standard::trackRecognition(cv::Mat &imageGray, cv::Mat &imageBinary,cv::Mat & imgShow)
{
    pointsEdgeLeft_size = 0;
    pointsEdgeRight_size = 0; // 边线长度清零
    pointsEdgeLeft.clear();
    pointsEdgeRight.clear();
    t_pointsEdgeLeft_size = 0;
    t_pointsEdgeRight_size = 0; // 边线长度清零
    t_pointsEdgeLeft.clear();
    t_pointsEdgeRight.clear();
    b_t_pointsEdgeLeft_size = 0;
    b_t_pointsEdgeRight_size = 0;
    b_t_pointsEdgeLeft.clear(); // 滤波后点集
    b_t_pointsEdgeRight.clear();
    s_b_t_pointsEdgeLeft_size = 0;
    s_b_t_pointsEdgeRight_size = 0;
    s_b_t_pointsEdgeLeft.clear(); // 等距采样后点集
    s_b_t_pointsEdgeRight.clear();
    a_t_pointsEdgeLeft_size = 0;
    a_t_pointsEdgeRight_size = 0;
    a_t_pointsEdgeLeft.clear(); // 等距采样后点集
    a_t_pointsEdgeRight.clear();
    n_a_t_pointsEdgeLeft_size = 0;
    n_a_t_pointsEdgeRight_size = 0;
    n_a_t_pointsEdgeLeft.clear(); // 等距采样后点集
    n_a_t_pointsEdgeRight.clear();
    is_t_L_pointLeft_find = false;
    is_t_L_pointRight_find = false;
    is_left_straight = false;
    is_right_straight = false;
    is_left_curve = false;
    is_right_curve = false;
    t_L_pointLeft_id = 0;
    t_L_pointRight_id = 0;
    
    // 洪水填充
    
    cv::Mat mask = cv::Mat::zeros(242,322, CV_8UC1);
    bool flood_effect = false;
    // 在底部区域寻找白色点
    if(imageBinary.at<uchar>(seedPoint.y,seedPoint.x) != 255)
    {
        if(aim_angle_last > 0)
        {
            for (int x = 150; x > 80; x--) {
                if (imageBinary.at<uchar>(239, x) == 255) {
                    seedPoint = cv::Point(x, 239);
                    flood_effect = true;
                    break;
                }
            }
        }
        else
        {
            for (int x = 170; x < 240; x++) {
                if (imageBinary.at<uchar>(239, x) == 255) {
                    seedPoint = cv::Point(x, 239);
                    flood_effect = true;
                    break;
                }
            }
        }
    }
    else 
    {
        flood_effect = true;
    }
    
    if(flood_effect)
    {
        cv::floodFill(imageBinary,mask,seedPoint,127,0,0,4);
        for (int y = 0; y < ROWSIMAGE; y++) 
        {
            for (int x = 0; x < COLSIMAGE; x++) 
            {
                if (imageBinary.at<uchar>(y, x) == 127) 
                {
                    imageBinary.at<uchar>(y, x) = 255;
                }
                else
                {
                    imageBinary.at<uchar>(y, x) = 0;
                }
            }
        }
    }
    mask.release();

    // 最长白列算法
    int left_start[320] = {0};
    int right_start[320] = {0};
    x0 = 160;
    x1 = 160;

    for (int i = 0; i < 320; i++)
    {
        for (int j = 220; j > 100; j--)
        {
            if (imageBinary.at<char>(j, i) < 128)
            {
                break;
            }
            else
            {
                left_start[i]++;
                right_start[i]++;
            }
        }
    }
    for (int i = 1; i < 320; i++)
    {
        if (left_start[i] > left_start[x0])
        {
            x0 = i;
        }
        if (right_start[320 - i] > right_start[x1])
        {
            x1 = 320 - i;   
        }
    }
    printf("x0 %d x1 %d\n", x0, x1);

    if(_config.saveImg || _config.en_show)
    {
        circle(imgShow,Point(x0,220),5,Scalar(0,255,0),-1);
        circle(imgShow,Point(x1,220),5,Scalar(0,255,255),-1);
    }
    

    // [0]迷宫法寻左右边线
    // 左手
    y0 = rowCutBottom;

    for (; x0 > 0; x0--)
    {

        if (imageBinary.at<char>(y0, (x0 - 1)) < thresOTSU)
        {
            break; // 找到赛道边缘即退出
        }
    }
    


    if (imageBinary.at<char>(y0, x0) >= thresOTSU && x0 > 10)
    {
        findline_lefthand_adaptive(imageBinary, block_size, clip_value, x0, y0,
                                   pointsEdgeLeft, pointsEdgeLeft_size);
    }
    else
    {
        pointsEdgeLeft_size = 0;
    }
    // 右手
    y1 = rowCutBottom;



    for (; x1 < WIDTH; x1++)
    {

        if (imageBinary.at<char>(y1, x1 + 1) < thresOTSU)
        {

            break; // 找到赛道边缘即退出
        }
    }


    if (imageBinary.at<char>(y1, x1) >= thresOTSU && x1 < WIDTH - 10)
    {

        findline_righthand_adaptive(imageBinary, block_size, clip_value, x1, y1,
                                    pointsEdgeRight, pointsEdgeRight_size);
    }
    else
    {
        pointsEdgeRight_size = 0;
    }

    if(pointsEdgeLeft_size > 8)
    {
        bool circular_flag = true;//判断线是否为环形
        for(int i = 0;i < 4;i++)
        {
            if(pointsEdgeLeft[i].x != pointsEdgeLeft[i + 4].x && pointsEdgeLeft[i].y != pointsEdgeLeft[i + 4].y)
            {
                circular_flag = false;
            }
        }
        if(circular_flag)
        {
            pointsEdgeLeft_size = 0;
        }
    }

    if(pointsEdgeRight_size > 8)
    {
        bool circular_flag = true;//判断线是否为环形
        for(int i = 0;i < 4;i++)
        {
            if(pointsEdgeRight[i].x != pointsEdgeRight[i + 4].x && pointsEdgeRight[i].y != pointsEdgeRight[i + 4].y)
            {
                circular_flag = false;
            }
        }
        if(circular_flag)
        {
            pointsEdgeRight_size = 0;
        }
    }


    ////////////////////////////////////////////////////////////////////////
    // 透视变换
    ////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < pointsEdgeLeft.size(); i++)
    {
        int a, b;
        general.transf(a, b, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y);
        t_pointsEdgeLeft.emplace_back(a, b);
    }
    t_pointsEdgeLeft_size = pointsEdgeLeft_size;
    for (int i = 0; i < pointsEdgeRight.size(); i++)
    {
        int a, b;
        general.transf(a, b, pointsEdgeRight[i].x, pointsEdgeRight[i].y);
        t_pointsEdgeRight.emplace_back(a, b);
    }
    t_pointsEdgeRight_size = pointsEdgeRight_size;

    ////////////////////////////////////////////////////////////////////////
    // 边线滤波
    ////////////////////////////////////////////////////////////////////////
    blur_points(0, 11);
    blur_points(1, 11);
    b_t_pointsEdgeLeft_size = t_pointsEdgeLeft_size;
    b_t_pointsEdgeRight_size = t_pointsEdgeRight_size;

    ////////////////////////////////////////////////////////////////////////
    // 等距采样
    ////////////////////////////////////////////////////////////////////////
    resample_points(b_t_pointsEdgeLeft, b_t_pointsEdgeLeft_size,
                    s_b_t_pointsEdgeLeft, s_b_t_pointsEdgeLeft_size,
                    SAMPLE_DIST * pixel_per_meter);
    resample_points(b_t_pointsEdgeRight, b_t_pointsEdgeRight_size,
                    s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                    SAMPLE_DIST * pixel_per_meter);

    ////////////////////////////////////////////////////////////////////////
    // 角度计算
    ////////////////////////////////////////////////////////////////////////
    local_angle_points(s_b_t_pointsEdgeLeft, s_b_t_pointsEdgeLeft_size,
                       a_t_pointsEdgeLeft, 11);
    a_t_pointsEdgeLeft_size = a_t_pointsEdgeLeft.size();
    local_angle_points(s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                       a_t_pointsEdgeRight, 11);
    a_t_pointsEdgeRight_size = a_t_pointsEdgeRight.size();

    // ////////////////////////////////////////////////////////////////////////
    // // 非极大值抑制
    // ////////////////////////////////////////////////////////////////////////
    nms_angle(a_t_pointsEdgeLeft, a_t_pointsEdgeLeft_size, n_a_t_pointsEdgeLeft,
              22);
    n_a_t_pointsEdgeLeft_size = n_a_t_pointsEdgeLeft.size();
    nms_angle(a_t_pointsEdgeRight, a_t_pointsEdgeRight_size,
              n_a_t_pointsEdgeRight, 22);
    n_a_t_pointsEdgeRight_size = n_a_t_pointsEdgeRight.size();

    ///////////////////////////////////////////////////////////////////////
    // 将处理后边线放回t_点集
    ///////////////////////////////////////////////////////////////////////
    t_pointsEdgeLeft.clear();
    t_pointsEdgeRight.clear();
    t_pointsEdgeLeft_size = 0;
    t_pointsEdgeRight_size = 0;
    for (int i = 0; i < s_b_t_pointsEdgeLeft_size; i++)
    {
        t_pointsEdgeLeft.emplace_back(s_b_t_pointsEdgeLeft[i].x,
                                      s_b_t_pointsEdgeLeft[i].y);
        t_pointsEdgeLeft_size++;
    }
    for (int i = 0; i < s_b_t_pointsEdgeRight_size; i++)
    {
        t_pointsEdgeRight.emplace_back(s_b_t_pointsEdgeRight[i].x,
                                       s_b_t_pointsEdgeRight[i].y);
        t_pointsEdgeRight_size++;
    }
    printf("L_size:%d R_size:%d\n", t_pointsEdgeLeft_size,
           t_pointsEdgeRight_size);

    ////////////////////////////////////////////////////////////////////////
    // 直道判断
    ///////////////////////////////////////////////////////////////////////

    line_straight_detection();
    if (is_left_straight)
        cout << "is left straight" << endl;
    if (is_right_straight)
        cout << "is right straight" << endl;
    find_corners();




}

void Standard::findline_lefthand_adaptive(cv::Mat &img, int block_size,
                                          int clip_value, int x, int y,
                                          vector<POINT> &pointsEdgeLeft,
                                          int &pointsEdgeLeft_size)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;

    while ((step < POINTS_MAX_LEN) && half < x && x < (img.cols - half - 1) &&
           half < y && y < (img.rows - half - 1) && turn < 4)
    {
        //[0]自适应二值化
        // int local_thres = 0;
        // for(int dy = -half; dy < half; dy++)
        // {
        //     for(int dx = -half; dx <= half; dx++)
        //     {
        //         local_thres += img.at<char>(y + dy, x +
        //         dx);//将周围7*7=49个像素点的灰度值累加
        //     }
        // }
        // local_thres /= block_size * block_size;//求均值
        // local_thres -= clip_value;//减去经验值

        int local_thres = 128;

        //[1]迷宫寻线
        int current_value = img.at<char>(y, x); // 当前（x，y）上的灰度值
        int front_value = img.at<char>(y + dir_front[dir][1] /*-1，即向上一点*/,
                                       x + dir_front[dir][0] /*0*/);
        int frontleft_value =
            img.at<char>(y + dir_frontleft[dir][1] /*-1，即向上向左找一个*/,
                         x + dir_frontleft[dir][0] /*-1*/);
        if (front_value < local_thres /*表示上一点为”墙“，此时需要转向*/)
        {
            dir = (dir + 1) % 4; // 转向，更新dir
            turn++;
        }
        else if (
            frontleft_value <
            local_thres /*上方有路，但是上左是墙，此时可以向上一格，即找到左边线一点*/)
        {
            x += dir_front[dir][0]; // 更新前进后的坐标
            y += dir_front[dir][1];
            pointsEdgeLeft.emplace_back(x, y); // 保存到ipts边线点集中
            step++;                            // 此时步数加一，即找到一点，表示边线长度
            turn = 0;                          // 将turn状态清零，即还可以继续搜线
        }
        else
        {                               /*上方有路，同时上左也有路，即墙角，此时可以运动到上左一格，即找到左边线一点*/
            x += dir_frontleft[dir][0]; // 更新位于上左一格时的坐标
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pointsEdgeLeft.emplace_back(x, y); // 保存到ipts边线点集中
            step++;
            turn = 0;
        }
    }
    pointsEdgeLeft_size = step; // 边线长度
}

void Standard::findline_righthand_adaptive(cv::Mat &img, int block_size,
                                           int clip_value, int x, int y,
                                           vector<POINT> &pointsEdgeRight,
                                           int &pointsEdgeRight_size)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;

    while ((step < POINTS_MAX_LEN) && 0 < x && x < (img.cols - 1) && 0 < y &&
           y < (img.rows - 1) && turn < 4)
    {
        //[0]自适应二值化
        // int local_thres = 0;
        // for(int dy = -half; dy < half; dy++)
        // {
        //     for(int dx = -half; dx <= half; dx++)
        //     {
        //         local_thres += img.at<char>(y + dy, x +
        //         dx);//将周围7*7=49个像素点的灰度值累加
        //     }
        // }
        // local_thres /= block_size * block_size;//求均值
        // local_thres -= general.clip_value;//减去经验值
        int local_thres = 128;

        //[1]迷宫寻线
        int current_value = img.at<char>(y, x); // 当前（x，y）上的灰度值
        int front_value = img.at<char>(y + dir_front[dir][1] /*-1，即向上一点*/,
                                       x + dir_front[dir][0] /*0*/);
        int frontright_value =
            img.at<char>(y + dir_frontright[dir][1] /*-1，即向上向左找一个*/,
                         x + dir_frontright[dir][0] /*-1*/);
        if (front_value < local_thres /*表示上一点为”墙“，此时需要转向*/)
        {
            dir = (dir + 3) % 4; // 转向，更新dir
            turn++;
        }
        else if (
            frontright_value <
            local_thres /*上方有路，但是上左是墙，此时可以向上一格，即找到左边线一点*/)
        {
            x += dir_front[dir][0]; // 更新前进后的坐标
            y += dir_front[dir][1];
            pointsEdgeRight.emplace_back(x, y); // 保存到ipts边线点集中

            step++;   // 此时步数加一，即找到一点，表示边线长度
            turn = 0; // 将turn状态清零，即还可以继续搜线
        }
        else
        {                                /*上方有路，同时上左也有路，即墙角，此时可以运动到上左一格，即找到左边线一点*/
            x += dir_frontright[dir][0]; // 更新位于上左一格时的坐标
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pointsEdgeRight.emplace_back(x, y);
            step++;
            turn = 0;
        }
    }
    pointsEdgeRight_size = step; // 边线长度
}


/******************************************************************************
 * Description    : 直道判断
 *******************************************************************************/
void Standard::line_straight_detection()
{

    if (t_pointsEdgeLeft_size > 30)
    {
        std::vector<cv::Point> points;
        cv::Vec4f line_para;
        float k, b, mea = 0.0f;
        int trans[2];
        int y_counter = 0;
        for (int i = 0; i < t_pointsEdgeLeft_size && i < _config.aim_distance_far/SAMPLE_DIST; i++, y_counter++)
        {
            general.Reverse_transf(trans[0], trans[1], t_pointsEdgeLeft[i].x,
                           t_pointsEdgeLeft[i].y);
            points.push_back(cv::Point(trans[0], trans[1]));
        }

        cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        k = line_para[1] / line_para[0];
        b = line_para[3] - k * line_para[2];

        for (int i = 0; i < y_counter; i++)
            mea += fabs(k * points[i].x + b - points[i].y);

        if (mea < 50.f && t_pointsEdgeLeft_size > 30)
            is_left_straight = true;
        else if(mea > 200.f)
            is_left_curve = true;
    }

    if (t_pointsEdgeRight_size > 30)
    {
        std::vector<cv::Point> points;
        cv::Vec4f line_para;
        float k, b, mea = 0.0f;
        int trans[2];
        int y_counter = 0;
        for (int i = 0; i < t_pointsEdgeRight_size && i < _config.aim_distance_far/SAMPLE_DIST; i++, y_counter++)
        {
            general.Reverse_transf(trans[0], trans[1], t_pointsEdgeRight[i].x,
                           t_pointsEdgeRight[i].y);
            points.push_back(cv::Point(trans[0], trans[1]));
        }

        cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        k = line_para[1] / line_para[0];
        b = line_para[3] - k * line_para[2];

        for (int i = 0; i < y_counter; i++)
            mea += fabs(k * points[i].x + b - points[i].y);

        printf("right_mea %f\n",mea);

        if (mea < 50.f && t_pointsEdgeRight_size > 30)
            is_right_straight = true;
        else if(mea > 200.f)
            is_right_curve = true;
    }
}

/******************************************************************************
 * Description    : 找拐点
 *******************************************************************************/
void Standard::find_corners()
{
    if (!is_left_straight && t_pointsEdgeLeft_size > 20)
    {
        for (int i = 0; i < n_a_t_pointsEdgeLeft_size; i++)
        {
            int im1 = general.clip(i - 2, 0, n_a_t_pointsEdgeLeft_size - 1);
            int ip1 = general.clip(i + 2, 0, n_a_t_pointsEdgeLeft_size - 1);
            float conf = fabs(n_a_t_pointsEdgeLeft[i].angle) -
                         (fabs(n_a_t_pointsEdgeLeft[im1].angle +
                               fabs(n_a_t_pointsEdgeLeft[ip1].angle))) /
                             2;

            conf = conf * 180 / M_PI;
            conf = fabs(conf);
            

            if (is_t_L_pointLeft_find == false && Lconf_Min < conf &&
                conf < Lconf_Max)
            {
                for (int j = 0; j < t_pointsEdgeLeft_size; j++)
                {
                    if (t_pointsEdgeLeft[j].x == n_a_t_pointsEdgeLeft[i].x &&
                        t_pointsEdgeLeft[j].y == n_a_t_pointsEdgeLeft[i].y)
                    {
                        t_L_pointLeft_id = j;
                        is_t_L_pointLeft_find = true;
                        t_L_pointLeft.x = t_pointsEdgeLeft[j].x;
                        t_L_pointLeft.y = t_pointsEdgeLeft[j].y;
                        break;
                    }
                }
            }
            else if (is_t_L_pointLeft_find == true)
            {
                break;
            }
        }
    }

    if (!is_right_straight && t_pointsEdgeRight_size > 20)
    {
        for (int i = 0; i < n_a_t_pointsEdgeRight_size; i++)
        {
            int im1 = general.clip(i - 2, 0, n_a_t_pointsEdgeRight_size - 1);
            int ip1 = general.clip(i + 2, 0, n_a_t_pointsEdgeRight_size - 1);
            float conf = fabs(n_a_t_pointsEdgeRight[i].angle) -
                         (fabs(n_a_t_pointsEdgeRight[im1].angle +
                               fabs(n_a_t_pointsEdgeRight[ip1].angle))) /
                             2;

            
            conf = conf * 180 / M_PI;
            conf = fabs(conf);

            if (is_t_L_pointRight_find == false && Lconf_Min < conf &&
                conf < Lconf_Max)
            {
                for (int j = 0; j < t_pointsEdgeRight_size; j++)
                {
                    if (t_pointsEdgeRight[j].x == n_a_t_pointsEdgeRight[i].x &&
                        t_pointsEdgeRight[j].y == n_a_t_pointsEdgeRight[i].y)
                    {
                        t_L_pointRight_id = j;
                        is_t_L_pointRight_find = true;
                        t_L_pointRight.x = t_pointsEdgeRight[j].x;
                        t_L_pointRight.y = t_pointsEdgeRight[j].y;
                        break;
                    }
                }
            }
            else if (is_t_L_pointRight_find == true)
            {
                break;
            }
        }
    }
}

/******************************************************************************
 * Description    : 拟合中线
 ********************************************************************************/
void Standard::fitting()
{

    t_left_CenterEdge_size = 0;
    t_left_CenterEdge.clear();
    t_right_CenterEdge_size = 0;
    t_right_CenterEdge.clear();
    t_both_CenterEdge.clear();
    t_both_CenterEdge_size = 0;
    AI_CenterEdge.clear();
    AI_CenterEdge_size = 0;
    AI_t_CenterEdge.clear();
    AI_t_CenterEdge_size = 0;
    CenterEdge.clear();
    CenterEdge_size = 0;
    t_CenterEdge.clear();
    t_CenterEdge_size = 0;

    centerCompute(t_pointsEdgeLeft, t_pointsEdgeLeft_size, 0);
    t_left_CenterEdge_size = t_left_CenterEdge.size();

    centerCompute(t_pointsEdgeRight, t_pointsEdgeRight_size, 1);
    t_right_CenterEdge_size = t_right_CenterEdge.size();

    if (t_pointsEdgeLeft_size > 20 && t_pointsEdgeRight_size > 20)
        track_both_edge();

    for (int i = 0; i < t_both_CenterEdge_size; i++)
    {
        int a, b;
        general.Reverse_transf(a, b, t_both_CenterEdge[i].x, t_both_CenterEdge[i].y);
        AI_CenterEdge.emplace_back(a, b);
    }
    AI_CenterEdge_size = AI_CenterEdge.size();
}

/******************************************************************************
 * Description    : 滤波
 *******************************************************************************/
void Standard::blur_points(int side, int kernel)
{
    int half = kernel / 2;
    if (side == 0)
    {
        for (int i = 0; i < t_pointsEdgeLeft_size; i++)
        {
            b_t_pointsEdgeLeft.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                b_t_pointsEdgeLeft[i].x +=
                    t_pointsEdgeLeft[general.clip(i + j, 0,
                                                  t_pointsEdgeLeft_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                b_t_pointsEdgeLeft[i].y +=
                    t_pointsEdgeLeft[general.clip(i + j, 0,
                                                  t_pointsEdgeLeft_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            b_t_pointsEdgeLeft[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_pointsEdgeLeft[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else if (side == 1)
    {
        for (int i = 0; i < t_pointsEdgeRight_size; i++)
        {
            b_t_pointsEdgeRight.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                b_t_pointsEdgeRight[i].x +=
                    t_pointsEdgeRight[general.clip(i + j, 0,
                                                   t_pointsEdgeRight_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                b_t_pointsEdgeRight[i].y +=
                    t_pointsEdgeRight[general.clip(i + j, 0,
                                                   t_pointsEdgeRight_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            b_t_pointsEdgeRight[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_pointsEdgeRight[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    b_t_pointsEdgeLeft_size = b_t_pointsEdgeLeft.size();
    b_t_pointsEdgeRight_size = b_t_pointsEdgeRight.size();
}

/******************************************************************************
 * Description    : 等距采样
 *******************************************************************************/
void Standard::resample_points(vector<POINT> &in, int in_size,
                               vector<POINT> &out, int &out_size, float dist)
{
    int remain = 0;
    int len = 0;
    for (int i = 0; i < (in_size - 1); i++)
    {
        float x0 = in[i].x;
        float y0 = in[i].y;
        float dx = in[i + 1].x - x0;
        float dy = in[i + 1].y - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn)
        {
            x0 += dx * remain;
            y0 += dy * remain;
            out.emplace_back(x0, y0);

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    out_size = len;
}

/******************************************************************************
 * Description    : 计算角度
 *******************************************************************************/
void Standard::local_angle_points(vector<POINT> pointsEdgeIn, int size,
                                  vector<POINT> &pointsEdgeOut, int dist)
{
    for (int i = 0; i < size; i++)
    {
        pointsEdgeOut.emplace_back(pointsEdgeIn[i].x, pointsEdgeIn[i].y);
        if (i <= 0 || i >= size - 1)
        {
            pointsEdgeOut[i].angle = 0;
            continue;
        }
        float dx1 = pointsEdgeIn[i].x -
                    pointsEdgeIn[general.clip(i - dist, 0, size - 1)].x;
        float dy1 = pointsEdgeIn[i].y -
                    pointsEdgeIn[general.clip(i - dist, 0, size - 1)].y;
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pointsEdgeIn[general.clip(i + dist, 0, size - 1)].x -
                    pointsEdgeIn[i].x;
        float dy2 = pointsEdgeIn[general.clip(i + dist, 0, size - 1)].y -
                    pointsEdgeIn[i].y;
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        pointsEdgeOut[i].angle = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

/******************************************************************************
 * Description    : 角度变化量
 *非极大抑制（NMS）在一个范围内保留其最大值，即局部最大值
 *******************************************************************************/
void Standard::nms_angle(vector<POINT> &in, int in_size, vector<POINT> &out,
                         int kernel)
{

    int half = kernel / 2;
    for (int i = 0; i < in_size; i++)
    {
        out.emplace_back(in[i].x, in[i].y);
        out[i].angle = in[i].angle;
        for (int j = -half; j <= half; j++)
        {
            if (fabs(in[general.clip(i + j, 0, in_size - 1)].angle) >
                fabs(out[i].angle))
            {
                out[i].angle = 0;
                break;
            }
        }
    }
}

/**
 * @brief 巡单边生成中线
 *
 * @param pointsEdge 赛道边缘点集
 * @param PointsEdgeIn_size 单边长度
 */
void Standard::centerCompute(vector<POINT> pointsEdge, int size, int side)
{
    t_left_CenterEdge.clear();
    t_right_CenterEdge.clear();
    if (side == 0)
    {
        for (int i = 0; i < size - 10; i++)
        {
            float dx = pointsEdge[general.clip(i + approx_num, 0, size - 1)].x -
                       pointsEdge[general.clip(i - approx_num, 0, size - 1)].x;
            float dy = pointsEdge[general.clip(i + approx_num, 0, size - 1)].y -
                       pointsEdge[general.clip(i - approx_num, 0, size - 1)].y;
            float dn = sqrt(dx * dx + dy * dy);
            dx /= dn;
            dy /= dn;
            t_left_CenterEdge.emplace_back(pointsEdge[i].x - dy * dist,
                                           pointsEdge[i].y + dx * dist);
        }
    }
    else if (side == 1)
    {
        for (int i = 0; i < size - 10; i++)
        {
            float dx = pointsEdge[general.clip(i + approx_num, 0, size - 1)].x -
                       pointsEdge[general.clip(i - approx_num, 0, size - 1)].x;
            float dy = pointsEdge[general.clip(i + approx_num, 0, size - 1)].y -
                       pointsEdge[general.clip(i - approx_num, 0, size - 1)].y;
            float dn = sqrt(dx * dx + dy * dy);
            dx /= dn;
            dy /= dn;
            t_right_CenterEdge.emplace_back(pointsEdge[i].x + dy * dist,
                                            pointsEdge[i].y - dx * dist);
        }
    }
}

/**
 * @brief 巡双边生成中线
 * @param track 图像处理类
 */
void Standard::track_both_edge()
{
    t_both_CenterEdge.clear();
    vector<POINT> v_center(4);
    v_center[0] = {(t_pointsEdgeLeft[0].x + t_pointsEdgeRight[0].x) / 2,
                   (t_pointsEdgeLeft[0].y + t_pointsEdgeRight[0].y) / 2};

    v_center[1] = {(t_pointsEdgeLeft[t_pointsEdgeLeft_size / 3].x +
                    t_pointsEdgeRight[t_pointsEdgeRight_size / 3].x) /
                       2,
                   (t_pointsEdgeLeft[t_pointsEdgeLeft_size / 3].y +
                    t_pointsEdgeRight[t_pointsEdgeRight_size / 3].y) /
                       2};

    v_center[2] = {(t_pointsEdgeLeft[t_pointsEdgeLeft_size * 2 / 3].x +
                    t_pointsEdgeRight[t_pointsEdgeRight_size * 2 / 3].x) /
                       2,
                   (t_pointsEdgeLeft[t_pointsEdgeLeft_size * 2 / 3].y +
                    t_pointsEdgeRight[t_pointsEdgeRight_size * 2 / 3].y) /
                       2};

    v_center[3] = {(t_pointsEdgeLeft[t_pointsEdgeLeft_size - 1].x +
                    t_pointsEdgeRight[t_pointsEdgeRight_size - 1].x) /
                       2,
                   (t_pointsEdgeLeft[t_pointsEdgeLeft_size - 1].y +
                    t_pointsEdgeRight[t_pointsEdgeRight_size - 1].y) /
                       2};

    t_both_CenterEdge = general.Bezier(0.01, v_center);
    t_both_CenterEdge_size = t_both_CenterEdge.size();
}

/**
 * @brief 动态预瞄点计算
 */
double Standard::DynamicAimDisCal()
{
    vector<PointsCurve> CurveEdge;
    vector<Point2d> d_t_CenterEdge;
    for(int i = 0;i< t_CenterEdge_size;i++)
    {
        d_t_CenterEdge.emplace_back(Point2d(t_CenterEdge[i].x,t_CenterEdge[i].y));
    }
    vector<double> t(d_t_CenterEdge.size());
    for (size_t i = 0; i < d_t_CenterEdge.size(); i++) {
        t[i] = i; // 参数化
    }
    for(int i = 1;i < t_CenterEdge_size - 1; i ++)
    {
        PointsCurve sp;
        sp.x = d_t_CenterEdge[i].x;
        sp.y = d_t_CenterEdge[i].y;
        
        // 使用中心差分计算一阶导数
        double dt1 = t[i] - t[i-1];
        double dt2 = t[i+1] - t[i];
        
        sp.dx = ((d_t_CenterEdge[i].x - d_t_CenterEdge[i-1].x) / dt1 + 
                    (d_t_CenterEdge[i+1].x - d_t_CenterEdge[i].x) / dt2) / 2.0;
        sp.dy = ((d_t_CenterEdge[i].y - d_t_CenterEdge[i-1].y) / dt1 + 
                    (d_t_CenterEdge[i+1].y - d_t_CenterEdge[i].y) / dt2) / 2.0;
        
        // 计算二阶导数
        sp.ddx = (d_t_CenterEdge[i+1].x - 2*d_t_CenterEdge[i].x + d_t_CenterEdge[i-1].x);
        sp.ddy = (d_t_CenterEdge[i+1].y - 2*d_t_CenterEdge[i].y + d_t_CenterEdge[i-1].y);
        
        // 计算曲率 κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        double numerator = abs(sp.dx * sp.ddy - sp.dy * sp.ddx);
        double denominator = pow(sp.dx * sp.dx + sp.dy * sp.dy, 1.5);
        
        sp.curve = (denominator > 1e-10) ? numerator / denominator : 0.0;
        
        CurveEdge.push_back(sp);   
    }
    if (!CurveEdge.empty()) {
        double sum = 0.0;
        double maxCurvature = 0.0;
        double maxCurvatureIndex = 0;
        
        for (size_t i = 0; i < CurveEdge.size(); i++) {
            sum += CurveEdge[i].curve;
            if (CurveEdge[i].curve > maxCurvature) {
                maxCurvature = CurveEdge[i].curve;
                maxCurvatureIndex = i;
            }
        }
        
        double meanCurvature = sum / CurveEdge.size();
        printf("最大曲率 %f n\n",maxCurvature);
        
    }
    return 0.0;

}