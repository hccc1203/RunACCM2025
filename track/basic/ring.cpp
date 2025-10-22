/*************************************/
// Author 胡城玮
/*************************************/


#include "ring.h"

Ring::Ring()
{
}

Ring::Ring(Config config)
{
}

void Ring::reset()
{
    exiting_loseline_flag = false;
    L_left_down_found = false;
    L_left_mid_found = false;
    L_left_up_found = false;
    L_right_down_found = false;
    L_right_mid_found = false;
    L_right_up_found = false;
    L_id_left_down = 666;
    L_id_left_mid = 666;
    L_id_left_up = 666;
    L_id_right_down = 666;
    L_id_right_mid = 666;
    L_id_right_up = 666;
    pointsLeft.clear();
    pointsRight.clear();
    pointsMid.clear();
    pointsLeft_size = 0;
    pointsRight_size = 0;
    pointsMid_size = 0;
    entering_lose_line_flag = false;
    ring_inside_counter = 0;
    pre_counter = 0;
    exitingnum = 0;
    ring_entering_counter = 0;
    inside_exit_flag = 0;
    pre_entering_flag = 0;
}

void Ring::Ring_Check(Mat &imgBinary, bool is_left_straight, bool is_right_straight, bool L_left_found, bool L_right_found, int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size, bool is_L_left_found, bool is_L_right_found, int t_L_pointLeft_id, int t_L_pointRight_id,std::vector<POINT> &t_pointsEdgeLeft,std::vector<POINT> &t_pointsEdgeRight)
{
    // if (is_left_straight && L_right_found)
    // {
    //     flag_ring = Right_Ring_pre_Entering;
    // }
    // if (is_right_straight && L_left_found)
    // {
    //     flag_ring = Left_Ring_pre_Entering;
    // }
    L_left_down_found = false;
    L_left_mid_found = false;
    L_left_up_found = false;
    L_right_down_found = false;
    L_right_mid_found = false;
    L_right_up_found = false;
    L_id_left_down = 666;
    L_id_left_mid = 666;
    L_id_left_up = 666;
    L_id_right_down = 666;
    L_id_right_mid = 666;
    L_id_right_up = 666;
    pointsLeft.clear();
    pointsRight.clear();
    pointsMid.clear();
    pointsLeft_size = 0;
    pointsRight_size = 0;
    pointsMid_size = 0;
    if(t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size == 0)
    {
        return ;
    }
    ring_find_line(imgBinary, 200);
    // 此代码为开源代码，如购买得到请投诉卖家
    if (flag_ring == Ring_None)
    {
        left_no_size = 0;
        right_no_size = 0;

        for (int i = 0; i < 30; i++)
        {
            // printf("%d %d\n",pointsLeft[i].x,pointsRight[i].x);
            if (pointsLeft[i].x < 10 && pointsRight[i].x < 320)
            {
                left_no_size++; 
            }
            if (pointsRight[i].x > 315 && pointsLeft[i].x > 0)
            {
                right_no_size++;
            }

            if (left_no_size > 12 || right_no_size > 12)
            {
                break;
            }
        }
        printf("left_no_size %d\n",left_no_size);
        if (left_no_size > 12)
        {
            Point left_mid_point = find_left_mid(40);
            if (L_left_mid_found && is_right_straight && !is_left_straight && left_mid_point.y > 100)
            {
                printf("left mid x%d y%d\n",left_mid_point.x,left_mid_point.y);
                if(fabs(t_pointsEdgeRight[t_pointsEdgeRight_size-1].x - t_pointsEdgeRight[t_pointsEdgeRight_size/3*2].x) < 20 &&
                    fabs(t_pointsEdgeRight[t_pointsEdgeRight_size/3].x - t_pointsEdgeRight[0].x) < 20)
                    flag_ring = Left_Ring_pre_Entering;
                else
                    reset();
                
            }
        }
        else if (right_no_size > 12)
        {
            Point right_mid_point = find_right_mid(40); // 40
            if (L_right_mid_found && is_left_straight && !is_right_straight && right_mid_point.y>100)
            {
                printf("right mid x%d y%d\n",right_mid_point.x,right_mid_point.y);
                if(fabs(t_pointsEdgeLeft[t_pointsEdgeLeft_size-1].x - t_pointsEdgeLeft[t_pointsEdgeLeft_size/3*2].x) < 20 &&
                    fabs(t_pointsEdgeLeft[t_pointsEdgeLeft_size/3].x - t_pointsEdgeLeft[0].x) < 20)
                    flag_ring = Right_Ring_pre_Entering;
                else
                    reset();
            }
        }
    }

    if (flag_ring == Left_Ring_pre_Entering)
    {
        if (t_pointsEdgeLeft_size > 20) 
        {
            flag_ring = Left_Ring_Entering;
        }
    }
    else if (flag_ring == Left_Ring_Entering)
    {
        static int right_loseline_counter = 0;
        if(t_pointsEdgeRight_size == 0)
        {
            right_loseline_counter++;
        }
        if((t_pointsEdgeRight_size > 10 && !is_right_straight && ring_entering_counter > 0 && right_loseline_counter > 0) || ring_entering_counter > 40)
        {
            right_loseline_counter = 0;
            flag_ring = Left_Ring_Inside;
        }
    }
    else if (flag_ring == Left_Ring_Inside)
    {
        if (is_L_right_found && ring_inside_counter > 15 && t_L_pointRight_id < t_pointsEdgeRight_size - 6 )
        {
            flag_ring = Left_Ring_Exiting;
            general.Reverse_transf(exiting_x0,exiting_y0,t_pointsEdgeRight[t_L_pointRight_id].x,t_pointsEdgeRight[t_L_pointRight_id].y);
            exiting_x0 -= 10;
            exiting_y0 -= 5;
        }
    }
    else if (exiting_loseline_flag && flag_ring == Left_Ring_Exiting && t_pointsEdgeRight_size > 30 && exitingnum > 5 && is_right_straight && t_pointsEdgeRight[t_pointsEdgeRight_size - 5].x - t_pointsEdgeRight[0].x < 0 && !is_L_right_found) //&& exitingnum >15
    {
        flag_ring = Left_Ring_Finish;
    }
    else if (flag_ring == Left_Ring_Finish && t_pointsEdgeRight_size > 20 && t_pointsEdgeLeft_size > 20) // && t_pointsEdgeRight_size > 10
    {
        flag_ring = Ring_None;
        reset();
    }

    ///////////////////////////////////////////////////////////////////

    if (flag_ring == Right_Ring_pre_Entering) //&& preEntering_flag == 0
    {
        if (t_pointsEdgeRight_size > 3) //&& t_pointsEdgeLeft_size > 50
        {
            flag_ring = Right_Ring_Entering;
        }
    }
    else if (flag_ring == Right_Ring_Entering) //&& t_pointsEdgeLeft_size > 50
    {
        static int left_loseline_counter = 0;
        if(t_pointsEdgeLeft_size == 0)
        {
            left_loseline_counter++;
        }
        if((ring_entering_counter > 10 && t_pointsEdgeLeft_size > 10 && !is_left_straight && left_loseline_counter > 0) || ring_entering_counter > 40)
        {
            flag_ring = Right_Ring_Inside;
        }
    }
    else if (flag_ring == Right_Ring_Inside)
    {
        if (is_L_left_found && ring_inside_counter > 10 && t_L_pointLeft_id < t_pointsEdgeLeft_size - 6)
        {
            flag_ring = Right_Ring_Exiting;
            general.Reverse_transf(exiting_x0,exiting_y0,t_pointsEdgeLeft[t_L_pointLeft_id].x,t_pointsEdgeLeft[t_L_pointLeft_id].y);
            exiting_x0 += 10;
            exiting_y0 -= 5;
        }
    }
    else if (exiting_loseline_flag&&flag_ring == Right_Ring_Exiting && exitingnum > 5 && t_pointsEdgeLeft_size > 30 && is_left_straight && t_pointsEdgeLeft[t_pointsEdgeLeft_size - 5].x - t_pointsEdgeLeft[0].x > 0 && !is_L_left_found) //&& exitingnum >15
    {
        flag_ring = Right_Ring_Finish;
    }
    else if (flag_ring == Right_Ring_Finish && t_pointsEdgeLeft_size > 20 && t_pointsEdgeRight_size > 20) // && t_pointsEdgeRight_size > 10
    {
        flag_ring = Ring_None;
        reset();
    }
}

void Ring::Ring_Run(std::vector<POINT> &t_pointsEdgeLeft, std::vector<POINT> &t_pointsEdgeRight,
                    int &t_pointsEdgeLeft_size, int &t_pointsEdgeRight_size,
                    bool is_L_left_found, bool is_L_right_found,
                    int t_L_pointLeft_id, int t_L_pointRight_id,
                    cv::Mat imgBinary)
{
    // 此代码为开源代码，如购买得到请投诉卖家
    if (flag_ring == Left_Ring_pre_Entering)
    {
        printf("L_preEntering_2\n");
        t_pointsEdgeLeft_size = 0;
        pre_counter++;
    }
    else if (flag_ring == Left_Ring_Entering)
    {
        /****** 第一版 ******/
        // t_pointsEdgeLeft_size = 0 ;
        // printf("L_Entering\n");
        // Point enter_left_up = find_left_up();
        // Point enter_left_mid = find_left_mid(60);
        // if(L_left_up_found)
        // {
        //     // printf("L_left_up_found\n");
        // }
        // if(L_left_mid_found)
        // {
        //     // printf("L_left_mid_found\n");
        // }
        // if(L_left_up_found && !L_left_mid_found  && ring_entering_counter == 0)
        // {
        //     vector<POINT> v_center(2);
        //     ring_points.clear();
        //     v_center[0] = {t_pointsEdgeRight[0].x,t_pointsEdgeRight[0].y};
        //     v_center[1] = {enter_left_up.x,enter_left_up.y};
        //     ring_points = general.Bezier(0.01,v_center);
        //     ring_points_size = ring_points.size();
        //     for(int i = 0;i < ring_points_size;i++)
        //     {
        //         int a,b;
        //         general.transf(a,b,ring_points[i].x,ring_points[i].y);
        //         ring_points[i].x = a;
        //         ring_points[i].y = b;
        //     }
        //     t_pointsEdgeRight.clear(); //清除后存放补线
        //     ring_entering_counter++;
        //     for(int i = 0; i < ring_points_size; i++)
        //     {
        //         t_pointsEdgeRight.emplace_back(ring_points[i].x - 20,ring_points[i].y);
        //     }
        //     t_pointsEdgeRight_size = t_pointsEdgeRight.size();
        // }

        // else if(ring_entering_counter > 0)
        // {
        //     t_pointsEdgeRight.clear();
        //     for (int i = 0; i < ring_points_size; i ++)
        //     {
        //         //int py = ring_points[i].y;
        //         t_pointsEdgeRight.emplace_back(ring_points[i].x - 20, ring_points[i].y);
        //     }
        //     t_pointsEdgeRight_size = t_pointsEdgeRight.size();
        //     ring_entering_counter++;
        // }

        /****** 第二版 ******/
        // if (t_pointsEdgeLeft_size > 25 && !entering_lose_line_flag)
        // {
        //     last_inner_side.clear();
        //     for (int i = 0; i < t_pointsEdgeLeft_size; i++)
        //     {
        //         last_inner_side.emplace_back(t_pointsEdgeLeft[i].x, t_pointsEdgeRight[i].y);
        //     }
        // }
        // else
        // {
        //     entering_lose_line_flag = true;
        //     t_pointsEdgeLeft.clear();
        //     for (int i = 0; i < last_inner_side.size(); i++)
        //     {
        //         t_pointsEdgeLeft.emplace_back(last_inner_side[i].x, last_inner_side[i].y);
        //     }
        //     t_pointsEdgeLeft_size = t_pointsEdgeLeft.size();
        // }     
        // ring_entering_counter++;
        // t_pointsEdgeRight_size = 0;

        /****** 第三版 ******/
        ring_entering_counter++;
        if(t_pointsEdgeLeft_size > 25 && !entering_lose_line_flag)
        {
            t_pointsEdgeRight_size = 0;
            entering_x0 = t_pointsEdgeLeft[t_pointsEdgeLeft_size / 5].x;
            entering_y0 = t_pointsEdgeLeft[t_pointsEdgeLeft_size / 5].y;
            general.Reverse_transf(entering_x0,entering_y0,entering_x0,entering_y0);
            entering_x0 += 10;
        }
        else
        {
            entering_lose_line_flag = true;
            t_pointsEdgeLeft_size = 0;
            entering_track_far_line(imgBinary);
            t_pointsEdgeRight_size = 0;
            t_pointsEdgeRight.clear();
            for(int i = 0;i < s_b_t_far_entering_edge_size;i++)
            {
                t_pointsEdgeRight.emplace_back(s_b_t_far_entering_edge[i].x,s_b_t_far_entering_edge[i].y);
                t_pointsEdgeRight_size++;
            } 
        }

    }
    else if (flag_ring == Left_Ring_Inside)
    {
        printf("L_inside\n");

        /***************************第一版***************************/
        ring_inside_counter++;
        t_pointsEdgeLeft_size = 0;
        last_inner_side.clear();
        if(ring_inside_counter == 1)
        {
            for(int i = 0;i < t_pointsEdgeRight_size;i++)
            {
                last_inner_side.emplace_back(t_pointsEdgeRight[i].x ,t_pointsEdgeRight[i].y);
            }
            inside_ring_points_size = last_inner_side.size();
        }

        /***********第二版***********/
//         ring_inside_counter++;
//         if(is_L_right_found)
//         {
// 
//         }
//         t_pointsEdgeLeft_size = 0;
    }
    else if (flag_ring == Left_Ring_Exiting)
    {
        printf("L_exiting\n");
        /****************第一版********************/               
        // t_pointsEdgeLeft_size = 0;
        // t_pointsEdgeRight.clear();
        // t_pointsEdgeRight_size = 0;
        // for(int i = 0;i < last_inner_side.size();i++)
        // {
        //     t_pointsEdgeRight.emplace_back(last_inner_side[i].x,last_inner_side[i].y);
        // }
        // t_pointsEdgeRight_size = t_pointsEdgeRight.size();
        // exitingnum++;
        // printf("exitingnum = %d\n",exitingnum);

        /****************第二版********************/ 
        if(is_L_right_found)
        {
            general.Reverse_transf(exiting_x0,exiting_y0,t_pointsEdgeRight[t_L_pointRight_id].x,t_pointsEdgeRight[t_L_pointRight_id].y);
            exiting_x0 -= 10;
            exiting_y0 -= 5;
        }
        if(t_pointsEdgeRight_size == 0)
        {
            exiting_loseline_flag = true;
        }
        exiting_track_far_line(imgBinary);
        t_pointsEdgeLeft_size = 0;
        t_pointsEdgeRight_size = 0;
        t_pointsEdgeRight.clear();
        // 此代码为开源代码，如购买得到请投诉卖家
        if(s_b_t_far_exiting_edge_size > 10)
        {
            if(s_b_t_far_exiting_edge[s_b_t_far_exiting_edge_size - 5].y - s_b_t_far_exiting_edge[0].y <= 0 && s_b_t_far_exiting_edge[s_b_t_far_exiting_edge_size - 5].x - s_b_t_far_exiting_edge[0].x < 0)
            {
                for(int i = 0;i < inside_ring_points_size;i++)
                {
                    t_pointsEdgeRight.emplace_back(last_inner_side[i].x,last_inner_side[i].y);
                    t_pointsEdgeRight_size++;
                }
            }
            else
            {
                for(int i = 0;i < inside_ring_points_size;i++)
                {
                    t_pointsEdgeRight.emplace_back(last_inner_side[i].x,last_inner_side[i].y);
                    t_pointsEdgeRight_size++;
                }
            }
        }
        else
        {
            for(int i = 0;i < s_b_t_far_entering_edge_size;i++)
            {
                t_pointsEdgeRight.emplace_back(s_b_t_far_entering_edge[i].x,s_b_t_far_entering_edge[i].y);
                t_pointsEdgeRight_size++;
            }
        }
        exitingnum++;
        
    }
    else if (flag_ring == Left_Ring_Finish)
    {
        // printf("L_finish\n");
        t_pointsEdgeLeft_size = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (flag_ring == Right_Ring_pre_Entering)
    {
        // printf("R_preEntering_2\n");
        t_pointsEdgeRight_size = 0;
        pre_counter++;
    }
    else if (flag_ring == Right_Ring_Entering)
    {
        ring_entering_counter++;
        if(t_pointsEdgeRight_size > 25 && !entering_lose_line_flag)
        {
            t_pointsEdgeLeft_size = 0;
            entering_x0 = t_pointsEdgeRight[t_pointsEdgeRight_size / 5].x;
            entering_y0 = t_pointsEdgeRight[t_pointsEdgeRight_size / 5].y;
            general.Reverse_transf(entering_x0,entering_y0,entering_x0,entering_y0);
            entering_x0 -= 30;
        }
        else
        {
            entering_lose_line_flag = true;
            t_pointsEdgeRight_size = 0;
            entering_track_far_line(imgBinary);
            t_pointsEdgeLeft_size = 0;
            t_pointsEdgeLeft.clear();
            for(int i = 0;i < s_b_t_far_entering_edge_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(s_b_t_far_entering_edge[i].x,s_b_t_far_entering_edge[i].y);
                t_pointsEdgeLeft_size++;
            } 
        }

    }
    else if (flag_ring == Right_Ring_Inside)
    {
        // printf("R_inside\n");
        /*******************第一版*************** */
        ring_inside_counter++;
        t_pointsEdgeRight_size = 0;
        last_inner_side.clear();
        if(ring_inside_counter == 1)
        {
            for (int i = 0; i < t_pointsEdgeLeft_size; i++)
            {
                last_inner_side.emplace_back(t_pointsEdgeLeft[i].x, t_pointsEdgeLeft[i].y);
            }
             inside_ring_points_size = last_inner_side.size();
        }
        /*******************第二版 *******************************/
        // ring_inside_counter++;
        // if(is_L_left_found)
        // {

        // }
        // t_pointsEdgeRight_size = 0;
    }
    else if (flag_ring == Right_Ring_Exiting)
    {
        // 此代码为开源代码，如购买得到请投诉卖家
        if(is_L_left_found)
        {
            general.Reverse_transf(exiting_x0,exiting_y0,t_pointsEdgeLeft[t_L_pointLeft_id].x,t_pointsEdgeLeft[t_L_pointLeft_id].y);
            exiting_x0 += 10;
            exiting_y0 -= 5;
        }
        if(t_pointsEdgeLeft_size == 0)
        {
            exiting_loseline_flag = true;
        }
        exiting_track_far_line(imgBinary);
        t_pointsEdgeRight_size = 0;
        t_pointsEdgeLeft_size = 0;
        t_pointsEdgeLeft.clear();
        if(s_b_t_far_exiting_edge_size > 10)
        {
            if(s_b_t_far_exiting_edge[s_b_t_far_exiting_edge_size - 5].y - s_b_t_far_exiting_edge[0].y <= 0 && s_b_t_far_exiting_edge[s_b_t_far_exiting_edge_size - 5].x - s_b_t_far_exiting_edge[0].x > 0)
            {
                for(int i = 0;i < s_b_t_far_exiting_edge_size;i++)
                {
                    t_pointsEdgeLeft.emplace_back(s_b_t_far_exiting_edge[i].x,s_b_t_far_exiting_edge[i].y);
                    t_pointsEdgeLeft_size++;
                }
            }
            else
            {
                for(int i = 0;i < inside_ring_points_size;i++)
                {
                    t_pointsEdgeLeft.emplace_back(last_inner_side[i].x,last_inner_side[i].y);
                    t_pointsEdgeLeft_size++;
                }
            }
        }
        else
        {
            for(int i = 0;i < inside_ring_points_size;i++)
            {
                t_pointsEdgeLeft.emplace_back(last_inner_side[i].x,last_inner_side[i].y);
                t_pointsEdgeLeft_size++;
            }
        }
        exitingnum++;
    }
    else if (flag_ring == Right_Ring_Finish)
    {
        t_pointsEdgeRight_size = 0;
        // printf("R_finish\n");
    }
}

void Ring::ring_find_line(Mat &img, int y_start)
{

        int step = 0;
        for (int row = rowstart; row > rowup; row--) // 遍历190行--40行
        {
            int base_x;
            if (step == 0)
            {
                base_x = 160; // 若巡线首行，以图象中点做起始点
            }
            else
            {
                base_x = pointsMid[step - 1].x; // 若非首行，以上一行中点做起点
            }
            if (img.at<char>(row, base_x) < thresOTSU)
            {
                break;
            }
            /*******************搜右线******************************/
            for (int x = base_x - 10; x < 320; x++)
            {
                if (x > 315)
                {
                    pointsRight.emplace_back(320, row);
                    break;
                }
                else if (img.at<char>(row, x + 1) < thresOTSU)
                {
                    pointsRight.emplace_back(x, row);
                    break;
                }
            }
            /*****************搜左线**********************/
            for (int x = base_x + 10; x > 0; x--)
            {

                if (x < 5)
                {
                    pointsLeft.emplace_back(0, row);
                    break;
                }
                else if (img.at<char>(row, x - 1) < thresOTSU)
                {
                    pointsLeft.emplace_back(x, row);
                    break;
                }
            }
            // 计算中心点
            pointsMid.emplace_back((pointsLeft[step].x + pointsRight[step].x) / 2, row);
            step++;
        }
    

    pointsLeft_size = pointsLeft.size();
    pointsRight_size = pointsRight.size();
    pointsMid_size = pointsMid.size();
}

Point Ring::find_left_down() // 寻左下拐点
{
    if (pointsLeft[0].x != 0 && pointsLeft[1].x != 0 && pointsLeft[3].x != 0)
    {
        for (int i = 2; i < 100; i++)
        {
            if (abs(pointsLeft[i - 2].x - pointsLeft[i - 1].x) < 5 && abs(pointsLeft[i].x - pointsLeft[i - 1].x) < 5 && pointsLeft[i].x - pointsLeft[i + 1].x > 8 && i + 1 >= 2 && pointsLeft[i + 1].x < 180 && i + 1 < 140)
            {
                L_left_down_found = true;
                L_id_left_down = i;
                // printf("\n\n\nfound!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
                Point left_down = Point(pointsLeft[i].x, pointsLeft[i].y);
                return left_down;
            }
        }
    }
    return Point(0, 0);
}
Point Ring::find_right_down() // 寻right下拐点
{
    if (pointsRight[0].x != 0 && pointsRight[1].x != 0 && pointsRight[3].x != 0)
    {
        for (int i = 2; i < 100; i++)
        {
            if (abs(pointsRight[i - 2].x - pointsRight[i - 1].x) < 8 && abs(pointsRight[i].x - pointsRight[i - 1].x) < 8 && pointsRight[i + 1].x - pointsRight[i].x > 5 && i + 1 >= 2 && pointsRight[i + 1].x > 180 && i + 1 < 140)
            {
                L_right_down_found = true;
                L_id_right_down = i;
                Point right_down = Point(pointsRight[i].x, pointsRight[i].y);
                return right_down;
            }
        }
    }
    return Point(0, 0);
}
Point Ring::find_left_mid(int start) // int start
{
    int max_index = 0;
    int no_size_counter = 0;
    for (int i = start; i < pointsLeft_size - 20; i++)
    {
        if (pointsLeft[i].x > pointsLeft[max_index].x)
        {
            max_index = i;
            no_size_counter = 0;
        }
        if(pointsLeft[i].x < pointsLeft[max_index].x - 1 && pointsLeft[i].x > pointsLeft[max_index].x - 5)
        {
            no_size_counter++;
        }
        if(no_size_counter > 4)
        {
            break;
        }
    }
    if (no_size_counter > 2 && max_index != 0 && pointsLeft[max_index].x > 30 && pointsLeft[max_index].x < 160)
    {
        L_left_mid_found = true;
        L_id_left_mid = max_index;
        Point left_mid = Point(pointsLeft[max_index].x, pointsLeft[max_index].y);
        return left_mid;
    }
    return Point(0, 0);
}
Point Ring::find_right_mid(int start)
{
    int min_index = 0;
    int no_size_counter = 0;
    
    for (int i = start; i < pointsRight_size - 20; i++)
    {
        if (pointsRight[i].x < pointsRight[min_index].x)
        {
            min_index = i;
            no_size_counter = 0;
        }
        if (pointsRight[i].x > pointsRight[min_index].x + 1 && pointsRight[i].x < pointsRight[min_index].x+5)
        {
            no_size_counter++;
        }
        if (no_size_counter > 4)
        {
            break;
        }
    }
    if (no_size_counter > 2 && min_index != 0 && pointsRight[min_index].x < 260 && pointsRight[min_index].x > 160)
    {
        L_right_mid_found = true;
        L_id_right_mid = min_index;
        Point right_mid = Point(pointsRight[min_index].x, pointsRight[min_index].y);
        return right_mid;
    }
    return Point(0, 0);
}

Point Ring::find_left_up() // 寻左上拐点
{
    for (int i = pointsLeft_size - 30; i > 20; i--)
    {

        if (abs(pointsLeft[i + 2].x - pointsLeft[i + 1].x) < 5 && abs(pointsLeft[i].x - pointsLeft[i + 1].x) < 5 && pointsLeft[i].x - pointsLeft[i - 1].x > 8 && pointsLeft[i].x < 180)
        {
            L_left_up_found = true;
            L_id_left_up = i;
            Point left_up = Point(pointsLeft[L_id_left_up].x, pointsLeft[L_id_left_up].y);
            return left_up;
        }
    }
    return Point(0, 0);
}

Point Ring::find_right_up() // 寻右上拐点
{
    for (int i = pointsRight_size - 30; i > 10; i--)
    {
        if (abs(pointsRight[i + 2].x - pointsRight[i + 1].x) < 5 && abs(pointsRight[i].x - pointsRight[i + 1].x) < 5 && pointsRight[i - 1].x - pointsRight[i].x > 8 && pointsRight[i + 1].x > 160)
        {
            L_right_up_found = true;
            L_id_right_up = i;
            Point right_up = Point(pointsRight[L_id_right_up].x, pointsRight[L_id_right_up].y);
            return right_up;
        }
    }
    return Point(0, 0);
}

void Ring::findline_lefthand_adaptive(cv::Mat &img, int block_size,
                                      int clip_value, int x, int y,
                                      vector<POINT> &pointsEdgeLeft,
                                      int &pointsEdgeLeft_size)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;

    while ((step < POINTS_MAX_LEN/2) && half < x && x < (img.cols - half - 1) &&
           half < y && y < (img.rows - half - 1) && turn < 4)
    {


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

void Ring::findline_righthand_adaptive(cv::Mat &img, int block_size,
                                       int clip_value, int x, int y,
                                       vector<POINT> &pointsEdgeRight,
                                       int &pointsEdgeRight_size)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;

    while ((step < POINTS_MAX_LEN/2) && 0 < x && x < (img.cols - 1) && 0 < y &&
           y < (img.rows - 1) && turn < 4)
    {

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
 * Description    : 滤波
 *******************************************************************************/
void Ring::blur_points(int side, int kernel) // side为0进环远线 为1出环远线
{
    int half = kernel / 2;
    if (side == 0)
    {
        for (int i = 0; i < t_far_entering_edge_size; i++)
        {
            b_t_far_entering_edge.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                b_t_far_entering_edge[i].x +=
                    t_far_entering_edge[general.clip(i + j, 0,
                                                  t_far_entering_edge_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                b_t_far_entering_edge[i].y +=
                    t_far_entering_edge[general.clip(i + j, 0,
                                                  t_far_entering_edge_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            b_t_far_entering_edge[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_far_entering_edge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else if (side == 1)
    {
        for (int i = 0; i < t_far_exiting_edge_size; i++)
        {
            b_t_far_exiting_edge.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                b_t_far_exiting_edge[i].x +=
                    t_far_exiting_edge[general.clip(i + j, 0,
                                                   t_far_exiting_edge_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                b_t_far_exiting_edge[i].y +=
                    t_far_exiting_edge[general.clip(i + j, 0,
                                                   t_far_exiting_edge_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            b_t_far_exiting_edge[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_far_exiting_edge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    b_t_far_entering_edge_size = b_t_far_entering_edge.size();
    b_t_far_exiting_edge_size = b_t_far_exiting_edge.size();
}

/******************************************************************************
 * Description    : 等距采样
 *******************************************************************************/
void Ring::resample_points(vector<POINT> &in, int in_size,
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

void Ring::entering_track_far_line(cv::Mat imgBinary)
{
    far_entering_edge.clear();
    far_entering_edge_size = 0;
    t_far_entering_edge.clear();
    t_far_entering_edge_size = 0;
    b_t_far_entering_edge.clear();
    b_t_far_entering_edge_size = 0;
    s_b_t_far_entering_edge.clear();
    s_b_t_far_entering_edge_size = 0;
    int y0 = entering_y0 - 5;
    for(;y0 > 0; y0 --)
    {
        if(imgBinary.at<char>(y0 - 1,entering_x0) < 128)
        {
            break;
        }
    }
    if(imgBinary.at<char>(y0,entering_x0) > 128)
    {
        if(flag_ring == Left_Ring_Entering)
        {
            findline_righthand_adaptive(imgBinary,block_size,clip_value,entering_x0,y0,far_entering_edge,far_entering_edge_size);
        }
        else if(flag_ring == Right_Ring_Entering)
        {
            findline_lefthand_adaptive(imgBinary,block_size,clip_value,entering_x0,y0,far_entering_edge,far_entering_edge_size);
        }
    }
    else
    {
        far_entering_edge_size = 0;
    }


    ////////////////////////////////////////////////////////////////////////
    // 透视变换
    ////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < far_entering_edge_size; i++)
    {
        int a, b;
        if(general.transf(a, b, far_entering_edge[i].x, far_entering_edge[i].y))
            t_far_entering_edge.emplace_back(a, b);
        else
            break;
    }
    t_far_entering_edge_size = t_far_entering_edge.size();

    ////////////////////////////////////////////////////////////////////////
    // 边线滤波
    ////////////////////////////////////////////////////////////////////////
    blur_points(0, 11);
    b_t_far_entering_edge_size = t_far_entering_edge_size;

    ////////////////////////////////////////////////////////////////////////
    // 等距采样
    ////////////////////////////////////////////////////////////////////////
    resample_points(b_t_far_entering_edge, b_t_far_entering_edge_size,
                    s_b_t_far_entering_edge, s_b_t_far_entering_edge_size,
                    SAMPLE_DIST * pixel_per_meter);
    printf("entering_size:%d\n",s_b_t_far_entering_edge_size);
}

void Ring::exiting_track_far_line(cv::Mat imgBinary)
{
    far_exiting_edge.clear();
    far_exiting_edge_size = 0;
    t_far_exiting_edge.clear();
    t_far_exiting_edge_size = 0;
    b_t_far_exiting_edge.clear();
    b_t_far_exiting_edge_size = 0;
    s_b_t_far_exiting_edge.clear();
    s_b_t_far_exiting_edge_size = 0;
    int y0 = exiting_y0 - 5;
    for(;y0 > 0; y0 --)
    {
        if(imgBinary.at<char>(y0 - 1,exiting_x0) < 128)
        {
            break;
        }
    }
    if(imgBinary.at<char>(y0,exiting_x0) > 128)
    {
        if(flag_ring == Left_Ring_Exiting)
        {
            findline_righthand_adaptive(imgBinary,block_size,clip_value,exiting_x0,y0,far_exiting_edge,far_exiting_edge_size);
        }
        else if(flag_ring == Right_Ring_Exiting)
        {
            findline_lefthand_adaptive(imgBinary,block_size,clip_value,exiting_x0,y0,far_exiting_edge,far_exiting_edge_size);
        }
    }
    else
    {
        far_exiting_edge_size = 0;
    }


    ////////////////////////////////////////////////////////////////////////
    // 透视变换
    ////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < far_exiting_edge_size; i++)
    {
        int a, b;
        if(general.transf(a, b, far_exiting_edge[i].x, far_exiting_edge[i].y))
            t_far_exiting_edge.emplace_back(a, b);
        else
            break;
    }
    t_far_exiting_edge_size = t_far_exiting_edge.size();

    ////////////////////////////////////////////////////////////////////////
    // 边线滤波
    ////////////////////////////////////////////////////////////////////////
    blur_points(1, 11);
    b_t_far_exiting_edge_size = t_far_exiting_edge_size;

    ////////////////////////////////////////////////////////////////////////
    // 等距采样
    ////////////////////////////////////////////////////////////////////////
    resample_points(b_t_far_exiting_edge, b_t_far_exiting_edge_size,
                    s_b_t_far_exiting_edge, s_b_t_far_exiting_edge_size,
                    SAMPLE_DIST * pixel_per_meter);
    printf("exiting_size:%d\n",s_b_t_far_exiting_edge_size);

}
