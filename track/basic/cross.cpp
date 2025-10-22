/*************************************/
// Author 胡城玮
/*************************************/


#include "cross.h"

Cross::Cross()
{
    both_L_find_counter = 0;
}

Cross::Cross(Config _config)
{
}

void Cross::Cross_Check(bool is_L_left_found, bool is_L_right_found, cv::Mat imgBinary, Point t_L_pointLeft, Point t_L_pointRight,int t_L_pointLeft_id, int t_L_pointRight_id,int t_pointsEdgeLeft_size,int t_pointsEdgeRight_size)
{
    if (flag_cross == Cross_None &&
        ((is_L_left_found && is_L_right_found && t_L_pointLeft_id < t_pointsEdgeLeft_size - 5 && t_L_pointRight_id < t_pointsEdgeRight_size - 5) ||
         (is_L_left_found && t_L_pointLeft_id < 0.70 / SAMPLE_DIST && t_pointsEdgeRight_size < 0.30/SAMPLE_DIST) ||
         (is_L_right_found && t_L_pointRight_id < 0.70 / SAMPLE_DIST && t_pointsEdgeLeft_size < 0.30 / SAMPLE_DIST))) {
        Cross_counter++;

    }
    if (flag_cross == Cross_None && Cross_counter > 1 &&
        ((is_L_left_found && is_L_right_found && t_L_pointLeft_id < t_pointsEdgeLeft_size - 5 && t_L_pointRight_id < t_pointsEdgeRight_size - 5) ||
         (is_L_left_found && t_L_pointLeft_id < 0.70 / SAMPLE_DIST && t_pointsEdgeRight_size < 0.30/SAMPLE_DIST) ||
         (is_L_right_found && t_L_pointRight_id < 0.70 / SAMPLE_DIST && t_pointsEdgeLeft_size < 0.30 / SAMPLE_DIST))) {
        
        flag_cross = Cross_Begin;
        Cross_counter = 0;

    }

    

}

void Cross::Cross_Run(std::vector<POINT> &t_pointsEdgeLeft, std::vector<POINT> &t_pointsEdgeRight, cv::Mat img,
                      bool is_L_left_found, bool is_L_right_found, int t_L_pointLeft_id, int t_L_pointRight_id,
                      int &t_pointsEdgeLeft_size,int &t_pointsEdgeRight_size)
{
    if(!cross_debug)
    {
        Cross_counter++;
    }
    
    if ((flag_cross == Cross_Out && t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size == 0))
    {
        no_line_counter ++;
    }
    else if(flag_cross == Cross_Out && t_pointsEdgeLeft_size > 5 && t_pointsEdgeRight_size > 5 &&  no_line_counter > 3)
    {
        no_line_counter = 0;
        flag_cross = Cross_None;
        return;
    }
    else if (flag_cross == Cross_Begin && 
            Cross_counter > 5 &&
            ((!is_L_left_found && !is_L_right_found) ||
            (is_L_left_found && t_L_pointLeft_id < 0.26 / SAMPLE_DIST) ||
            (is_L_right_found && t_L_pointRight_id < 0.26 / SAMPLE_DIST) ||
            (t_pointsEdgeLeft_size < 0.10 / SAMPLE_DIST && t_pointsEdgeRight_size < 0.10 / SAMPLE_DIST) ||
            (Cross_counter > 30)))
    {
        flag_cross = Cross_Out;
        L_left_found = false;
        L_right_found = false;
        no_line_counter = 0;
    }
    // 此代码为开源代码，如购买得到请投诉卖家
    if(flag_cross == Cross_Begin)
    {

        if (is_L_left_found)
        {
            t_pointsEdgeLeft_size = t_L_pointLeft_id;
            t_pointsEdgeRight_size = t_L_pointLeft_id;
        }
        if (is_L_right_found)
        {
            t_pointsEdgeLeft_size = t_L_pointRight_id;
            t_pointsEdgeRight_size = t_L_pointRight_id;
        }
    }
    if(flag_cross == Cross_Out)
    {
        cross_find_farline(img, is_L_left_found, is_L_right_found, t_pointsEdgeLeft, t_pointsEdgeRight,t_L_pointLeft_id, t_L_pointRight_id);
        t_pointsEdgeLeft.clear();
        t_pointsEdgeRight.clear();

        for(int i = far_t_L_pointLeft_id;i < far_t_pointsEdgeLeft_size;i++)
        {
            t_pointsEdgeLeft.emplace_back(far_t_pointsEdgeLeft[i].x,far_t_pointsEdgeLeft[i].y);
        }
        t_pointsEdgeLeft_size = t_pointsEdgeLeft.size();
        for(int i = far_t_L_pointRight_id;i < far_t_pointsEdgeRight_size;i++)
        {
            t_pointsEdgeRight.emplace_back(far_t_pointsEdgeRight[i].x,far_t_pointsEdgeRight[i].y);
        }
        t_pointsEdgeRight_size = t_pointsEdgeRight.size();
    }
}


void Cross::cross_find_farline(cv::Mat &img, bool is_L_left_found, bool is_L_right_found, 
                            std::vector<POINT> t_pointsEdgeLeft, std::vector<POINT> t_pointsEdgeRight, 
                            int t_L_pointLeft_id, int t_L_pointRight_id)
{
    far_pointsEdgeLeft_size = 0;
    far_pointsEdgeRight_size = 0; // 边线长度清零
    far_pointsEdgeLeft.clear();
    far_pointsEdgeRight.clear();
    far_t_pointsEdgeLeft_size = 0;
    far_t_pointsEdgeRight_size = 0; // 边线长度清零
    far_t_pointsEdgeLeft.clear();
    far_t_pointsEdgeRight.clear();
    far_b_t_pointsEdgeLeft_size = 0;
    far_b_t_pointsEdgeRight_size = 0;
    far_b_t_pointsEdgeLeft.clear(); // 滤波后点集
    far_b_t_pointsEdgeRight.clear();
    far_s_b_t_pointsEdgeLeft_size = 0;
    far_s_b_t_pointsEdgeRight_size = 0;
    far_s_b_t_pointsEdgeLeft.clear(); // 等距采样后点集
    far_s_b_t_pointsEdgeRight.clear();
    far_a_t_pointsEdgeLeft_size = 0;
    far_a_t_pointsEdgeRight_size = 0;
    far_a_t_pointsEdgeLeft.clear(); // 等距采样后点集
    far_a_t_pointsEdgeRight.clear();
    far_n_a_t_pointsEdgeLeft_size = 0;
    far_n_a_t_pointsEdgeRight_size = 0;
    far_n_a_t_pointsEdgeLeft.clear(); // 等距采样后点集
    far_n_a_t_pointsEdgeRight.clear();
    is_far_t_L_pointLeft_find = false;
    is_far_t_L_pointRight_find = false;
    far_t_L_pointLeft_id = 0;
    far_t_L_pointRight_id = 0;
    Point L_pointLeft, L_pointRight;
    
    if (is_L_left_found) {
        general.Reverse_transf(far_left_x0,far_left_y0,
                            t_pointsEdgeLeft[t_L_pointLeft_id].x,t_pointsEdgeLeft[t_L_pointLeft_id].y);
        far_left_y0 -= 5;
        far_left_x0 -=18;
        L_left_found = true;
        
    }
    if (is_L_right_found) {
        general.Reverse_transf(far_right_x0,far_right_y0,t_pointsEdgeRight[t_L_pointRight_id].x,t_pointsEdgeRight[t_L_pointRight_id].y);
        far_right_y0 -= 5;
        far_right_x0 += 18;
        L_right_found = true;
    }
    if((far_left_x0 < 0 || far_left_x0 > 320) || (far_left_y0 < 0 || far_left_y0 > 240) || 
        (far_right_x0 < 0 || far_right_x0 > 320) || (far_right_y0 < 0 || far_right_y0 > 240))
    {
        return;
    }

    printf("far_l_x0 %d far_l_y0 %d far_r_x0 %d far_r_y0 %d\n",far_left_x0,far_left_y0,far_right_x0,far_right_y0);


    /* ***************************************************************** */
    int y0 = far_left_y0;
    int y1 = far_right_y0;
    for(;y0>0;y0--)
    {
        if(img.at<char>(y0-1,far_left_x0) < 128 && img.at<char>(y0,far_left_x0) > 128)
        {
            break;
        }
    }
    for(;y1>0;y1--)
    {
        if(img.at<char>(y1-1,far_right_x0) < 128 && img.at<char>(y1,far_right_x0) > 128)
        {
            break;
        }
    }
    if(y0 > 30 && L_left_found)
    {
        findline_lefthand_adaptive(img,block_size,clip_value,far_left_x0,y0,far_pointsEdgeLeft,far_pointsEdgeLeft_size);
    }
    else
    {
        far_pointsEdgeLeft_size = 0;
    }

    if(y1 > 30 && L_right_found)
    {
        findline_righthand_adaptive(img,block_size,clip_value,far_right_x0,y1,far_pointsEdgeRight,far_pointsEdgeRight_size);
    }
    else
    {
        far_pointsEdgeRight_size = 0;
    }

    if(far_pointsEdgeLeft_size && far_pointsEdgeRight_size)
    {
        if(far_pointsEdgeRight[far_pointsEdgeRight_size - 1].x < far_pointsEdgeLeft[0].x)
        {
            far_pointsEdgeRight_size = 0;
        }
    }


    ////////////////////////////////////////////////////////////////////////
    // 透视变换
    ////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < far_pointsEdgeLeft_size; i++)
    {
        int a, b;
        if(general.transf(a, b, far_pointsEdgeLeft[i].x, far_pointsEdgeLeft[i].y))
        {
            far_t_pointsEdgeLeft.emplace_back(a, b);
        }
        else 
            break;
    }
    far_t_pointsEdgeLeft_size = far_t_pointsEdgeLeft.size();
    for (int i = 0; i < far_pointsEdgeRight_size; i++)
    {
        int a, b;
        if(general.transf(a, b, far_pointsEdgeRight[i].x, far_pointsEdgeRight[i].y))
        {
            far_t_pointsEdgeRight.emplace_back(a, b);
        }
        else
            break;
    }
    far_t_pointsEdgeRight_size = far_t_pointsEdgeRight.size();
    printf("far_L_size:%d far_r_size:%d\n",far_t_pointsEdgeLeft_size,far_t_pointsEdgeRight_size);

    ////////////////////////////////////////////////////////////////////////
    // 边线滤波
    ////////////////////////////////////////////////////////////////////////
    blur_points(0, 11);
    blur_points(1, 11);
    far_b_t_pointsEdgeLeft_size = far_t_pointsEdgeLeft_size;
    far_b_t_pointsEdgeRight_size = far_t_pointsEdgeRight_size;
    
    ////////////////////////////////////////////////////////////////////////
    // 等距采样
    ////////////////////////////////////////////////////////////////////////
    resample_points(far_b_t_pointsEdgeLeft, far_b_t_pointsEdgeLeft_size,
                    far_s_b_t_pointsEdgeLeft, far_s_b_t_pointsEdgeLeft_size,
                    SAMPLE_DIST * pixel_per_meter);
    resample_points(far_b_t_pointsEdgeRight, far_b_t_pointsEdgeRight_size,
                    far_s_b_t_pointsEdgeRight, far_s_b_t_pointsEdgeRight_size,
                    SAMPLE_DIST * pixel_per_meter);

    ////////////////////////////////////////////////////////////////////////
    // 角度计算
    ////////////////////////////////////////////////////////////////////////
    local_angle_points(far_s_b_t_pointsEdgeLeft, far_s_b_t_pointsEdgeLeft_size,
                       far_a_t_pointsEdgeLeft, 11);
    far_a_t_pointsEdgeLeft_size = far_a_t_pointsEdgeLeft.size();
    local_angle_points(far_s_b_t_pointsEdgeRight, far_s_b_t_pointsEdgeRight_size,
                       far_a_t_pointsEdgeRight, 11);
    far_a_t_pointsEdgeRight_size = far_a_t_pointsEdgeRight.size();

    // ////////////////////////////////////////////////////////////////////////
    // // 非极大值抑制
    // ////////////////////////////////////////////////////////////////////////
    nms_angle(far_a_t_pointsEdgeLeft, far_a_t_pointsEdgeLeft_size, 
              far_n_a_t_pointsEdgeLeft,22);
    far_n_a_t_pointsEdgeLeft_size = far_n_a_t_pointsEdgeLeft.size();
    nms_angle(far_a_t_pointsEdgeRight, far_a_t_pointsEdgeRight_size,
              far_n_a_t_pointsEdgeRight, 22);
    far_n_a_t_pointsEdgeRight_size = far_n_a_t_pointsEdgeRight.size();
    // 此代码为开源代码，如购买得到请投诉卖家
    far_t_pointsEdgeLeft.clear();
    far_t_pointsEdgeRight.clear();
    far_t_pointsEdgeLeft_size = 0;
    far_t_pointsEdgeRight_size = 0;
    for (int i = 0; i < far_s_b_t_pointsEdgeLeft_size; i++)
    {
        far_t_pointsEdgeLeft.emplace_back(far_s_b_t_pointsEdgeLeft[i].x,
                                      far_s_b_t_pointsEdgeLeft[i].y);
        far_t_pointsEdgeLeft_size++;
    }
    for (int i = 0; i < far_s_b_t_pointsEdgeRight_size; i++)
    {
        far_t_pointsEdgeRight.emplace_back(far_s_b_t_pointsEdgeRight[i].x,
                                       far_s_b_t_pointsEdgeRight[i].y);
        far_t_pointsEdgeRight_size++;
    }
    far_t_L_pointLeft_id = far_t_pointsEdgeLeft_size;
    far_t_L_pointRight_id = far_a_t_pointsEdgeRight_size;
    find_corners();
    


}





void Cross::calculate_s_i(int start, int end, std::vector<POINT> &pointsEdge, float &slope_rate, float &intercept)
{
    int i, num = 0;
    int xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += pointsEdge[i].x;
        ysum += pointsEdge[i].y;
        num++;
    }

    // 计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
    }
    // 此代码为开源代码，如购买得到请投诉卖家
    /*计算斜率*/
    slope_rate = Slope_Calculate(start, end, pointsEdge); // 斜率
    intercept = y_average - slope_rate * x_average;       // 截距
}

void Cross::findline_lefthand_adaptive(cv::Mat &img, int block_size,
                                       int clip_value, int x, int y,
                                       vector<POINT> &pointsEdgeLeft,
                                       int &pointsEdgeLeft_size)
{
    int half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;

    while ((step < POINTS_MAX_LEN ) && half < x && x < (img.cols - half - 1) &&
           half < y && y < (img.rows - half - 1) && turn < 4)
    {
        //[0]自适应二值化
        int local_thres = 0;
        for(int dy = -half; dy < half; dy++)
        {
            for(int dx = -half; dx <= half; dx++)
            {
                local_thres += img.at<char>(y + dy, x +
                dx);//将周围7*7=49个像素点的灰度值累加
            }
        }
        local_thres /= block_size * block_size;//求均值
        local_thres -= clip_value;//减去经验值

        // int local_thres = 128;

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

void Cross::findline_righthand_adaptive(cv::Mat &img, int block_size,
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
        int local_thres = 0;
        for(int dy = -half; dy < half; dy++)
        {
            for(int dx = -half; dx <= half; dx++)
            {
                local_thres += img.at<char>(y + dy, x +
                dx);//将周围7*7=49个像素点的灰度值累加
            }
        }
        local_thres /= block_size * block_size;//求均值
        local_thres -= clip_value;//减去经验值
        // int local_thres = 128;

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
void Cross::blur_points(int side, int kernel)
{
    int half = kernel / 2;
    if (side == 0)
    {
        for (int i = 0; i < far_t_pointsEdgeLeft_size; i++)
        {
            far_b_t_pointsEdgeLeft.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                far_b_t_pointsEdgeLeft[i].x +=
                    far_t_pointsEdgeLeft[general.clip(i + j, 0,
                                                      far_t_pointsEdgeLeft_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                far_b_t_pointsEdgeLeft[i].y +=
                    far_t_pointsEdgeLeft[general.clip(i + j, 0,
                                                      far_t_pointsEdgeLeft_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            far_b_t_pointsEdgeLeft[i].x /= (2 * half + 2) * (half + 1) / 2;
            far_b_t_pointsEdgeLeft[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else if (side == 1)
    {
        for (int i = 0; i < far_t_pointsEdgeRight_size; i++)
        {
            far_b_t_pointsEdgeRight.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                far_b_t_pointsEdgeRight[i].x +=
                    far_t_pointsEdgeRight[general.clip(i + j, 0,
                                                       far_t_pointsEdgeRight_size - 1)]
                        .x *
                    (half + 1 - fabs(j));
                far_b_t_pointsEdgeRight[i].y +=
                    far_t_pointsEdgeRight[general.clip(i + j, 0,
                                                       far_t_pointsEdgeRight_size - 1)]
                        .y *
                    (half + 1 - fabs(j));
            }
            far_b_t_pointsEdgeRight[i].x /= (2 * half + 2) * (half + 1) / 2;
            far_b_t_pointsEdgeRight[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    far_b_t_pointsEdgeLeft_size = far_b_t_pointsEdgeLeft.size();
    far_b_t_pointsEdgeRight_size = far_b_t_pointsEdgeRight.size();
}

/******************************************************************************
 * Description    : 等距采样
 *******************************************************************************/
void Cross::resample_points(vector<POINT> &in, int in_size,
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
 * Description    : 找拐点
 *******************************************************************************/
void Cross::find_corners()
{
    if (far_t_pointsEdgeLeft_size > 20)
    {
        for (int i = 0; i < far_n_a_t_pointsEdgeLeft_size; i++)
        {
            int im1 = general.clip(i - 2, 0, far_n_a_t_pointsEdgeLeft_size - 1);
            int ip1 = general.clip(i + 2, 0, far_n_a_t_pointsEdgeLeft_size - 1);
            float conf = fabs(far_n_a_t_pointsEdgeLeft[i].angle) -
                         (fabs(far_n_a_t_pointsEdgeLeft[im1].angle +
                               fabs(far_n_a_t_pointsEdgeLeft[ip1].angle))) /
                             2;

            conf = conf * 180 / M_PI;
            conf = fabs(conf);

            if (is_far_t_L_pointLeft_find == false && Lconf_Min < conf &&
                conf < Lconf_Max)
            {
                for (int j = 0; j < far_t_pointsEdgeLeft_size; j++)
                {
                    if (far_t_pointsEdgeLeft[j].x == far_n_a_t_pointsEdgeLeft[i].x &&
                        far_t_pointsEdgeLeft[j].y == far_n_a_t_pointsEdgeLeft[i].y)
                    {
                        far_t_L_pointLeft_id = j;
                        is_far_t_L_pointLeft_find = true;
                        break;
                    }
                }
            }
            else if (is_far_t_L_pointLeft_find == true)
            {
                break;
            }
        }
    }

    if (far_t_pointsEdgeRight_size > 20)
    {
        for (int i = 0; i < far_n_a_t_pointsEdgeRight_size; i++)
        {
            int im1 = general.clip(i - 2, 0, far_n_a_t_pointsEdgeRight_size - 1);
            int ip1 = general.clip(i + 2, 0, far_n_a_t_pointsEdgeRight_size - 1);
            float conf = fabs(far_n_a_t_pointsEdgeRight[i].angle) -
                         (fabs(far_n_a_t_pointsEdgeRight[im1].angle +
                               fabs(far_n_a_t_pointsEdgeRight[ip1].angle))) /
                             2;
            conf = conf * 180 / M_PI;
            conf = fabs(conf);
            if (is_far_t_L_pointRight_find == false && Lconf_Min < conf &&
                conf < Lconf_Max)
            {
                for (int j = 0; j < far_t_pointsEdgeRight_size; j++)
                {
                    if (far_t_pointsEdgeRight[j].x == far_n_a_t_pointsEdgeRight[i].x &&
                        far_t_pointsEdgeRight[j].y == far_n_a_t_pointsEdgeRight[i].y)
                    {
                        far_t_L_pointRight_id = j;
                        is_far_t_L_pointRight_find = true;
                        break;
                    }
                }
            }
            else if (is_far_t_L_pointRight_find == true)
            {
                break;
            }
        }
    }
}

/******************************************************************************
 * Description    : 计算角度
 *******************************************************************************/
void Cross::local_angle_points(vector<POINT> pointsEdgeIn, int size,
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
void Cross::nms_angle(vector<POINT> &in, int in_size, vector<POINT> &out,
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

float Cross::Slope_Calculate(int begin, int end, vector<POINT> &pointsEdge)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += pointsEdge[i].x;
        ysum += pointsEdge[i].y;
        xysum += pointsEdge[i].x * pointsEdge[i].y;
        x2sum += pointsEdge[i].x * pointsEdge[i].x;
    }
    if ((end - begin) * x2sum - xsum * xsum) // 判断除数是否为零
    {
        result = ((end - begin) * xysum - xsum * ysum) / ((end - begin) * x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}
