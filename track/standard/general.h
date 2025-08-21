/*************************************/
// Author 胡城玮
/*************************************/


#pragma once

#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
class General{
public:

    cv::Mat rotation;
    General(){
        Point2f src_points[4];
        Point2f dst_points[4];
        dst_points[0] = Point2f(123, 105.0);
        dst_points[1] = Point2f(195.0, 104.0);
        dst_points[2] = Point2f(102.0, 135.0);
        dst_points[3] = Point2f(211.0, 134.0);
        // 期望透视变换后二维码四个角点的坐标
        src_points[0] = Point2f(110.0, 40.0);
        src_points[1] = Point2f(210.0, 40.0);
        src_points[2] = Point2f(110.0, 140.0);
        src_points[3] = Point2f(210.0, 140.0);

        // 计算透视变换矩阵
        rotation = getPerspectiveTransform(src_points, dst_points);
        // cout<<rotation<<endl;

        for(int i = 0;i < 3;i ++)
        {
            for(int j = 0;j < 3;j++)
            {
                Re_change_un_Mat[i][j] = rotation.at<double>(i,j);
            }
        }


        src_points[0] = Point2f(123, 105.0);
        src_points[1] = Point2f(195.0, 104.0);
        src_points[2] = Point2f(102.0, 135.0);
        src_points[3] = Point2f(211.0, 134.0);
        // 期望透视变换后二维码四个角点的坐标
        dst_points[0] = Point2f(110.0, 40.0);
        dst_points[1] = Point2f(210.0, 40.0);
        dst_points[2] = Point2f(110.0, 140.0);
        dst_points[3] = Point2f(210.0, 140.0);

        // 计算透视变换矩阵
        rotation = getPerspectiveTransform(src_points, dst_points);
        // cout<<rotation<<endl;

        for(int i = 0;i < 3;i ++)
        {
            for(int j = 0;j < 3;j++)
            {
                change_un_Mat[i][j] = rotation.at<double>(i,j);
            }
        }
    };

    int clip(int x, int low, int up)
    { // 确定范围
        return x > up ? up : x < low ? low
                                     : x;
    }

    void savePicture(cv::Mat &image, int delta = 1, std::string prefix = "")
    {
        // 存图
        string name = ".jpg";
        static int counter = 0;
        counter += delta;
        printf("image:%d\n", counter);
        string img_path = "/home/edgeboard/Run_ACCM_2025Demo/image/";
        name = img_path + to_string(counter) + prefix + ".jpg";
        cv::imwrite(name, image);
    }

    /**
     * @brief 阶乘计算
     *
     * @param x
     * @return int
     */
    int factorial(int x)
    {
        int f = 1;
        for (int i = 1; i <= x; i++)
        {
            f *= i;
        }
        return f;
    }

    std::vector<POINT> Bezier(double dt, std::vector<POINT> input)
    {
        vector<POINT> output;

        double t = 0;
        while (t <= 1)
        {
            POINT p;
            double x_sum = 0.0;
            double y_sum = 0.0;
            int i = 0;
            int n = input.size() - 1;
            while (i <= n)
            {
                double k =
                    factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
                x_sum += k * input[i].x;
                y_sum += k * input[i].y;
                i++;
            }
            p.x = x_sum;
            p.y = y_sum;
            output.push_back(p);
            t += dt;
        }
        return output;
    }

    double sigma(std::vector<POINT> vec,int n,int m)
    {
        if (vec.size() < 1)
            return 0;

        double sum = 0;
        for (int i = n;i<m;i++)
            sum += vec[i].x;

        double aver = (double)sum / (m-n);
        double sigma = 0;

        for(int i = n;i<m;i++)
            sigma += (vec[i].x - aver) * (vec[i].x - aver);
        sigma /= (double)(m - n);

        return sigma;
    }

    // 偏差滑动平均滤波
    float filter(float value) {
        static float filter_buf[3] = {0};

        filter_buf[2] = filter_buf[1];
        filter_buf[1] = filter_buf[0];
        filter_buf[0] = value;

        return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
    }

    // 位置式 PID 角度外环
    float pid_realize_a(float actual, float set, float _p, float _d) {
        static float last_error = 0.0f;
        static float last_out_d = 0.0f;
        static float last_actual = 0.0f;
        static float derivative = 0.0f;

        /* 当前误差 */
        float error = set - actual;

        /* 微分先行 */
        
        // float temp = 0.618f * _d + _p;
        // float c3 = _d / temp;
        // float c2 = (_d + _p) / temp;
        // float c1 = 0.618f * c3;
        // derivative = c1 * derivative + c2 * actual - c3 * last_actual;
        

        /* 不完全微分 */
        // float out_d = _d  * (error - last_error) ;
        float out_d = 0.7f *_d * (error - last_error) + 0.3f * last_out_d;

        /* 实际输出 */
        float output = _p * error + out_d;

        /* 更新参数 */
        last_error = error;
        last_out_d = out_d;
        // last_actual = actual;

        return output;
    }

    double change_un_Mat[3][3] = {{-3.614457831325295, -5.466867469879507, 744.9397590361433},
                                {-9.964747281289631e-16, -11.48343373493974, 999.0963855421669},
                                {-4.864999171266168e-18, -0.03463855421686741, 1}};







    bool transf(int &ri, int &rj, int i, int j)
    {
        int local_x = static_cast<int>((change_un_Mat[0][0] * i + change_un_Mat[0][1] * j + change_un_Mat[0][2]) / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
        int local_y = static_cast<int>((change_un_Mat[1][0] * i + change_un_Mat[1][1] * j + change_un_Mat[1][2]) / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
        if (local_x >= 0 && local_x < 320 && local_y >= 0 && local_y < 240)
        {
            ri = local_x;
            rj = local_y;
            return true;
        }
        else
        {
            return false;
        }
    }

    double Re_change_un_Mat[3][3] = {{0.5571147540983606, -0.4899672131147541, 74.50754098360656},
                                {4.174901165517516e-17, -0.08708196721311479, 87.00327868852459},
                                {1.719608593143229e-19, -0.003016393442622951, 1}};







    bool Reverse_transf(int &ri, int &rj, int i, int j)
    {
        int local_x =
            static_cast<int>((Re_change_un_Mat[0][0] * i +
                              Re_change_un_Mat[0][1] * j + Re_change_un_Mat[0][2]) /
                             (Re_change_un_Mat[2][0] * i +
                              Re_change_un_Mat[2][1] * j + Re_change_un_Mat[2][2]));
        int local_y =
            static_cast<int>((Re_change_un_Mat[1][0] * i +
                              Re_change_un_Mat[1][1] * j + Re_change_un_Mat[1][2]) /
                             (Re_change_un_Mat[2][0] * i +
                              Re_change_un_Mat[2][1] * j + Re_change_un_Mat[2][2]));
        if (local_x >= 0 && local_x < 320 && local_y >= 0 && local_y < 240)
        {
            ri = local_x;
            rj = local_y;
        }
        return true;
    }
};