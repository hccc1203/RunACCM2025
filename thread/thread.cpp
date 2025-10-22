/*************************************/
// Author 长安大学
/*************************************/

#include "./thread.h"

#include "../track/special/catering.h"

#include <signal.h>

#include <iostream>

using namespace cv;

extern Uart uart;
bool flag = false;
float pitch_angle;
General general;


/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
    flag = true;
	cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
}

bool producer(Factory<TaskData> & task_data, Factory<TaskData> & AI_task_data, Config & config) {
	Capture capture(config);
    

	while (true) {
		TaskData src;
		if (!capture.getImage(src.img)) 
        {
            printf("noimage");
            continue;
        }
        auto time_now = std::chrono::steady_clock::now();
        src.timestamp = time_now;
        // general.savePicture(src.img);
		AI_task_data.produce(src);
        task_data.produce(src);
        if(flag)
        {
            exit(0);
        }
	}
	return true;	
}


General image_writer1;

bool AIConsumer(Factory<TaskData> & AI_task_data, std::vector<PredictResult> & predict_result, Config & config) {
    shared_ptr<Detection> detection = make_shared<Detection>(config.model);

	detection->score = config.score;
	std::mutex lock;
	while (true) {
		TaskData dst;
		AI_task_data.consume(dst);
		detection->inference(dst.img);
		lock.lock();
		predict_result = detection->results;
        Mat img = dst.img.clone();
        // 此代码为开源代码
        if(flag)
        {
            exit(0);
        }

        // image_writer1.savePicture(img, 1, "_AI_RAW");

        // // 三通道分离
        // std::vector<cv::Mat> channels;
        // cv::split(img, channels);

        // // img RB 通道乘以 1.3
        // channels[2] *= 1.4; // R通道
        // channels[1] *= 1.2; // G通道
        // channels[0] *= 1.4; // B通道

        // // img RB 通道合并
        // cv::merge(channels, img);

        detection->drawBox(img);
        general.savePicture(img);
        printf("Detected\n");
        if(config.en_AI_show)
        {
            imshow("Detection Result", img);
            cv::waitKey(1);
        }
		lock.unlock();
	}
	return true;	
}

bool consumer(Factory<TaskData> & task_data, Factory<DebugData> & debug_data, std::vector<PredictResult> & predict_result, Config & config, shared_ptr<Uart> & uart) {
	// 此代码为开源代码
    Standard standard(config);
    bool stop2_flag = false;
    int stop_counter = 0;
    while (true) {
		auto time_start = std::chrono::steady_clock::now();
		
		TaskData src;
        
		DebugData debug;
		auto result = predict_result;
		task_data.consume(src);
        
        if (src.img.empty()) continue;
        
        // 执行巡线代码
        TaskData dst = standard.run(src.img, predict_result,pitch_angle);
        printf("run_finish!\n");
        
        uart->carControl(dst.speed, dst.pwm,dst.buzzer_enable); // 串口通信控制车辆
        printf("run_finish2!\n");
        
        if(standard.Stop_flag)
        {
            
            uart->carControl(0,dst.pwm,2);
            stop_counter ++;
            // standard.videoWriter.release();
            if(stop_counter > 100)
            {
                flag = true;
                stop2_flag = false;
            }
        }
        if(flag)
        {
            exit(0);
        }

		// debug
        debug.img = dst.img;
		debug.results = result;
        debug.pwm = dst.pwm;
        debug.speed = dst.speed;
		debug_data.produce(debug);



        // std::cout << standard.element_string[standard.elem_state] << std::endl;
	    
		auto time_end = std::chrono::steady_clock::now();
		auto t = std::chrono::duration<double, std::milli>(time_end - time_start).count();
		// std::cout << (int)t << "ms" << std::endl;
        signal(SIGINT, callbackSignal);

	}
	return true;
}

void drawBox(Mat &img, std::vector<PredictResult> results)
{
    // 此代码为开源代码
    for (int i = 0; i < results.size(); i++)
    {
        PredictResult result = results[i];

        auto score = std::to_string(result.score);
        int pointY = result.y - 20;
        if (pointY < 0)
            pointY = 0;
        cv::Rect rectText(result.x, pointY, result.width, 20);
        cv::rectangle(img, rectText, getCvcolor(result.type), -1);
        std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 3) + "]";
        cv::Rect rect(result.x, result.y, result.width, result.height);
        cv::rectangle(img, rect, getCvcolor(result.type), 1);
        cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
    }
}

bool debugDataConsumer(Factory<DebugData> & debug_data) {
	while (true) {
		DebugData dst;
		debug_data.consume(dst);
        if (dst.img.empty()) continue;
		// drawBox(dst.img, dst.results);
        // cv::resize(dst.img, dst.img, cv::Size(640, 480));
		cv::imshow("output", dst.img);
		cv::waitKey(1);
	}
	return true;
}

bool uartReceive(std::shared_ptr<Uart> & uart)
{
    while(true){
        // uart->receiveCheck();
        // pitch_angle = uart->pitch_angle;
        if(flag)
        {
            exit(0);
        }
    }
    return true;
}

// -------------------------------------------------------------------- //
/**
 * @brief 获取Opencv颜色
 *
 * @param index 序号
 * @return cv::Scalar
 */
cv::Scalar getCvcolor(int index)
{
    switch (index)
    {
    case 0:
        return cv::Scalar(0, 255, 0); // 绿
        break;
    case 1:
        return cv::Scalar(255, 255, 0); // 天空蓝
        break;
    case 2:
        return cv::Scalar(0, 0, 255); // 大红
        break;
    case 3:
        return cv::Scalar(0, 250, 250); // 大黄
        break;
    case 4:
        return cv::Scalar(250, 0, 250); // 粉色
        break;
    case 5:
        return cv::Scalar(0, 102, 255); // 橙黄
        break;
    case 6:
        return cv::Scalar(255, 0, 0); // 深蓝
        break;
    case 7:
        return cv::Scalar(255, 255, 255); // 大白
        break;
    case 8:
        return cv::Scalar(247, 43, 113);
        break;
    case 9:
        return cv::Scalar(40, 241, 245);
        break;
    case 10:
        return cv::Scalar(237, 226, 19);
        break;
    case 11:
        return cv::Scalar(245, 117, 233);
        break;
    case 12:
        return cv::Scalar(55, 13, 19);
        break;
    case 13:
        return cv::Scalar(255, 255, 255);
        break;
    case 14:
        return cv::Scalar(237, 226, 19);
        break;
    case 15:
        return cv::Scalar(0, 255, 0);
        break;
    default:
        return cv::Scalar(255, 0, 0);
        break;
    }
}
