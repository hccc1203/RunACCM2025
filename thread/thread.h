/*************************************/
// Author 长安大学
/*************************************/

#pragma once

#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>
#include <unistd.h>

#include "../include/detection.hpp"
#include "../param/param.hpp"
#include "../capture/capture.h"
#include "../track/standard/standard.h"
// #include "../port/canPort.hpp"
#include "../include/uart.hpp"
#include "../track/special/catering.h"

struct DebugData{
	cv::Mat img;
	cv::Mat bin_img;
	std::vector<PredictResult> results;
	int pwm;
	double speed;
};

template<typename T>
class Factory{
private:
	std::deque<T> buffer;
	int buffer_size;
	std::mutex lock;
public:
	Factory(int size) {
		buffer_size = size;	
	}
	bool produce(T &product);
	bool consume(T &product);
};

template<typename T>
bool Factory<T>::produce(T &product) {
	lock.lock();
	if (buffer.size() < buffer_size) {
		buffer.push_back(product);
	} else {
		buffer.pop_front();
		buffer.push_back(product);
	}
	lock.unlock();

	return true;
}

template<typename T>
bool Factory<T>::consume(T &product) {
	while (true) {
		lock.lock();
		if(!buffer.empty()) break;
		lock.unlock();
		usleep(50);
	}
	product = buffer.front();
	buffer.pop_front();
	lock.unlock();

	return true;
}
// ------------------------------------ //

bool producer(Factory<TaskData> & task_data, Factory<TaskData> & AI_task_data, Config & config);
bool AIConsumer(Factory<TaskData> & task_data, std::vector<PredictResult> & predict_result, Config & config);
bool consumer(Factory<TaskData> & task_data, Factory<DebugData> & debug_data, std::vector<PredictResult> & predict_result, Config & config, std::shared_ptr<Uart> & uart);
void drawBox(Mat &img, std::vector<PredictResult> results);
bool debugDataConsumer(Factory<DebugData> & debug_data);
bool uartReceive(std::shared_ptr<Uart> & uart);

cv::Scalar getCvcolor(int index);
