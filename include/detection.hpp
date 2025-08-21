#pragma once

#include <onnxruntime_cxx_api.h>
#include <stdlib.h>
#include <sys/time.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common.hpp"
#include "predictor_api.h"

enum PredictLabel {
    battery,
    block,
    bridge,
    burger,
    car,
    company,
    cone,
    crosswalk,
    pedestrian,
    school,
};

/**
 * @brief 目标检测结果
 *
 */
struct PredictResult {
    int type; // ID
    std::string label; // 标签
    float score;       // 置信度
    int x;             // 坐标
    int y;             // 坐标
    int width;         // 尺寸
    int height;        // 尺寸
};

class Detection {
  public:
    std::vector<PredictResult> results; // AI推理结果
    float score = 0.8;                  // AI检测置信度

    /**
     * @brief Construct a new Detection object
     *
     * @param pathModel
     */
    Detection(const std::string pathModel) {
        // 模型初始化
        this->predictor_nna_ = std::make_shared<PPNCPredictor>(
            "../param/config_ppncnna.json");
        this->predictor_nms_ = std::make_shared<PPNCPredictor>(
            "../param/config_ppncnms.json");
        this->onnx_env_ =
            Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "test");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(8);
        session_options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        std::string onnx_model = pathModel + "/post.onnx";
        this->predictor_onnx_ = std::make_shared<Ort::Session>(
            this->onnx_env_, onnx_model.c_str(), session_options);

        // ONNX模型加载
        this->onnx_input_names_.first.push_back("im_shape");
        this->onnx_input_names_.first.push_back("scale_factor");
        std::string io_paddle = pathModel + "/io_paddle.json";
        // read io_paddle
        std::ifstream ifs(io_paddle);
        nlohmann::json j;
        ifs >> j;
        ifs.close();
        for (size_t i = 0; i < j.size(); ++i) {
            if (j[i]["type"] == "OUTPUT") {
                assert(j[i]["shape"].size() == 4);
                this->onnx_input_names_.first.push_back(j[i]["name"]);
            }
            if (j[i]["type"] == "post_out") {
                this->onnx_out_names_.first.push_back(j[i]["name"]);
            }
        }

        for (auto &s : this->onnx_input_names_.first) {
            this->onnx_input_names_.second.push_back(s.c_str());
        }

        for (auto &s : this->onnx_out_names_.first) {
            this->onnx_out_names_.second.push_back(s.c_str());
        }
        buildNms(pathModel); // 编译生成.so文件

        this->predictor_nna_->load();
        this->predictor_nms_->load();

        // 模型标签加载
        std::string pathLabels = pathModel + "/label_list.txt";
        labels.clear();
        std::ifstream file(pathLabels);
        if (file.is_open()) {
            std::string line;
            while (getline(file, line)) {
                labels.push_back(line);
            }
            file.close();
        } else {
            std::cout << "Open Lable File failed: " << pathLabels << std::endl;
        }
    };

    /**
     * @brief AI模型推理
     *
     */
    void inference(cv::Mat img) {
        auto feeds = preprocess(img, {320, 320}); // 图像前处理
        run(*feeds);                              // 模型推理
        render();                                 // 后处理
    }

    /**
     * @brief
     *
     * @param model_dir
     */
    void buildNms(const std::string &model_dir) {
        std::string model_file = model_dir + "/nms.tar";
        std::string untar_cmd =
            "tar -xf " + model_file + " -C . --no-same-owner";
        std::string final_file = model_file + ".so";
        std::string cc_cmd =
            "g++ -shared -fPIC -o " + final_file + " lib0.o devc.o";
        int sys_status = 0;

        sys_status = system(untar_cmd.c_str());
        if (sys_status) {
            std::cout << "Error: cannot untar file " << model_file << std::endl;
            exit(-1);
        }

        // create shared
        sys_status = system(cc_cmd.c_str());
        if (sys_status) {
            std::cout << "Error: compile for " << model_file << std::endl;
            exit(-1);
        }
        std::cout << "compile done." << std::endl;
    }

    void transposeAndCopyToTensor(const Mat &src, NDTensor &dst) {
        Mat channels[3];
        split(src, channels);
        int offset = src.rows * src.cols;
        auto start = dst.value();
        for (int i = 0; i < 3; ++i) {
            std::memcpy(start + i * offset, channels[i].data,
                        offset * sizeof(float));
        }
    }

    std::shared_ptr<std::unordered_map<std::string, NDTensor>>
    preprocess(cv::Mat frame, const std::vector<int64_t> &input_size) {
        cv::Mat x;
        NDTensor scale_factor({1, 2}),
            img({1, 3, input_size[0], input_size[1]});
        scale_factor.value()[0] =
            static_cast<float>(input_size[0]) / frame.size[0];
        scale_factor.value()[1] =
            static_cast<float>(input_size[1]) / frame.size[1];
        NDTensor im_shape({1, 2});
        im_shape.value()[0] = input_size[0];
        im_shape.value()[1] = input_size[1];
        cv::cvtColor(frame, x, cv::COLOR_BGR2RGB);
        cv::resize(x, x, cv::Size(input_size[0], input_size[1]), 0, 0, 2);
        x.convertTo(x, CV_32FC3);
        x *= 1 / 255.0;
        cv::subtract(x, cv::Scalar(0.485, 0.456, 0.406), x);
        cv::multiply(x, cv::Scalar(1 / 0.229, 1 / 0.224, 1 / 0.225), x);
        transposeAndCopyToTensor(x, img);

        std::unordered_map<std::string, NDTensor> ret = {
            {"image", img},
            {"im_shape", im_shape},
            {"scale_factor", scale_factor}};

        return std::make_shared<std::unordered_map<std::string, NDTensor>>(ret);
    }

    void run(const std::unordered_map<std::string, NDTensor> &feeds) {
        auto &image = feeds.at("image");
        this->predictor_nna_->set_inputs({{"image", image}});

        // ppnc_nna run
        this->predictor_nna_->run();

        auto memory_info =
            Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::vector<Ort::Value> input_tensors;
        NDTensor im_shape = feeds.at("im_shape");
        NDTensor scale_factor = feeds.at("scale_factor");

        int numel =
            std::accumulate(im_shape.shape.begin(), im_shape.shape.end(), 1,
                            std::multiplies<>());
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, im_shape.value(), numel, im_shape.shape.data(),
            im_shape.shape.size()));
        numel =
            std::accumulate(scale_factor.shape.begin(),
                            scale_factor.shape.end(), 1, std::multiplies<>());
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, scale_factor.value(), numel, scale_factor.shape.data(),
            scale_factor.shape.size()));

        for (size_t i = 2; i < this->onnx_input_names_.second.size(); ++i) {
            const NDTensor &t = this->predictor_nna_->get_output(i - 2);
            numel = std::accumulate(t.shape.begin(), t.shape.end(), 1,
                                    std::multiplies<>());
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, t.value(), numel, t.shape.data(), t.shape.size()));
        }

        Ort::RunOptions run_options;
        auto onnx_out = this->predictor_onnx_->Run(
            run_options, this->onnx_input_names_.second.data(),
            input_tensors.data(), input_tensors.size(),
            this->onnx_out_names_.second.data(),
            this->onnx_out_names_.second.size());

        auto *data0 = onnx_out[0].GetTensorData<float>();
        auto *data1 = onnx_out[1].GetTensorData<float>();
        auto output_shape0 = onnx_out[0].GetTensorTypeAndShapeInfo().GetShape();
        auto output_shape1 = onnx_out[1].GetTensorTypeAndShapeInfo().GetShape();

        int32_t cls_num, num, box_point;
        // bbox
        num = output_shape0[1];
        box_point = output_shape0[2];
        cls_num = output_shape1[1];

        NDTensor bboxes({1, num, box_point});
        NDTensor scores({1, cls_num, num});

        std::memcpy(bboxes.value(), data0, num * box_point * sizeof(float));
        std::memcpy(scores.value(), data1, cls_num * num * sizeof(float));

        // ppnc_nms run
        this->predictor_nms_->set_inputs(
            {{"bboxes", bboxes}, {"scores", scores}});
        this->predictor_nms_->run();
    }

    NDTensor get_output(int index) {
        return this->predictor_nms_->get_output(index);
    }

    void render() {
        NDTensor res = get_output(0);
        auto data = res.value();
        auto prod = std::accumulate(res.shape.begin(), res.shape.end(), 1,
                                    std::multiplies<int64_t>());

        results.clear();
        PredictResult result;
        for (int i = 0; i < prod; i += 6) {
            result.type = int(data[i]);
            result.score = data[i + 1];

            if (result.score < score) {
                continue;
            }
            // if (result.type == 4)
            // {
            //     continue;
            // }


            // turnning....
            if (static_cast<size_t>(result.type) < labels.size())
                result.label = labels[result.type];
            result.x = data[i + 2];
            result.y = data[i + 3];
            result.width = data[i + 4] - data[i + 2];
            result.height = data[i + 5] - data[i + 3];
            results.push_back(result);
        }
    }

    void drawBox(Mat &img) {
        for (size_t i = 0; i < results.size(); i++) {
            PredictResult result = results[i];

            auto score = std::to_string(result.score);
            int pointY = result.y - 20;
            if (pointY < 0)
                pointY = 0;
            cv::Rect rectText(result.x, pointY, result.width, 20);
            cv::rectangle(img, rectText, getCvcolor(result.type), -1);
            std::string label_name = result.label + " [" +
                                     score.substr(0, score.find(".") + 3) + "]";
            cv::Rect rect(result.x, result.y, result.width, result.height);
            cv::rectangle(img, rect, getCvcolor(result.type), 1);
            cv::putText(img, label_name, Point(result.x, result.y),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
        }
    }

    /**
     * @brief 获取Opencv颜色
     *
     * @param index 序号
     * @return cv::Scalar
     */
    cv::Scalar getCvcolor(int index) {
        switch (index) {
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

  private:
    std::vector<std::string> labels;
    // onnx info
    std::pair<std::vector<std::string>, std::vector<const char *>>
        onnx_input_names_;
    std::pair<std::vector<std::string>, std::vector<const char *>>
        onnx_out_names_;
    Ort::Env onnx_env_;
    // predictor
    std::shared_ptr<PPNCPredictor> predictor_nna_;
    std::shared_ptr<PPNCPredictor> predictor_nms_;
    std::shared_ptr<Ort::Session> predictor_onnx_;
};
