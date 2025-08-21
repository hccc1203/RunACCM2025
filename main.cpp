/*************************************/
// Author 胡城玮
/*************************************/

#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>


#include "param/param.hpp"
#include "thread/thread.h"


//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//               佛祖保佑         国一拿下
//
//
//

//        ┏┓　　　┏┓+ +
//　　　┏┛┻━━━┛┻┓ + +
//　　　┃　　　　　　　┃ 　
//　　　┃　　　━　　　┃ ++ + + +
//　　 ████━████ ┃+
//　　　┃　　　　　　　┃ +
//　　　┃　　　┻　　　┃
//　　　┃　　　　　　　┃ + +
//　　　┗━┓　　　┏━┛
//　　　　　┃　　　┃　　　　　　　　　　　
//　　　　　┃　　　┃ + + + +
//　　　　　┃　　　┃　　　　Codes are far away from bugs with the animal protecting　　　
//　　　　　┃　　　┃ + 　　　　神兽保佑,国一拿下　　
//　　　　　┃　　　┃
//　　　　　┃　　　┃　　+　　　　　　　　　
//　　　　　┃　 　　┗━━━┓ + +
//　　　　　┃ 　　　　　　　┣┓
//　　　　　┃ 　　　　　　　┏┛
//　　　　　┗┓┓┏━┳┓┏┛ + + + +
//　　　　　　┃┫┫　┃┫┫
//　　　　　　┗┻┛　┗┻┛+ + + +




Uart uart = Uart("/dev/ttyUSB0"); // 初始化串口驱动

void Set_Config(Config &config)
{
    Config cfg_pth;
    std::string jsonPath = "../param/config_choose.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good())
    {
        std::cout << "Error: Params file path:[" << jsonPath
                    << "] not find .\n";
        exit(-1);
    }

    nlohmann::json js_pth;
    config_is >> js_pth;

    try
    {
        cfg_pth = js_pth.get<Config>();
    }
    catch (const nlohmann::detail::exception &e)
    {
        std::cerr << "Json Params Parse failed :" << e.what() << '\n';
        exit(-1);
    }

    jsonPath = cfg_pth.jsonPth;
    nlohmann::json js_value;
    std::ifstream config_is2(jsonPath);
    if (!config_is.good())
    {
        std::cout << "Error: Params file path:[" << jsonPath
                    << "] not find .\n";
        exit(-1);
    }
    config_is2 >> js_value;
    try
    {
        config = js_value.get<Config>();
    }
    catch (const nlohmann::detail::exception &e)
    {
        std::cerr << "Json Params Parse failed :" << e.what() << '\n';
        exit(-1);
    }
    cout << "--- steeringP:" << config.steering_p << " | steeringD:" << config.steering_d << endl;
    cout << "--- speedLow:" << config.speedLow << "m/s  |  speedHigh:" << config.speedHigh << "m/s" << endl;
}
int main()
{
    Config config;
    Set_Config(config);

    shared_ptr<Uart> uart;
    if (config.ttyUsb == 0)
        uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
    else if (config.ttyUsb == 1)
        uart = make_shared<Uart>("/dev/ttyUSB1");
    else if (config.ttyUsb == 2)
        uart = make_shared<Uart>("/dev/ttyUSB2");
    else if (config.ttyUsb == 3)
        uart = make_shared<Uart>("/dev/ttyUSB3");
    else if (config.ttyUsb == 4)
        uart = make_shared<Uart>("/dev/ttyUSB4");
    int ret = uart->open();
    if (ret != 0)
    {
        printf("[Error] Uart Open failed!\n");
        return -1;
    }

    
    std::cout << config.speedDown << std::endl;
    Factory<TaskData> task_factory(3);
	Factory<TaskData> AI_task_factory(3);
	Factory<DebugData> debug_factory(5);
    if(!config.wifi_start)
    {
        for(int i = 5;i > 0;i--)
        {
            sleep(1);
            printf("%d\n",i);
        }
    }
    std::vector<PredictResult> predict_result;

    std::thread task_producer(&producer, std::ref(task_factory), std::ref(AI_task_factory), std::ref(config));

	std::thread AI_producer(&AIConsumer, std::ref(AI_task_factory), std::ref(predict_result), std::ref(config));

	std::thread task_consumer(&consumer, std::ref(task_factory), std::ref(debug_factory), std::ref(predict_result), std::ref(config), std::ref(uart));
	
    // std::thread uart_receive(&uartReceive,std::ref(uart));

    
    if (config.en_show) {
		std::thread debug_data_consumer(&debugDataConsumer, std::ref(debug_factory));
		debug_data_consumer.join();
	}

    
	task_producer.join();
    AI_producer.join();
    task_consumer.join();
    // uart_receive.join();


    while(1);

    
    return 0;
}