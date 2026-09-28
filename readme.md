# 二十届完全模型组国一开源
## Author 胡城玮 孙酩贺
大家好 我们是**南京信息工程大学 RunACCM2025**  
在二十届智能车竞赛完全模型组的成绩是全国第五名  
在此开源所有的上位机代码  
大家一起共同进步  
如有想交流的车友 可联系邮箱 <ccch1203@gmail.com>  
祝愿车赛越办越好 大家都能取得理想的成绩！  
[决赛视频](https://www.bilibili.com/video/BV1rqegzqEai/?spm_id_from=333.1387.homepage.video_card.click)  
开源交流QQ群431268082  
作者个人BLOG(内含开源教程) http://www.hcc1203.top/

## 运行环境

代码运行在百度 EdgeBoard（智能汽车赛事版，aarch64）上，依赖以下库，缺任意一项 cmake 都会直接报错：

| 依赖 | 用途 |
| --- | --- |
| OpenCV | 图像采集与处理 |
| libserial | 与下位机串口通信 |
| ppnc | AI 模型主干推理（EdgeBoard NNA） |
| onnxruntime | AI 后处理 NMS |
| glib-2.0 | ppnc 依赖 |

编译需要 C++17。

另外 `model/` 目录不在仓库里（体积原因，已在 `.gitignore` 中排除），需自行放入训练好的模型，目录内应包含 `post.onnx`、`io_paddle.json`、`nms.tar`、`label_list.txt` 四个文件，缺失会在 `Detection` 构造时退出。

## 使用教程

```bash
# 进入文件夹后
mkdir build
cd build
cmake ..
sudo make -j3
./icar
```

注意必须在 `build/` 目录下启动，程序内的配置、模型、存图路径都是相对工作目录的（`../param/`、`../model/`、`../image/`）。

摄像头按固定设备名打开：`/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0`，换摄像头需要改 `capture/capture.cpp`。

## 目录结构描述

```C
RunACCM2025
│  .gitignore
│  CMakeLists.txt
│  compile_commands.json
│  devc.o
│  lib0.o
│  LICENSE
│  main.cpp // 读取config及线程启动
│  readme.md
├─docs
│      runaccm2025-pipeline.html // 上位机架构图（浏览器打开）
├─.vscode
│      c_cpp_properties.json
│      settings.json
│      tasks.json
├─capture
│      capture.cpp // 相机配置
│      capture.h
├─include
│      common.hpp // 常用结构体等定义
│      detection.hpp // AI推理头文件
│      json.hpp
│      logger.hpp // logger，记录并输出调试信息
│      uart.hpp // 通信协议
├─param
│      config_110.json
│      config_130.json
│      config_150.json
│      config_90.json
│      config_choose.json // 选择使用哪一种config
│      config_ppncnms.json
│      config_ppncnna.json
│      param.hpp
├─thread
│      thread.cpp // 多线程文件
│      thread.h
└─track
    ├─basic
    │    |--cross.cpp // 十字识别及处理
    │    |--cross.h
    │    |--ring.cpp // 圆环识别及处理
    │    |--ring.h
    │
    ├─imgprocess
    │    |--imgprocess.cpp
    │    |--imgprocess.h
    │
    ├─special
    │      bridge.cpp // 桥识别及处理
    │      bridge.h
    │      catering.cpp // 汉堡识别及处理
    │      catering.h
    │      charging.cpp // 充电区识别及处理
    │      charging.h
    │      crosswalk.cpp // 斑马线识别及处理
    │      crosswalk.h
    │      layby.cpp // 临时停车区识别及处理
    │      layby.h
    │      obstacle.cpp // 障碍区识别及处理
    │      obstacle.h
    │
    └─standard
            general.h // 常用函数
            standard.cpp // 主要进程文件函数
            standard.h
```

## 代码结构与运行逻辑

架构图见 `docs/runaccm2025-pipeline.html`，用浏览器打开，图上每个节点都标了对应的源码位置。

程序是三条线程的流水线，靠 `Factory<T>`（`deque` + 互斥锁，满了丢最旧的一帧）解耦，巡线与 AI 队列深度 3、调试队列深度 5：

- `producer`（`thread/thread.cpp`）取一帧图像，同帧投给巡线队列和 AI 队列
- `AIConsumer` 跑目标检测，ppnc 算主干、onnx 算后处理 NMS，结果写进共享的 `predict_result`
- `consumer` 调 `Standard::run` 拿到舵机 PWM 和目标速度，再经 `Uart::carControl` 下发

`Standard::run`（`track/standard/standard.cpp`）是核心，一帧内依次做五件事：

1. **巡线**：OTSU 二值化 → `floodFill` 只留车道连通域 → 最长白列定左右起点 → 迷宫法爬边线 → 逆透视到俯视图 → 滤波 → 等距重采样 → 局部角度加非极大值抑制找拐点
2. **中线**：单边巡线时把边线按法向外推半个车道宽；双边时取四个采样点做三阶贝塞尔曲线
3. **元素检测与处理**：见下一节
4. **偏差计算**：按近、远预瞄距离各取一个预瞄点算前轮转角偏差，按场景加权
5. **执行**：PD 输出叠加到舵机中值 `PWMSERVOMID` 上，夹紧到 `PWMSERVOMIN`~`PWMSERVOMAX`

## 参数配置

`param/config_choose.json` 里的 `jsonPth` 字段指向真正生效的配置文件，改这一行来切换档位（`config_90/110/130/150.json`，数字是速度档）。

其余字段按元素分组，命名前缀即对应模块，比如 `CHARGING_*` 是充电区、`LAYBY_*` 是临时停车区、`OBSTACLE_width_*` 是各类障碍的避让宽度。

最需要理解的是**元素顺序表**。因为赛道元素的出现次序是赛前已知的，代码不做通用识别，而是按顺序表逐个「放行」——只有 `elem_order[order_index]` 匹配时才检测对应元素，处理完 `order_index++` 前进到下一个：

```
elem_order 取值：
  0 斑马线（终止位）   1 汉堡      2 虚线停车    5 圆环     6 十字     7 桥
  8 左1充电   9 左2充电   10 右1充电   11 右2充电
```

障碍不在 `elem_order` 里，它由下面的 `obstacle_order_index` 单独开关控制（`standard.cpp` 顶部注释中的「4.障碍」已过时）。

与它并列的几个数组共用同一个 `order_index`，即第 n 个元素各自用第 n 项参数：

| 数组 | 作用 |
| --- | --- |
| `elem_order` | 该位置是什么元素 |
| `speed_order` | 该段目标速度 |
| `aim_dis_n_order` | 该段近预瞄距离 |
| `aim_angle_p_order` / `aim_angle_d_order` | 该段转向 PD 参数 |
| `obstacle_order_index` | 该段是否开启障碍检测 |

> **注意**：这几个数组目前长度并不一致（例如 `config_110.json` 中 `elem_order` 15 项，而 `speed_order` 13 项、`obstacle_order_index` 11 项），而代码对 `order_index` 没有上界保护。元素数量超过较短数组的长度时会发生越界读取。自行调整顺序表时，建议把所有数组补成与 `elem_order` 等长。

## 特别鸣谢

**长安大学 没有霹雳猫猫队**  
**天津大学      天有四时队**  
**上海海事大学  赤霄驭风队**  
**华南理工大学  华工龙泉队**  
**上海电机学院  孤注一航队**  
**江苏理工学院  凌波·夜幕队**  
**大连民族大学  大佬说的队**  
**长安大学十九届开源链接 <https://gitee.com/JYSimilar/icar_2024>**
