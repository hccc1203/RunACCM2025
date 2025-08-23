# 二十届完全模型组国一开源

大家好 我们是**南京信息工程大学 RunACCM2025**
在二十届智能车竞赛完全模型组的成绩是全国第五名
在此开源所有的上位机代码
大家一起共同进步
如有想交流的车友 可联系邮箱 <ccch1203@gmail.com>
祝愿车赛越办越好 大家都能取得理想的成绩！
[决赛视频](https://www.bilibili.com/video/BV1rqegzqEai/?spm_id_from=333.1387.homepage.video_card.click)
开源交流QQ群431268082

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

## 特别鸣谢

**长安大学 没有霹雳猫猫队**
**天津大学      天有四时队**
**上海海事大学  赤霄驭风队**
**华南理工大学  华工龙泉队**
**上海电机学院  孤注一航队**
**江苏理工学院  凌波·夜幕队**
**大连民族大学  大佬说的队**
**长安大学十九届开源链接 <https://gitee.com/JYSimilar/icar_2024>**
