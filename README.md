# 海陆空协同智能救援系统  

![ROS](https://img.shields.io/badge/ROS-Noetic-green)
![Python](https://img.shields.io/badge/Python-3.8+-blue)
![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)
![Platform](https://img.shields.io/badge/Platform-Jetson%20NX-lightgrey)

---

# 1 项目简介

本项目实现了一套 **海陆空协同智能救援系统**，通过无人机（UAV）、无人车（UGV）和无人船（USV）三种无人平台的协同工作，实现对突发事件的 **自动识别、快速定位和协同救援**。

系统基于 **ROS 分布式通信架构**，结合 **PX4飞控系统、MAVROS通信接口、YOLO深度学习目标检测算法以及RTK高精度定位技术**，实现从 **目标发现 → 目标定位 → 协同调度 → 执行救援 → 状态反馈** 的完整自动化流程。

系统主要应用场景包括：

- 水域安全巡检
- 景区应急救援
- 公共安全监控
- 智慧海事管理

---

# 2 系统总体架构

系统采用 **分布式 ROS 节点架构**，通过 Topic 实现多设备通信。


                 ┌──────────────────────────┐
                 │      Jetson NX 边缘计算     │
                 │                            │
                 │     YOLOv5 视觉识别节点     │
                 │      detect_beset.py       │
                 │                            │
                 └─────────────┬──────────────┘
                               │
                 fall_alert / drowning_alert
                               │
                     ┌─────────▼─────────┐
                     │   UAV控制中心节点   │
                     │      center.py     │
                     └───────┬───────┬────┘
                             │       │
                    ┌────────▼─┐   ┌─▼────────┐
                    │ UGV控制节点│   │USV控制节点│
                    │  uav2.py  │   │ uav3.py  │
                    └──────┬────┘   └────┬─────┘
                           │             │
                       MAVROS         MAVROS
                           │             │
                      无人车平台        无人船平台
                           │
                    ┌──────▼──────┐
                    │ 串口通信模块 │
                    │   usart.py  │
                    └─────────────┘
                    
---

# 3 系统核心功能

## 3.1 空中巡航侦察

无人机执行自主巡航任务，对目标区域进行持续监测。

主要功能：

- 自动解锁
- 自动起飞
- 圆形巡航
- 实时图像采集
- AI目标识别

巡航轨迹采用 **圆形轨迹算法**：

### x = cx + r cosθ
### y = cy + r sinθ

参数说明：

|参数|说明|
|---|---|
cx,cy | 巡航中心坐标  
r | 巡航半径  
θ | 巡航角度  

---

# 4 AI视觉识别系统

系统使用 **YOLOv5目标检测模型**进行实时识别。

识别目标：

|类别|说明|
|---|---|
fall | 人员摔倒  
drown | 人员溺水  

模型文件：beset.pt

检测节点：detect_beset.py

检测流程：摄像头视频流->图像预处理->YOLOv5推理->置信度筛选->事件发布

ROS发布消息：
fall_alert
drowning_alert

---

# 5 UAV控制系统

核心控制节点：


center.py


主要功能：

- 无人机飞行控制
- 巡航任务管理
- 目标识别处理
- 协同任务调度

当检测到目标时：


无人机悬停
↓
获取GPS坐标
↓
发布目标位置
↓
等待执行设备反馈


---

# 6 无人车协同救援

控制节点：


uav2.py


主要功能：

- 接收目标GPS
- 计算目标距离
- 自动导航
- 到达目标点

距离计算使用 **Haversine公式**：


d = 2R * asin( √(sin²((lat2-lat1)/2) +
cos(lat1)cos(lat2)sin²((lon2-lon1)/2)) )


导航流程：


接收目标坐标
↓
计算当前位置距离
↓
航向调整
↓
自动前进
↓
抵达目标
↓
发送 arrived 信号


---

# 7 无人船协同救援

控制节点：


uav3.py


执行流程：


接收溺水目标
↓
获取GPS坐标
↓
自动航行
↓
抵达目标区域
↓
发送完成信号


适用场景：

- 湖泊
- 河流
- 景区水域
- 海岸巡检

---

# 8 串口通信系统

通信节点：


usart.py


主要功能：

- 向外部设备发送识别信息
- 上传各设备位置
- 实现设备通信桥接

串口配置：


设备: /dev/ttyTHS1
波特率: 115200


发送数据示例：


fall
drown

UAV:lat,lon,alt
UGV:lat,lon,alt
USV:lat,lon,alt


---

# 9 ROS通信机制

系统通过 ROS Topic 实现节点通信。

|Topic|类型|说明|
|---|---|---|
fall_alert | String | 摔倒检测事件  
drowning_alert | String | 溺水检测事件  
/uav1/position1 | String | 无人车目标  
/uav1/position2 | String | 无人船目标  
/uav2/arrived | String | 无人车到达  
/uav3/arrived | String | 无人船到达  

任务闭环：


无人机识别
↓
发布目标
↓
车/船导航
↓
抵达目标
↓
发送确认
↓
无人机继续巡航


---

# 10 项目结构


rescue_system/
│
├── center.py
│ UAV任务控制节点
│
├── uav2.py
│ 无人车控制节点
│
├── uav3.py
│ 无人船控制节点
│
├── detect_beset.py
│ YOLO目标检测节点
│
├── usart.py
│ 串口通信节点
│
├── teset.py
│ 系统测试脚本
│
├── control.launch
│ ROS启动文件
│
├── beset.pt
│ 训练好的检测模型
│
└── README.md


---

# 11 系统环境

软件环境：


Ubuntu 20.04
ROS Noetic
Python 3.8
PX4 Autopilot
MAVROS


硬件平台：


Jetson NX
STM32F765飞控
RTK高精度GPS
GW200摄像头
3DR无线数传
无人车平台
无人船平台
