&nbsp;								 海陆空协同救援系统 

\*\*Air-Land-Sea Collaborative Emergency Rescue System based on ROS \& MAVROS\*\*



!\[ROS](https://img.shields.io/badge/ROS-Noetic-green.svg) !\[Python](https://img.shields.io/badge/Python-3.8+-blue.svg) !\[MAVROS](https://img.shields.io/badge/MAVROS-v1.13-orange.svg) !\[Platform](https://img.shields.io/badge/Platform-Jetson%20NX-lightgrey.svg)



本项目是一个基于 ROS1 与 PX4 (MAVROS) 框架开发的\*\*异构多智能体协同控制系统\*\*。系统由无人机（UAV）、无人车（UGV）和无人船（USV）组成，旨在解决复杂环境下的突发事件（如人员落水、人员摔倒）应急救援问题。



通过边缘计算平台（Jetson NX）部署目标检测算法，实现\*\*“空中广域巡航侦察 -> AI 异常目标识别 -> 空地/空水协同调度 -> 载具自主寻迹救援 -> 握手确认与恢复巡航”\*\*的全链路闭环自动控制。



---



\## 核心特性 (Core Features)



\- \*\*完善的飞行状态机 (FSM)\*\*：UAV 节点 (`center.py`) 采用健壮的状态机架构（`IDLE` -> `TAKING\_OFF` -> `CRUISING` -> `HOVERING`），支持防误触检测机制与丢点容错自愈。

\- \*\*纯 GPS （本人采用RTK）全局协同导航\*\*：车船节点摒弃了易产生死区的局部步进算法，采用基于\*\*Haversine 大圆航线公式\*\*的纯 GPS 全局位置控制 (`GlobalPositionTarget`)，实现平滑、精准的跨平台制导。

\- \*\*节点级握手通信\*\*：创新引入目标到达回调机制。UGV/USV 到达目标区域后发布 `arrived` 信号，UAV 收到反馈后自动终止悬停并恢复广域巡航，形成完整的业务闭环。

\- \*\*软硬解耦的硬件桥接\*\*：独立的 `usart.py` 节点负责提取三端设备的实时高精度 GPS 及救援状态，通过 Jetson NX 的 UART 发送至外部飞控或地面站终端，支持 4G/原子云透传。

\- \*\*内置 SIL/HIL 测试桩\*\*：提供交互式命令行测试工具 (`yolo.py`)，无需启动真实视觉节点即可在仿真环境下验证全套多智能体协同逻辑。



---



\## 系统架构图 (System Architecture)



```text

\[边缘计算端/Jetson NX]                       \[外部/地面站端]

&nbsp;      |                                          |

&nbsp;      |-- (fall\_alert / drowning\_alert)          |

\[YOLO 视觉节点] ---------+                        |

&nbsp;                        v                        |

&nbsp;              +------------------+               |

&nbsp;              |  center.py (UAV) | <----- (状态/指令) ---- \[yolo.py 模拟器/人工]

&nbsp;              +------------------+               |

&nbsp;                |      |       ^                 |

&nbsp;(GPS 目标点)    |      |       | (arrived 回调)   |

&nbsp;       +--------+      +-------+                 |

&nbsp;       v                       v                 v

+----------------+      +----------------+    +----------------+

| uav2\_controller|      |      uav3      |    |    usart.py    | 

|     (UGV)      |      |     (USV)      |    | (串口数据透传) | 

+----------------+      +----------------+    +----------------+

&nbsp;       |                       |                 | UART (ttyTHS1)

&nbsp;       v                       v                 v

&nbsp; \[MAVROS / PX4]          \[MAVROS / PX4]    \[STM32 / 4G DTU / 地面站]



