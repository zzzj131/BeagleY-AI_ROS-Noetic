# BeagleY-AI ROS工作空间

这是一个基于ROS-Noetic的机器人开发工作空间，专门为BeagleY-AI开发板环境设计，集成了导航、视觉识别、MAVROS控制等多个功能模块。
别问为什么用BeagleY-AI。。。物联网TI赛道的要求
## 功能模块

### 1. 飞行控制
- MAVROS - 用于ROS和PX4/APM之间的通信
- mavlink - MAVLink协议支持
- offboard_single_position - 单点位置控制
- offboard_multi_position - 多点位置控制

### 2. 视觉系统
- ar_track_alvar - AR标签追踪
- ar_track_landing - 基于AR标签的自动降落
- realsense-ros - Intel RealSense相机驱动
- vision_opencv - OpenCV视觉处理
- usb_cam - USB相机驱动
- robot_vision - 机器人视觉处理

### 3. 导航系统
- navigation_single_goals - 单点导航
- navigation_multi_goals - 多点导航
- ros_navigation - ROS导航功能包
- global_waypoints - 全局路径点
- slam - SLAM建图定位

## 环境要求

- 开发板环境
- ROS版本：Noetic
- 其他依赖：
  - OpenCV
  - PCL
  - Eigen

## 编译说明

1. 首次编译前，请确保已安装所需依赖：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

2. 编译工作空间：
```bash
catkin_make
```

3. 配置环境：
```bash
source devel/setup.bash
```

## 目录结构

本工作空间包含两个主要部分：

### mavros_build/
包含MAVROS及其依赖项的源码，主要包括：
- ros_comm
- common_msgs
- geometry2
- mavros
- mavlink等核心功能包

### ros_ws/
包含机器人功能实现相关的ROS包：
- 导航相关包
- 视觉处理包
- 控制相关包
- 硬件驱动包
- 我完成的是`global_waypoints_cruise`功能包，其他功能包是超维空间给的

## 常见问题

如果遇到编译或运行问题，请检查：
1. 环境变量是否正确设置
2. 依赖是否完整安装
3. 开发板资源使用情况

## 注意事项

1. 请注意开发板的资源限制，编译时可能需要合理分配内存使用
2. 建议使用分步编译方式，避免一次性编译所有包
3. 确保板载传感器和执行器的正确连接
