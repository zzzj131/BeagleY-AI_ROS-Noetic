# global_waypoints_cruise

这是一个用于无人机/机器人在全局（GPS）航点上执行巡航任务的 ROS Noetic 包，包含：

- 航点加载器（从 MQTT/远端服务获取航点并写入参数服务器）：`waypoints_loader_node`。
- 航点巡航执行器（读取参数服务器上的航点，上传给飞控 via MAVROS 并执行）：`global_waypoints_cruise_node`。
- GPS 日志记录器：`drone_gps_logger_node`（用于记录位置信息，便于回放与调试）。
- `drone_gps_logger_node`还包含了通过串口读取环境传感器数据并上报到后端的实现

下文基于仓内实现与配置文件撰写，包含构建、运行与参数/话题说明。

## 先决条件

-  ROS Noetic
- 安装系统依赖（示例，Ubuntu，本项目是在Debian 12上编译完成）：

```bash
sudo apt update
sudo apt install -y libmosquitto-dev libyaml-cpp-dev libjsoncpp-dev libserial-dev
```

（如果使用 `catkin_tools`，也请确保已安装 `python3-catkin-tools`）

此外该包依赖 MAVROS（请先安装并配置好与飞控的连接）：

```bash
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
```

如果你通过 MQTT 与远端数据库通信，请确保能够访问配置中的 `mqtt_broker`（默认示例在源码为 192.168.137.1）。

## 构建

在工作空间根目录下构建：

```bash
# 进入你的 catkin 工作空间（示例）
cd ~/catkin_ws
catkin_make    # 或者 catkin build
source devel/setup.bash
```

如果遇到找不到 yaml-cpp / mosquitto / jsoncpp 的错误，请确认系统依赖已安装并且 CMake 能检测到这些库。

## 配置与运行

默认 launch 文件位于 `launch/mission.launch`。该 launch 启动两个主要节点：

- `waypoints_loader`：从 MQTT/远端请求航点并将其写入参数服务器（私有命名空间 `~`，默认参数名 `/global_waypoints_cruise/waypoints`）。
- `global_waypoints_cruise`：读取参数服务器上的航点，上传到飞控并执行任务。

示例：

```bash
# 启动 MAVROS（如果未在其他终端启动）
roslaunch mavros px4.launch  # 或者你的平台对应的启动文件

# 在另一个终端启动本包的 mission
roslaunch global_waypoints_cruise mission.launch
```

如果需要直接运行节点：

```bash
rosrun global_waypoints_cruise waypoints_loader_node
rosrun global_waypoints_cruise global_waypoints_cruise_node
```

### MQTT 与远端交互参数（waypoints_loader）

在 `waypoints_loader_node` 中，可通过私有参数配置以下选项（可在 launch 中以 `<param>` 方式传入）：

- `mqtt_broker`（string）: MQTT Broker 地址，默认 `192.168.137.1`（请根据你的环境修改）。
- `mqtt_port`（int）: MQTT 端口，默认 `8899`。
- `mqtt_topic_send`（string）: 发送查询到后端的主题，默认 `ros/to/mysql`。
- `mqtt_topic_receive`（string）: 接收后端响应的主题，默认 `mysql/to/ros`。
- `vehicle_id`（int）: 请求参数中的车辆/设备ID（示例默认 1001）。
- `param_name`（string）: 写入参数服务器的目标参数名，默认 `/global_waypoints_cruise/waypoints`。
- `response_timeout`（double）: 等待远端响应的超时时间（秒），默认 `10.0`。

后端返回 JSON 的示例格式（waypoints 字段应为数组）：

```json
{
  "status": "success",
  "waypoints": [
    {"latitude": 47.39783, "longitude": 8.54616, "altitude": 3.0, "delay": 5.0},
    {"latitude": 47.39687, "longitude": 8.54655, "altitude": 4.0, "delay": 10.0}
  ]
}
```

### 参数 / 话题 / 服务（在包内的约定）

- 参数（私有/节点命名空间）: `~waypoints` 或 `/global_waypoints_cruise/waypoints` — 包含航点的数组（由 `waypoints_loader_node` 写入）。
- 订阅：`mavros/state`（`mavros_msgs/State`）——用于获取飞控连接和模式信息。
- 订阅：`mavros/mission/reached`（`mavros_msgs/WaypointReached`）——用于检测到达航点的回调。
- 服务调用：`mavros/cmd/arming`、`mavros/set_mode`、`mavros/mission/push`、`mavros/mission/clear` —— 用于上传航点、解锁、设置模式等。由 `global_waypoints_cruise_node` 调用。

具体消息与话题名请参考源码 `src/global_waypoints_cruise.cpp` 与 `src/waypoints_loader_node.cpp`。

## config/waypoints.yaml 示例

仓内已包含 `config/waypoints.yaml`，格式示例：

```yaml
waypoints:
  - latitude: 47.39783
    longitude: 8.54616
    altitude: 3.0
    delay: 5.0
  - latitude: 47.39687
    longitude: 8.54655
    altitude: 4.0
    delay: 10.0
```

该文件可用于本地测试，或由 `waypoints_loader_node` 的后端响应生成并写入参数服务器。

## 故障排查

- 节点在等待参数（`~waypoints`）时会阻塞：确保 `waypoints_loader_node` 已成功将参数写入参数服务器，或直接在 launch 中使用 `<rosparam>` 加载 `config/waypoints.yaml`。
- 如果 MAVROS 相关服务调用失败，检查 MAVROS 是否已启动且与飞控连接正常（`rostopic echo /mavros/state`）。
- 如果无法连接 MQTT broker，确认网络连通性并能通过 `mosquitto_sub` / `mosquitto_pub` 测试 broker。


代码风格请遵循项目中已有 C++/Python 风格并保持注释清晰。


## 串口传感器（Serial）读取说明

`drone_gps_logger_node` 包含通过串口读取环境传感器数据并上报到后端的实现，关键信息如下：

- 依赖：使用 ROS 的 `serial` 库（源码包含 `#include <serial/serial.h>`）。在 Ubuntu/Noetic 上可安装系统包 `ros-noetic-serial`。
- 节点私有参数（可在 launch 中以 `<param>` 方式传入）：
  - `serial_port`（string，默认 `/dev/ttyS2`）——串口设备路径。
  - `serial_baudrate`（int，默认 `115200`）——串口波特率。
  - `csv_delimiter`（string，默认 `,`）——CSV 字段的分隔符。

- 读取频率：节点通过 ROS 定时器每 2 秒调用一次串口读取回调（源码中使用 `nh.createTimer(ros::Duration(2.0), readAndPublishEnvironmentData)`）。
- 重连与错误处理：
  - 若串口未打开，回调会尝试重新打开并在日志中输出重连尝试信息；若打开失败会记录错误并在后续定时器触发时再次尝试。
  - 读取过程中若抛出 `serial::IOException` 或 `serial::PortNotOpenedException`，节点会记录错误并在必要时关闭串口以便下次重连。

### CSV 数据格式（必须为 10 个字段）

串口返回的每行文本会按 `csv_delimiter` 切分，期望得到 10 个字段，源码中按索引解析并进行必要的缩放：

0. temperature (°C) — 原始数值
1. humidity (%) — 原始数值
2. illuminance (lux) — 原始数值
3. formaldehyde (ppm) — 原始数值
4. TVOC (ppb) — 原始数值
5. CO2 (ppm) — 原始数值
6. smoke — 源数据需除以 1000.0
7. temperature2 — 源数据需除以 100.0
8. pressure — 源数据需除以 1000.0
9. altitude — 源数据需除以 100.0

如果解析出的字段数量不等于 10，节点会在日志中发出警告并忽略该行（`Received CSV line with incorrect number of fields`）。

### 上报（MQTT）格式

解析成功后，节点构建 JSON 并通过配置的 `mqtt_topic_send` 主题发布，消息包含 `action: "insert_environment"`、`vehicle_id` 以及各环境量字段（`temperature`、`humidity`、`illuminance`、`formaldehyde`、`tvoc`、`co2`、`smoke`、`temperature2`、`pressure`、`altitude`）。

### 使用建议

- 在调试阶段可使用 `mosquitto_sub -h <broker> -t <topic>` 验证 MQTT 上报是否正确。
- 若串口设备行尾或编码不同，请在设备端或通过 `csv_delimiter` 配合清洗数据，确保字段能被正确切分与转换。
- 如果串口不是必需，可将串口参数留空或在 launch 中不传入；节点在无法打开串口时会继续运行 GPS 上报功能，但环境数据将不可用。


