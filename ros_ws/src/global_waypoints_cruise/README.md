# global_waypoints_cruise

## 简介
`global_waypoints_cruise` 是一个用于全局航点巡航的 ROS 包（针对 Noetic / catkin 工程结构）。
该包提供将一组经纬度/相对坐标航点按顺序送入导航栈或下发给飞控（通过 mavros）的示例节点和 launch 文件，便于在室外/室内巡航任务中自动路径巡航与测试。

> 说明：下文中的命令假定工作区根目录为 `d:\CodesLib\beagle_ws\ws`（与当前仓库结构一致）。

## 先决条件
- Ubuntu 20.04 + ROS Noetic（或与 Noetic 兼容的环境）
- catkin 工具（`catkin_make` 或 `catkin build`）
- `mavros`（如果要通过 MAVLink 下发命令）
- 常见依赖：`rospy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`，以及包内部 `package.xml`/`CMakeLists.txt` 中声明的依赖

## 构建
在工作区根目录下运行：

```cmd
cd d:\CodesLib\beagle_ws\ws
catkin_make
rem # 或者使用 catkin_tools: catkin build
```

构建完成后，记得 source 工作区：

```cmd
call devel\setup.bat  # Windows 下若使用 ROS on Windows 或在 WSL 中使用 source devel/setup.bash
```

## 使用方法（运行示例）
下面给出一个典型运行流程：

1. 启动 ROS 主节点（如果尚未启动）
2. 启动飞控/仿真和 `mavros`（若使用 MAVLink）
3. 启动 `global_waypoints_cruise` 的示例 launch 或节点

示例（假设存在 `launch/cruise_example.launch`）：

```cmd
roslaunch global_waypoints_cruise cruise_example.launch
```

或者运行节点（假设使用 Python 节点 `scripts/waypoint_cruise.py`）：

```cmd
rosrun global_waypoints_cruise waypoint_cruise.py
```

> 如果包中没有这些文件，请查看包内 `launch/` 与 `scripts/` 或 `nodes/` 目录以确定具体的可执行项和参数。

## 参数与配置
常见配置项（可在 launch 中或参数服务器上设置）：
- `~waypoints`：航点列表（可为经纬度或局部坐标序列）
- `~frame_id`：坐标系（如 `map` / `odom`）
- `~cruise_speed`：巡航速度
- `~use_mavros`：是否通过 mavros 下发航点

具体参数以包内节点实现为准，请查看 `params` 或节点源码以获得准确的参数名称与格式。

## 话题 / 服务 / Action（示例）
- 订阅：
  - `/mavros/state`（飞控状态，若使用 mavros）
  - `/current_pose`（当前位姿，或由定位模块提供）
- 发布：
  - `/mavros/setpoint_position/local`（向飞控发送位置 setpoint）
- 服务/Action：
  - `/start_cruise`（可选，触发开始巡航）
  - `/stop_cruise`（可选，停止巡航）

请查看包内源码以确认实际使用的 topic/service/action 名称和消息类型。

## 开发与贡献
- 如果要修改或添加航点格式，请在 `scripts/` 或 `nodes/` 中更新解析逻辑，并在 `launch/` 中给出示例参数。
- 提交合并请求前，请确保：
  - 本地构建通过
  - 新增功能有简要文档和（如有必要）测试

## 常见问题
- 文件换行符：在 Windows/Unix 混合开发环境下可能会出现 LF/CRLF 警告。建议使用统一的换行策略（例如 .gitattributes 或编辑器配置）。
- 子模块：如果仓库中包含依赖子模块，请在拉取后执行 `git submodule update --init --recursive`。

## 许可证
请遵循仓库的主许可证（查看仓库根目录的 `LICENSE` 文件）。

---

如果您希望我把 README 调整为更具体的内容（例如增加具体的 launch 示例、列出确切的话题/参数名或加入示例航点文件），告诉我您希望包含的细节，我会在此基础上更新文件。