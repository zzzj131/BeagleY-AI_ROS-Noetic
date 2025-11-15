
# global_waypoints_cruise

本包用于在 ROS Noetic 环境下对一组全局航点进行巡航（waypoint cruise）的演示与工具集合。

## 主要功能

- 提供将航点序列发送到飞控/控制节点的示例节点与启动文件。
- 支持通过参数/话题动态加载或修改航点列表。
- 示例包含轨迹循环、按顺序到达以及到达后的动作回调示例。

## 依赖

- ROS: Noetic
- 常用 ROS 包: `roscpp`, `rospy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
- 编译与运行需要已配置的 ROS 工作空间（catkin 工作区）

> 注：如果在 Windows 上开发，请使用 WSL（推荐 Ubuntu + ROS Noetic 环境）来编译与运行此包。

## 编译

在工作空间根目录下执行（假设已将包放在 `src/` 下）：

```bash
cd ~/catkin_ws        # 或您的工作空间路径
catkin_make
source devel/setup.bash
```

或使用 `catkin build`（若使用 catkin_tools）：

```bash
catkin build
source devel/setup.bash
```

## 运行示例

示例启动一个航点巡航节点并加载默认航点文件：

```bash
roslaunch global_waypoints_cruise cruise.launch
```

或者直接运行节点（视实现而定）：

```bash
rosrun global_waypoints_cruise waypoint_cruise_node
```

启动后可通过相应话题发布目标点或查看状态（见下文话题说明）。

## 话题 / 服务 / 参数（示例）

- 话题: `/global_waypoints` (type: `nav_msgs/Path` 或 自定义消息) — 发布/订阅航点序列
- 话题: `/current_waypoint_index` (type: `std_msgs/Int32`) — 当前航点索引
- 服务: `/start_cruise` (type: `std_srvs/Trigger`) — 启动巡航
- 参数: `~loop` (bool) — 是否循环航点序列，默认：false

请根据包内实现的具体消息/服务名称调整使用。

## 文件/目录说明

- `launch/`：包含启动文件，例如 `cruise.launch`。
- `src/`：节点源代码。
- `config/`：示例航点或参数配置文件（如果存在）。
- `README.md`：本文件。

## 开发与贡献

欢迎提交 issue 或 PR：

1. Fork 本仓库并创建 feature 分支。
2. 在本地实现并添加必要测试。
3. 提交 PR，描述变更内容与测试步骤。

代码风格：请遵循包中已有风格（C++/Python 根据实现）并保持简洁可读。

## 许可证

请参见仓库顶层的 LICENSE 文件。

## 联系

如有问题，可在仓库中打开 issue，或联系维护者。

