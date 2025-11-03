#include <ros/ros.h>
#include <ros/param.h> // <--- 包含参数头文件
#include <vector>      // <--- 包含vector头文件 (如果之前没包含)
#include <atomic>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointReached.h>
#include <XmlRpcValue.h> // <--- 包含XmlRpcValue头文件，用于解析参数
// ... (全局变量和回调函数保持不变) ...
mavros_msgs::State current_state;
std::atomic<bool> mission_completed(false);
int current_wp_reached = -1;
int last_wp_index = -1;
double last_wp_delay = 0.0;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
current_state = *msg;
if (msg->mode == "AUTO.LOITER") {
// ROS_WARN("任务已完成（检测到悬停模式）"); // 可以在主循环中处理，避免多次触发
// mission_completed.store(true);
}
}
void reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg) {
current_wp_reached = msg->wp_seq;
ROS_INFO("已到达航点 %d", current_wp_reached);
// 检查是否是最后一个航点 (逻辑移至主循环)
// if (current_wp_reached == last_wp_index) {
//     ROS_INFO("已到达最终航点！");
// }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "global_waypoints_cruise");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // <--- 创建私有NodeHandle以方便获取私有参数
    // ... (订阅者和客户端设置保持不变) ...
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber reached_sub = nh.subscribe<mavros_msgs::WaypointReached>(
        "mavros/mission/reached", 10, reached_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");
    ros::ServiceClient wp_push_client = nh.serviceClient<mavros_msgs::WaypointPush>(
        "mavros/mission/push");
    ros::ServiceClient wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>(
        "mavros/mission/clear");

    // 等待连接
    ros::Rate rate(20.0);
    ROS_INFO("等待连接飞控...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接");

    // --- 从参数服务器读取航点 ---
    XmlRpc::XmlRpcValue waypoints_param;
    std::vector<mavros_msgs::Waypoint> waypoint_list;

    if (!private_nh.getParam("waypoints", waypoints_param)) {
        ROS_ERROR("未能从参数服务器获取 'waypoints' 参数。请确保在launch文件或YAML文件中定义了航点！");
        return -1;
    }

    // 检查参数类型是否为数组
    if (waypoints_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'waypoints' 参数必须是一个数组（列表）类型！");
        return -1;
    }

    ROS_INFO("正在从参数服务器加载 %d 个航点...", waypoints_param.size());

    for (int i = 0; i < waypoints_param.size(); ++i) {
        // 检查每个航点是否是结构体（字典）
        if (waypoints_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("航点 %d 的格式不是结构体，已跳过。", i);
            continue;
        }

        XmlRpc::XmlRpcValue wp_data = waypoints_param[i];
        mavros_msgs::Waypoint wp;

        // --- 设置航点通用属性 ---
        wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT; // 默认使用相对高度全局坐标系
        wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;     // 默认是导航航点
        wp.is_current = (i == 0); // 第一个航点设置为当前航点
        wp.autocontinue = true;   // 默认自动继续

        // --- 解析具体航点数据 ---
        // 纬度 (必需)
        if (wp_data.hasMember("latitude") && wp_data["latitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.x_lat = static_cast<double>(wp_data["latitude"]);
        } else {
            ROS_ERROR("航点 %d 缺少有效的 'latitude' (double类型) 字段。", i);
            return -1;
        }

        // 经度 (必需)
        if (wp_data.hasMember("longitude") && wp_data["longitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.y_long = static_cast<double>(wp_data["longitude"]);
        } else {
            ROS_ERROR("航点 %d 缺少有效的 'longitude' (double类型) 字段。", i);
            return -1;
        }

        // 高度 (必需)
        if (wp_data.hasMember("altitude") && wp_data["altitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.z_alt = static_cast<double>(wp_data["altitude"]);
        } else {
            ROS_ERROR("航点 %d 缺少有效的 'altitude' (double类型) 字段。", i);
            return -1;
        }

        // 停留时间 (可选, param1)
        if (wp_data.hasMember("delay") && wp_data["delay"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.param1 = static_cast<double>(wp_data["delay"]);
        } else {
            wp.param1 = 0.0; // 默认停留时间为0
        }

        // 其他参数 (param2, param3, param4) 可以类似地添加 (如果需要)
        // wp.param2 = ...
        // wp.param3 = ...
        // wp.param4 = ... (通常是航向角 yaw, 如果command是NAV_WAYPOINT)
        if (wp_data.hasMember("yaw") && wp_data["yaw"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            // 注意：param4通常是角度（度），但 MAVLink/MAVROS 可能需要弧度，请查阅文档确认
            // 这里假设是度
            wp.param4 = static_cast<double>(wp_data["yaw"]);
        } else {
            wp.param4 = std::numeric_limits<double>::quiet_NaN(); // NaN 表示不改变航向
        }

        // 可以根据需要添加对 command, frame, autocontinue 等的参数化设置
        // 例如:
        // if (wp_data.hasMember("command") && wp_data["command"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        //     wp.command = static_cast<int>(wp_data["command"]);
        // }
        // ...

        waypoint_list.push_back(wp);
        ROS_INFO("  航点 %d: Lat=%.6f, Lon=%.6f, Alt=%.2f, Delay=%.1f, Yaw=%.1f", i, wp.x_lat, wp.y_long, wp.z_alt, wp.param1, wp.param4);
    }

    if (waypoint_list.empty()) {
        ROS_ERROR("从参数服务器加载的航点列表为空！");
        return -1;
    }
    // --- 航点加载结束 ---


    // 存储最后一个航点的索引和延迟时间
    last_wp_index = waypoint_list.size() - 1;
    if (last_wp_index >= 0) {
        last_wp_delay = waypoint_list.back().param1;
    } else {
        // 这个检查理论上在上面已经覆盖了，但保留无妨
        ROS_ERROR("未能加载任何航点！");
        return -1;
    }

    // ... (清除航点、上传航点、解锁、设置模式的代码保持不变) ...
    // 首先清除所有现有航点
    mavros_msgs::WaypointClear wp_clear_srv;
    if (!wp_clear_client.call(wp_clear_srv)) {
        ROS_WARN("调用mission/clear服务失败");
        // 可以考虑重试或退出
    } else if (!wp_clear_srv.response.success) {
        ROS_WARN("清除现有任务失败 (飞控拒绝)");
    } else {
        ROS_INFO("已清除飞控中的现有任务");
    }
    ros::Duration(1.0).sleep(); // 等待清除完成

    // 上传航点
    mavros_msgs::WaypointPush wp_push_srv;
    wp_push_srv.request.start_index = 0; // 从索引0开始上传
    wp_push_srv.request.waypoints = waypoint_list;

    ROS_INFO("正在上传包含 %zu 个航点的任务...", waypoint_list.size());
    if (!wp_push_client.call(wp_push_srv)) {
        ROS_ERROR("调用 mission/push 服务失败");
        return -1;
    }
    if (!wp_push_srv.response.success) {
        ROS_ERROR("任务上传失败 (飞控拒绝)。原因代码: %u. 请检查航点数据和飞控状态！", wp_push_srv.response.wp_transfer_sent);
        return -1;
    }
    ROS_INFO("成功上传 %u 个航点到飞控", wp_push_srv.response.wp_transfer_sent);


    // 解锁无人机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time(0); // 用于控制请求频率

    ROS_INFO("正在解锁无人机...");
    ros::Time arm_start_time = ros::Time::now();
    while (ros::ok() && !current_state.armed) {
        if (current_state.mode != "AUTO.MISSION" && current_state.mode != "OFFBOARD" && current_state.mode != "GUIDED") {
            // 在某些模式下解锁可能被阻止，这里可以尝试先设置模式
            // 但通常建议先解锁再设置模式
        }
        if(ros::Time::now() - last_request > ros::Duration(1.0)){ // 每隔1秒尝试一次
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO_ONCE("发送解锁指令成功"); // 只打印一次成功发送
            } else if (!current_state.armed) { // 如果还没解锁且调用失败/未成功
                ROS_WARN_THROTTLE(5.0, "发送解锁指令失败或飞控未响应");
            }
            last_request = ros::Time::now();
        }

        // 30秒后超时
        if ((ros::Time::now() - arm_start_time).toSec() > 30.0) {
            ROS_ERROR("解锁无人机超时 (30秒)");
            return -1;
        }

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("无人机已解锁");


    // 将无人机设置为任务模式
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";
    last_request = ros::Time(0);

    ROS_INFO("正在设置为AUTO.MISSION模式...");
    ros::Time mode_start_time = ros::Time::now();
    while (ros::ok() && current_state.mode != "AUTO.MISSION") {
        if(ros::Time::now() - last_request > ros::Duration(1.0)){
            if(set_mode_client.call(auto_set_mode) && auto_set_mode.response.mode_sent)
            {
                ROS_INFO_ONCE("发送 AUTO.MISSION 模式设置指令成功");
            } else if (current_state.mode != "AUTO.MISSION") {
                ROS_WARN_THROTTLE(5.0, "发送模式设置指令失败或飞控未响应");
            }
            last_request = ros::Time::now();
        }

        // 30秒后超时
        if ((ros::Time::now() - mode_start_time).toSec() > 30.0) {
            ROS_ERROR("设置 AUTO.MISSION 模式超时 (30秒)");
            // 可以尝试切换到其他模式如 LOITER 或 LAND
            // mavros_msgs::SetMode loiter_set_mode;
            // loiter_set_mode.request.custom_mode = "AUTO.LOITER";
            // set_mode_client.call(loiter_set_mode);
            return -1;
        }

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("无人机已进入 AUTO.MISSION 模式");


    // 等待任务完成
    ROS_INFO("正在执行任务... 最终航点索引: %d", last_wp_index);
    ros::Time mission_start = ros::Time::now();
    double mission_timeout = 600.0;  // 增加超时时间到10分钟
    bool final_wp_reached_flag = false; // 重命名以区分回调变量
    ros::Time final_wp_reached_time;

    while (ros::ok() && !mission_completed.load()) { // 使用 .load() 获取原子变量的值
        ros::spinOnce();

        // 检查是否到达最后一个航点 (通过回调更新的 current_wp_reached)
        if (!final_wp_reached_flag && current_wp_reached == last_wp_index) {
            final_wp_reached_flag = true;
            final_wp_reached_time = ros::Time::now();
            ROS_INFO("逻辑确认：已到达最终航点 %d，开始等待 %.1f 秒延迟...", last_wp_index, last_wp_delay);
        }

        // 如果到达最后一个航点且延迟时间已过，则标记任务完成
        if (final_wp_reached_flag && last_wp_delay > 0.0 && // 只有当需要延迟时才检查
            (ros::Time::now() - final_wp_reached_time).toSec() > last_wp_delay) {
            ROS_INFO("最终航点延迟等待完成。");
            mission_completed.store(true); // 明确在此设置完成状态
        } else if (final_wp_reached_flag && last_wp_delay <= 0.0) {
            // 如果最后一个航点不需要延迟，到达即完成
            mission_completed.store(true);
            ROS_INFO("已到达最终航点且无需等待延迟，任务完成。");
        }

        // 检查飞控是否自己切换到了悬停模式 (也表示任务完成)
        if (!mission_completed.load() && current_state.mode == "AUTO.LOITER") {
            ROS_WARN("检测到飞控自动切换到 AUTO.LOITER 模式，认为任务完成。");
            mission_completed.store(true);
        }

        // 检查是否超时
        if (!mission_completed.load() && (ros::Time::now() - mission_start).toSec() > mission_timeout) {
            ROS_ERROR("任务超时 (%.1f 秒)。最后到达的航点: %d。当前模式: %s",
                    mission_timeout, current_wp_reached, current_state.mode.c_str());
            // 超时后也应该尝试RTL或Land
            break; // 跳出循环去执行RTL
        }

        ROS_INFO_THROTTLE(5.0, "任务进行中... 当前到达航点: %d / %d, 当前模式: %s",
                        current_wp_reached, last_wp_index, current_state.mode.c_str());

        rate.sleep();
    }

    // ... (RTL、清除任务、锁定的代码保持不变，但可以增加健壮性) ...
    if (mission_completed.load()) {
        ROS_INFO("任务逻辑完成，准备返回起飞点 (RTL)...");
    } else {
        ROS_WARN("任务未按预期完成 (可能超时或出错)，仍然尝试返回起飞点 (RTL)...");
    }

    // 切换到RTL模式
    mavros_msgs::SetMode rtl_set_mode;
    rtl_set_mode.request.custom_mode = "AUTO.RTL";
    last_request = ros::Time(0);
    ros::Time rtl_start_time = ros::Time::now();
    bool rtl_success = false;

    while(ros::ok() && (ros::Time::now() - rtl_start_time).toSec() < 30.0) { // 尝试30秒设置RTL模式
        if (current_state.mode == "AUTO.RTL") {
            ROS_INFO("成功切换到 AUTO.RTL 模式");
            rtl_success = true;
            break;
        }
        if(ros::Time::now() - last_request > ros::Duration(1.0)){
            if(set_mode_client.call(rtl_set_mode) && rtl_set_mode.response.mode_sent) {
                ROS_INFO_ONCE("发送 RTL 模式设置指令成功");
            } else {
                ROS_WARN_THROTTLE(5.0, "发送 RTL 模式设置指令失败或飞控未响应");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    if (!rtl_success) {
        ROS_ERROR("设置 RTL 模式失败，请手动控制！");
        // 这里可能需要采取更紧急的措施，或者至少持续警告
    } else {
        // 等待RTL完成
        ros::Time rtl_wait_start = ros::Time::now();
        double rtl_timeout = 180.0; // 3分钟 RTL 超时
        ROS_INFO("等待 RTL 完成 (最长 %.1f 秒)...", rtl_timeout);
        while (ros::ok() && (ros::Time::now() - rtl_wait_start).toSec() < rtl_timeout) {
            ros::spinOnce();
            // 更可靠的RTL完成判断：检查是否接近home点并降落，或者是否已锁定
            // 简单的判断：检查模式是否变为 LAND 或已锁定
            if (current_state.mode == "AUTO.LAND") {
                ROS_INFO("无人机已进入 LAND 模式，认为 RTL 接近完成。");
                break;
            }
            if (!current_state.armed) {
                ROS_INFO("无人机已锁定，认为 RTL 完成。");
                break;
            }
            // 检查是否又切回 LOITER (可能在降落点上方悬停)
            if (current_state.mode == "AUTO.LOITER" && (ros::Time::now() - rtl_start_time).toSec() > 10.0) { // 切换到RTL后一段时间又变回LOITER
                ROS_INFO("无人机切换回 LOITER 模式，可能已到达返航点上方，认为 RTL 完成。");
                break;
            }

            ROS_INFO_THROTTLE(5.0, "无人机正在返航，当前模式: %s", current_state.mode.c_str());
            rate.sleep();
        }
        if ((ros::Time::now() - rtl_wait_start).toSec() >= rtl_timeout) {
            ROS_WARN("RTL 等待超时 (%.1f 秒)。当前模式: %s", rtl_timeout, current_state.mode.c_str());
        }
    }


    // 清除任务 (无论如何都尝试清除)
    if (wp_clear_client.call(wp_clear_srv) && wp_clear_srv.response.success) {
        ROS_INFO("已清除飞控中的任务航点");
    } else {
        ROS_ERROR("清除飞控任务失败");
    }

    // 锁定无人机 (如果它还未锁定)
    if (current_state.armed) {
        ROS_INFO("正在锁定无人机...");
        arm_cmd.request.value = false;
        last_request = ros::Time(0);
        ros::Time disarm_start_time = ros::Time::now();
        while(ros::ok() && current_state.armed && (ros::Time::now() - disarm_start_time).toSec() < 15.0) { // 尝试锁定15秒
            if(ros::Time::now() - last_request > ros::Duration(1.0)){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO_ONCE("发送锁定指令成功");
                } else if (current_state.armed) {
                    ROS_WARN_THROTTLE(5.0, "发送锁定指令失败或飞控未响应");
                }
                last_request = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }
        if (current_state.armed) {
            ROS_ERROR("锁定无人机失败！请手动锁定！");
        } else {
            ROS_INFO("无人机已锁定");
        }
    } else {
        ROS_INFO("无人机已经处于锁定状态。");
    }

    ROS_INFO("任务序列执行完毕。");
    return 0;
}