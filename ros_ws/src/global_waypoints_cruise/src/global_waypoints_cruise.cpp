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
    ROS_INFO("Reached waypoint %d", current_wp_reached); 
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
    ROS_INFO("Waiting for FCU connection..."); 
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected.");


    XmlRpc::XmlRpcValue waypoints_param;
    std::vector<mavros_msgs::Waypoint> waypoint_list;
    std::string waypoints_param_name = "waypoints"; 

    // --- 新增：等待参数出现的逻辑 ---
    ROS_INFO("Waiting for parameter '%s' on server...", private_nh.resolveName(waypoints_param_name).c_str());
    // 循环检查参数是否存在，并且ROS没有被关闭
    while (!private_nh.hasParam(waypoints_param_name) && ros::ok()) {
        ros::Duration(0.5).sleep(); // 等待0.5秒再次检查
        // 每隔5秒打印一次提示信息，避免刷屏
        ROS_INFO_THROTTLE(5.0, "Still waiting for parameter '%s'...", private_nh.resolveName(waypoints_param_name).c_str());
    }

    // 检查是否因为ROS关闭而退出了等待循环
    if (!ros::ok()) {
        ROS_WARN("ROS shutdown while waiting for parameters.");
        return -1; // 保持和原代码一致的退出码
    }
    ROS_INFO("Parameter '%s' found on server. Proceeding to load.", private_nh.resolveName(waypoints_param_name).c_str());
    // --- 等待逻辑结束 ---
  


    // --- 从参数服务器读取航点 ---
    

    if (!private_nh.getParam("waypoints", waypoints_param)) {
        ROS_ERROR("Failed to get 'waypoints' parameter from server. Ensure waypoints are defined in launch/YAML file!"); 
        return -1;
    }

    // 检查参数类型是否为数组
    if (waypoints_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'waypoints' parameter must be an array (list) type!"); 
        return -1;
    }

    ROS_INFO("Loading %d waypoints from parameter server...", (int)waypoints_param.size());

    for (int i = 0; i < waypoints_param.size(); ++i) {
        // 检查每个航点是否是结构体（字典）
        if (waypoints_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("Waypoint %d format is not a struct, skipping.", i);
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
            ROS_ERROR("Waypoint %d is missing a valid 'latitude' (double type) field.", i); 
            return -1;
        }

        // 经度 (必需)
        if (wp_data.hasMember("longitude") && wp_data["longitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.y_long = static_cast<double>(wp_data["longitude"]);
        } else {
            ROS_ERROR("Waypoint %d is missing a valid 'longitude' (double type) field.", i); 
            return -1;
        }

        // 相对高度 (必需)
        if (wp_data.hasMember("altitude") && wp_data["altitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            wp.z_alt = static_cast<double>(wp_data["altitude"]);
        } else {
            ROS_ERROR("Waypoint %d is missing a valid 'altitude' (double type) field.", i); 
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
        ROS_INFO("  Waypoint %d: Lat=%.6f, Lon=%.6f, Alt=%.2f, Delay=%.1f, Yaw=%.1f", i, wp.x_lat, wp.y_long, wp.z_alt, wp.param1, wp.param4);
    }

    if (waypoint_list.empty()) {
        ROS_ERROR("Waypoint list loaded from parameter server is empty!"); 
        return -1;
    }
    // --- 航点加载结束 ---


    // 存储最后一个航点的索引和延迟时间
    last_wp_index = waypoint_list.size() - 1;
    if (last_wp_index >= 0) {
        last_wp_delay = waypoint_list.back().param1;
    } else {
        // 这个检查理论上在上面已经覆盖了，但保留无妨
        ROS_ERROR("Failed to load any waypoints!"); 
        return -1;
    }

    // ... (清除航点、上传航点、解锁、设置模式的代码保持不变) ...
    // 首先清除所有现有航点
    mavros_msgs::WaypointClear wp_clear_srv;
    if (!wp_clear_client.call(wp_clear_srv)) {
        ROS_WARN("Call to (mission/clear) service failed."); 
        // 可以考虑重试或退出
    } else if (!wp_clear_srv.response.success) {
        ROS_WARN("Clearing existing mission failed (FCU rejected)."); 
    } else {
        ROS_INFO("Cleared existing mission in FCU.");
    }
    ros::Duration(1.0).sleep(); // 等待清除完成

    // 上传航点
    mavros_msgs::WaypointPush wp_push_srv;
    wp_push_srv.request.start_index = 0; // 从索引0开始上传
    wp_push_srv.request.waypoints = waypoint_list;

    ROS_INFO("Uploading mission with %zu waypoints...", waypoint_list.size()); 
    if (!wp_push_client.call(wp_push_srv)) {
        ROS_ERROR("Call to (mission/push) service failed."); 
        return -1;
    }
    if (!wp_push_srv.response.success) {
        ROS_ERROR("Mission upload failed (FCU rejected). Sent count/reason: %u. Check waypoint data and FCU status!", wp_push_srv.response.wp_transfered);
        return -1;
    }
    ROS_INFO("Successfully uploaded %u waypoints to FCU.", wp_push_srv.response.wp_transfered);


    // 解锁无人机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time(0); // 用于控制请求频率

    ROS_INFO("Arming vehicle..."); 
    ros::Time arm_start_time = ros::Time::now();
    while (ros::ok() && !current_state.armed) {
        if (current_state.mode != "AUTO.MISSION" && current_state.mode != "OFFBOARD" && current_state.mode != "GUIDED") {
            // 在某些模式下解锁可能被阻止，这里可以尝试先设置模式
            // 但通常建议先解锁再设置模式
        }
        if(ros::Time::now() - last_request > ros::Duration(1.0)){ // 每隔1秒尝试一次
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO_ONCE("Arm command sent successfully."); // 只打印一次成功发送
            } else if (!current_state.armed) { // 如果还没解锁且调用失败/未成功
                ROS_WARN_THROTTLE(5.0, "Sending arm command failed or FCU unresponsive."); 
            }
            last_request = ros::Time::now();
        }

        // 30秒后超时
        if ((ros::Time::now() - arm_start_time).toSec() > 30.0) {
            ROS_ERROR("Arming vehicle timed out (30 seconds)."); 
            return -1;
        }

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed."); 


    // 将无人机设置为任务模式
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";
    last_request = ros::Time(0);

    ROS_INFO("Setting to AUTO.MISSION mode...");
    ros::Time mode_start_time = ros::Time::now();
    while (ros::ok() && current_state.mode != "AUTO.MISSION") {
        if(ros::Time::now() - last_request > ros::Duration(1.0)){
            if(set_mode_client.call(auto_set_mode) && auto_set_mode.response.mode_sent)
            {
                ROS_INFO_ONCE("AUTO.MISSION mode set command sent successfully."); 
            } else if (current_state.mode != "AUTO.MISSION") {
                ROS_WARN_THROTTLE(5.0, "Sending mode set command failed or FCU unresponsive."); 
            }
            last_request = ros::Time::now();
        }

        // 30秒后超时
        if ((ros::Time::now() - mode_start_time).toSec() > 30.0) {
            ROS_ERROR("Setting AUTO.MISSION mode timed out (30 seconds)."); 
            // 可以尝试切换到其他模式如 LOITER 或 LAND
            // mavros_msgs::SetMode loiter_set_mode;
            // loiter_set_mode.request.custom_mode = "AUTO.LOITER";
            // set_mode_client.call(loiter_set_mode);
            return -1;
        }

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle in AUTO.MISSION mode.");


    // 等待任务完成
    ROS_INFO("Executing mission... Final waypoint index: %d", last_wp_index); 
    ros::Time mission_start = ros::Time::now();
    double mission_timeout = 3600.0;  // 增加超时时间到10×6分钟
    bool final_wp_reached_flag = false; // 重命名以区分回调变量
    ros::Time final_wp_reached_time;

    while (ros::ok() && !mission_completed.load()) { // 使用 .load() 获取原子变量的值
        ros::spinOnce();

        // 检查是否到达最后一个航点 (通过回调更新的 current_wp_reached)
        if (!final_wp_reached_flag && current_wp_reached == last_wp_index) {
            final_wp_reached_flag = true;
            final_wp_reached_time = ros::Time::now();
            ROS_INFO("Logic confirm: Reached final waypoint %d, starting %.1f second delay...", last_wp_index, last_wp_delay); 
        }

        // 如果到达最后一个航点且延迟时间已过，则标记任务完成
        if (final_wp_reached_flag && last_wp_delay > 0.0 && // 只有当需要延迟时才检查
            (ros::Time::now() - final_wp_reached_time).toSec() > last_wp_delay) {
            ROS_INFO("Final waypoint delay complete."); 
            mission_completed.store(true); // 明确在此设置完成状态
        } else if (final_wp_reached_flag && last_wp_delay <= 0.0) {
            // 如果最后一个航点不需要延迟，到达即完成
            mission_completed.store(true);
            ROS_INFO("Reached final waypoint with no delay, mission complete.");
        }

        // 检查飞控是否自己切换到了悬停模式 (也表示任务完成)
        if (!mission_completed.load() && current_state.mode == "AUTO.LOITER") {
            ROS_WARN("Detected FCU automatically switched to AUTO.LOITER mode, considering mission complete."); 
            mission_completed.store(true);
        }

        // 检查是否超时
        if (!mission_completed.load() && (ros::Time::now() - mission_start).toSec() > mission_timeout) {
            ROS_ERROR("Mission timed out (%.1f seconds). Last reached waypoint: %d. Current mode: %s",
                     mission_timeout, current_wp_reached, current_state.mode.c_str());
            // 超时后也应该尝试RTL或Land
            break; // 跳出循环去执行RTL
        }

        ROS_INFO_THROTTLE(5.0, "Mission in progress... Current waypoint reached: %d / %d, Current mode: %s",
                                current_wp_reached, last_wp_index, current_state.mode.c_str()); 

        rate.sleep();
    }

    // ... (RTL、清除任务、锁定的代码保持不变，但可以增加健壮性) ...
    if (mission_completed.load()) {
        ROS_INFO("Mission logic complete, attempting Return to Launch (RTL)...");
    } else {
        ROS_WARN("Mission did not complete as expected (timeout or error), still attempting Return to Launch (RTL)..."); 
    }

    // 切换到RTL模式
    mavros_msgs::SetMode rtl_set_mode;
    rtl_set_mode.request.custom_mode = "AUTO.RTL";
    last_request = ros::Time(0);
    ros::Time rtl_start_time = ros::Time::now();
    bool rtl_success = false;

    while(ros::ok() && (ros::Time::now() - rtl_start_time).toSec() < 30.0) { // 尝试30秒设置RTL模式
        if (current_state.mode == "AUTO.RTL") {
            ROS_INFO("Successfully switched to AUTO.RTL mode."); 
            rtl_success = true;
            break;
        }
        if(ros::Time::now() - last_request > ros::Duration(1.0)){
            if(set_mode_client.call(rtl_set_mode) && rtl_set_mode.response.mode_sent) {
                ROS_INFO_ONCE("RTL mode set command sent successfully.");
            } else {
                ROS_WARN_THROTTLE(5.0, "Sending RTL mode set command failed or FCU unresponsive."); 
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    if (!rtl_success) {
        ROS_ERROR("Failed to set RTL mode, please take manual control!"); 
        // 这里可能需要采取更紧急的措施，或者至少持续警告
    } else {
        // 等待RTL完成
        ros::Time rtl_wait_start = ros::Time::now();
        double rtl_timeout = 180.0; // 3分钟 RTL 超时
        ROS_INFO("Waiting for RTL to complete (max %.1f seconds)...", rtl_timeout);
        while (ros::ok() && (ros::Time::now() - rtl_wait_start).toSec() < rtl_timeout) {
            ros::spinOnce();
            // 更可靠的RTL完成判断：检查是否接近home点并降落，或者是否已锁定
            // 简单的判断：检查模式是否变为 LAND 或已锁定
            if (current_state.mode == "AUTO.LAND") {
                ROS_INFO("Vehicle entered LAND mode, assuming RTL nearing completion."); 
                break;
            }
            if (!current_state.armed) {
                ROS_INFO("Vehicle disarmed, assuming RTL complete.");
                break;
            }
            // 检查是否又切回 LOITER (可能在降落点上方悬停)
            if (current_state.mode == "AUTO.LOITER" && (ros::Time::now() - rtl_start_time).toSec() > 10.0) { // 切换到RTL后一段时间又变回LOITER
                ROS_INFO("Vehicle switched back to LOITER mode, possibly above home, assuming RTL complete.");
                break;
            }

            ROS_INFO_THROTTLE(5.0, "Vehicle returning to launch, Current mode: %s", current_state.mode.c_str()); 
            rate.sleep();
        }
        if ((ros::Time::now() - rtl_wait_start).toSec() >= rtl_timeout) {
            ROS_WARN("RTL wait timed out (%.1f seconds). Current mode: %s", rtl_timeout, current_state.mode.c_str()); 
        }
    }


    // 清除任务 (无论如何都尝试清除)
    if (wp_clear_client.call(wp_clear_srv) && wp_clear_srv.response.success) {
        ROS_INFO("Cleared mission waypoints from FCU."); 
    } else {
        ROS_ERROR("Failed to clear mission from FCU.");
    }

    // 锁定无人机 (如果它还未锁定)
    if (current_state.armed) {
        ROS_INFO("Disarming vehicle..."); 
        arm_cmd.request.value = false;
        last_request = ros::Time(0);
        ros::Time disarm_start_time = ros::Time::now();
        while(ros::ok() && current_state.armed && (ros::Time::now() - disarm_start_time).toSec() < 15.0) { // 尝试锁定15秒
            if(ros::Time::now() - last_request > ros::Duration(1.0)){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO_ONCE("Disarm command sent successfully."); 
                } else if (current_state.armed) {
                    ROS_WARN_THROTTLE(5.0, "Sending disarm command failed or FCU unresponsive."); 
                }
                last_request = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }
        if (current_state.armed) {
            ROS_ERROR("Failed to disarm vehicle! Please disarm manually!"); 
        } else {
            ROS_INFO("Vehicle disarmed.");
        }
    } else {
        ROS_INFO("Vehicle is already disarmed."); 
    }

    ROS_INFO("Mission sequence finished.");
    return 0;
}