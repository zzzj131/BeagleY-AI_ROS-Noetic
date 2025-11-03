// waypoints_loader_node.cpp
// 从MySQL数据库读取航点信息，并写入ROS参数服务器

#include <ros/ros.h>
#include <mosquitto.h>
// #include <yaml-cpp/yaml.h> //不再需要YAML库进行设置
// 可能仍用于其他目的，但不再用于YAML转换
#include <string>
#include <sstream> 
#include <vector>
#include <iostream>
#include <map> // 需要包含 map
#include <jsoncpp/json/json.h>  // 使用jsoncpp库
#include <XmlRpcValue.h>

// 航点结构 (这个结构用于从JSON解析，是OK的)
struct Waypoint {
    double latitude;
    double longitude;
    double altitude;
    double delay;
};

// MQTT回调相关变量和结构体
struct MQTTContext {
    bool response_received = false;
    std::string response_payload; // 保留用于调试或完整响应记录
    std::vector<Waypoint> waypoints; // 用于存储解析后的航点
};

// MQTT消息回调 (这部分解析JSON到 context.waypoints)
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    MQTTContext* context = static_cast<MQTTContext*>(obj);
    
    // 处理收到的消息
    std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
    ROS_INFO("Received MQTT message via callback."); // 添加日志确认回调被触发
    
    // 使用jsoncpp解析JSON响应
    Json::Value root;
    // 推荐使用较新的 CharReaderBuilder API
    Json::CharReaderBuilder readerBuilder;
    std::unique_ptr<Json::CharReader> reader(readerBuilder.newCharReader());
    std::string errs;
    bool success = reader->parse(payload.c_str(), payload.c_str() + payload.length(), &root, &errs);

    // (旧的 Reader API)
    // Json::Reader reader;
    // bool success = reader.parse(payload, root);
    
    if (success) {
        if (root.isMember("status") && root["status"].asString() == "success" && root.isMember("waypoints") && root["waypoints"].isArray()) {
            ROS_INFO("Received success status with waypoints array.");
            context->response_payload = payload; // 存储原始负载
            context->response_received = true;   // 标记收到有效响应
            
            // 清空旧的航点，以防重复调用或其他情况
            context->waypoints.clear(); 

            // 解析航点列表
            const Json::Value& waypoints_json_array = root["waypoints"];
            ROS_INFO("Parsing %d waypoints from JSON.", waypoints_json_array.size());
            for (const auto& wp_json : waypoints_json_array) { // 使用 range-based for 提高可读性
                if (!wp_json.isObject()) {
                     ROS_WARN("Waypoint item is not an object, skipping.");
                     continue;
                }
                Waypoint wp;
                bool waypoint_valid = true;

                if (wp_json.isMember("latitude") && wp_json["latitude"].isNumeric()) {
                    wp.latitude = wp_json["latitude"].asDouble();
                } else {
                     ROS_WARN("Waypoint missing or non-numeric 'latitude', skipping waypoint.");
                     waypoint_valid = false;
                }

                if (wp_json.isMember("longitude") && wp_json["longitude"].isNumeric()) {
                    wp.longitude = wp_json["longitude"].asDouble();
                } else {
                     ROS_WARN("Waypoint missing or non-numeric 'longitude', skipping waypoint.");
                     waypoint_valid = false;
                }

                if (wp_json.isMember("altitude") && wp_json["altitude"].isNumeric()) {
                    wp.altitude = wp_json["altitude"].asDouble();
                } else {
                     ROS_WARN("Waypoint missing or non-numeric 'altitude', skipping waypoint.");
                     waypoint_valid = false;
                }

                // 延迟可能为空或非数字，设置默认值
                if (wp_json.isMember("delay") && wp_json["delay"].isNumeric()) {
                    wp.delay = wp_json["delay"].asDouble();
                } else {
                    // 如果字段存在但类型错误，给个警告
                    if (wp_json.isMember("delay")){
                        ROS_WARN("Waypoint 'delay' is non-numeric, using default 0.0.");
                    } else {
                        ROS_DEBUG("Waypoint missing 'delay', using default 0.0."); // Debug级别，可能比较常见
                    }
                    wp.delay = 0.0;
                }
                
                if (waypoint_valid) {
                    context->waypoints.push_back(wp);
                    ROS_DEBUG("Parsed waypoint: lat=%.6f, lon=%.6f, alt=%.2f, delay=%.1f", // 使用 DEBUG 避免刷屏
                             wp.latitude, wp.longitude, wp.altitude, wp.delay);
                }
            }
            ROS_INFO("Finished parsing. Stored %zu valid waypoints.", context->waypoints.size());

        } else if (root.isMember("status") && root["status"].asString() == "error") {
            ROS_ERROR("Error response from server: %s", 
                     root.isMember("message") ? root["message"].asCString() : "Unknown error");
            context->response_received = true; // 收到错误响应也是一种响应
            context->waypoints.clear(); // 确保出错时不保留旧航点
        } else {
             ROS_WARN("Received JSON, but status is not 'success' with waypoints array, or not 'error'. Payload: %s", payload.c_str());
             // 可以选择是否将这种情况视为“已接收响应”
             // context->response_received = true; 
        }
    } else {
        ROS_ERROR("Failed to parse received JSON: %s", errs.c_str());
        // 如果使用旧 API: ROS_ERROR("Failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());
        // 解析失败时，可能不应标记为 response_received=true，除非你想让程序继续执行而不设置参数
    }
}

// MQTT连接回调
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        ROS_INFO("Connected to MQTT broker");
        // 订阅响应主题
        // 注意: 使用参数中的 mqtt_topic_receive
        MQTTContext* context = static_cast<MQTTContext*>(obj); // 需要获取上下文来访问主题参数（虽然这里没直接用，但逻辑上应一致）
        std::string topic_to_subscribe = "mysql/to/ros"; // 硬编码了，最好也用参数
        // 实际上，我们应该在 main 函数获取参数后，再设置订阅主题，或者将主题存入 context
        // 这里暂时保持硬编码，与原代码一致
        int sub_rc = mosquitto_subscribe(mosq, NULL, topic_to_subscribe.c_str(), 0);
        if (sub_rc == MOSQ_ERR_SUCCESS) {
             ROS_INFO("Successfully subscribed to topic: %s", topic_to_subscribe.c_str());
        } else {
             ROS_ERROR("Failed to subscribe to topic %s: %s", topic_to_subscribe.c_str(), mosquitto_strerror(sub_rc));
        }

    } else {
        ROS_ERROR("Failed to connect to MQTT broker, return code: %d (%s)", rc, mosquitto_strerror(rc));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoints_loader");
    ros::NodeHandle nh("~"); // 使用私有 NodeHandle 获取参数更符合 ROS 规范
    
    // 获取参数
    std::string mqtt_broker;
    int mqtt_port;
    std::string mqtt_topic_send;
    std::string mqtt_topic_receive; // 读取这个参数，但上面的 on_connect 回调没用它
    int vehicle_id;
    std::string param_name;
    double response_timeout;
    
    // 使用 private_nh 获取参数，这样参数需要放在节点的私有命名空间下
    // 例如，在 launch 文件中用 <param name="mqtt_broker" value="..."/>
    ros::NodeHandle private_nh("~"); 
    private_nh.param<std::string>("mqtt_broker", mqtt_broker, "192.168.31.81");
    private_nh.param<int>("mqtt_port", mqtt_port, 8899);
    private_nh.param<std::string>("mqtt_topic_send", mqtt_topic_send, "ros/to/mysql");
    private_nh.param<std::string>("mqtt_topic_receive", mqtt_topic_receive, "mysql/to/ros"); // 读取了但没在 subscribe 时用
    private_nh.param<int>("vehicle_id", vehicle_id, 1001);
    private_nh.param<std::string>("param_name", param_name, "/global_waypoints_cruise/waypoints"); // 目标参数名
    private_nh.param<double>("response_timeout", response_timeout, 10.0); // 设置超时时间参数

    ROS_INFO("Waypoint Loader Parameters:");
    ROS_INFO(" - MQTT Broker: %s", mqtt_broker.c_str());
    ROS_INFO(" - MQTT Port: %d", mqtt_port);
    ROS_INFO(" - MQTT Send Topic: %s", mqtt_topic_send.c_str());
    ROS_INFO(" - MQTT Receive Topic: %s", mqtt_topic_receive.c_str()); // 确认参数被读取
    ROS_INFO(" - Vehicle ID: %d", vehicle_id);
    ROS_INFO(" - Target Parameter Name: %s", param_name.c_str());
    ROS_INFO(" - Response Timeout: %.1f seconds", response_timeout);

    
    // 初始化Mosquitto库
    mosquitto_lib_init();
    
    // 创建上下文
    MQTTContext context;
    
    // 创建MQTT客户端实例
    struct mosquitto *mosq = mosquitto_new(NULL, true, &context); // Client ID 可以为 NULL，让库自动生成
    if (!mosq) {
        ROS_ERROR("Failed to create MQTT client instance");
        mosquitto_lib_cleanup();
        return 1;
    }
    
    // 设置回调
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);
    
    // 连接到MQTT broker
    ROS_INFO("Connecting to MQTT broker %s:%d...", mqtt_broker.c_str(), mqtt_port);
    int rc = mosquitto_connect(mosq, mqtt_broker.c_str(), mqtt_port, 60); // 60秒 keepalive
    if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Unable to connect to MQTT broker: %s", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }
    
    // 启动MQTT网络循环 (使用非阻塞的 loop_start)
    rc = mosquitto_loop_start(mosq);
    if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Failed to start MQTT network loop: %s", mosquitto_strerror(rc));
        // 不直接断开，尝试继续后面的逻辑，但可能收不到消息
        // mosquitto_disconnect(mosq); // 如果 loop 启动失败，连接可能已经有问题
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }
    
    // 等待连接建立 和 订阅完成 (给一点时间，更健壮的方式是在 on_connect 中设置标志)
    ros::Duration(1.5).sleep(); // 稍微增加等待时间
    
    // 使用jsoncpp准备查询消息
    Json::Value query;
    query["action"] = "query_waypoints";
    query["vehicle_id"] = vehicle_id;
    
    // 推荐使用 StreamBuilderWriter 获取更精确的 JSON (FastWriter 可能不符合严格标准)
    Json::StreamWriterBuilder writerBuilder;
    // writerBuilder.settings_["indentation"] = ""; // 不需要缩进，紧凑格式
    std::string query_msg = Json::writeString(writerBuilder, query);
    
    // (旧的 FastWriter API)
    // Json::FastWriter writer;
    // std::string query_msg = writer.write(query);
    
    // 发送查询
    ROS_INFO("Sending query to topic '%s': %s", mqtt_topic_send.c_str(), query_msg.c_str());
    rc = mosquitto_publish(mosq, NULL, mqtt_topic_send.c_str(), 
                         query_msg.length(), query_msg.c_str(), 0, false); // QoS 0
    if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Failed to publish message: %s", mosquitto_strerror(rc));
        // 即使发布失败，也可能收到旧消息，所以继续等待？或者直接退出？
        // 决定继续等待，但后续设置参数会失败
    }
    
    // 等待响应，使用参数设置的超时时间
    ROS_INFO("Waiting for response on topic '%s' (timeout: %.1f s)...", mqtt_topic_receive.c_str(), response_timeout);
    ros::Time start_time = ros::Time::now();
    // 使用 ros::ok() 检查 ROS 是否关闭
    while (!context.response_received && ros::ok() &&
           (ros::Time::now() - start_time).toSec() < response_timeout) 
    {
        ros::spinOnce(); // 处理 ROS 回调，虽然这里主要是 MQTT 回调在工作
        ros::Duration(0.1).sleep(); // 避免 CPU 忙等待
    }
    
    // 检查是否因为 ROS 关闭而退出循环
    if (!ros::ok()) {
         ROS_WARN("ROS shutdown during waiting for MQTT response.");
         // 执行清理并退出
         mosquitto_loop_stop(mosq, true);
         mosquitto_disconnect(mosq);
         mosquitto_destroy(mosq);
         mosquitto_lib_cleanup();
         return 0; // 或者返回错误码
    }

    if (!context.response_received) {
        ROS_ERROR("No valid response received from MQTT within timeout period (%.1f s).", response_timeout);
    } else if (context.waypoints.empty()) {
        ROS_WARN("Response received, but no valid waypoints were parsed or the list was empty.");
        // 设置空列表
        ROS_INFO("Setting an empty waypoint list to parameter '%s'", param_name.c_str());
        // <<< MODIFICATION START >>> 设置空列表也需要手动转换
        XmlRpc::XmlRpcValue empty_waypoints_param;
        empty_waypoints_param.setSize(0); // 创建一个空数组
        try {
             ros::param::set(param_name, empty_waypoints_param);
        } catch (const ros::Exception& e) { // <<< MODIFICATION: 使用 ros::param::...
             ROS_ERROR("ROS InvalidParameterTypeException while setting empty list for '%s': %s", param_name.c_str(), e.what());
        } catch (const std::exception& e) {
             ROS_ERROR("Standard exception while setting empty list for '%s': %s", param_name.c_str(), e.what());
        } catch (...) {
             ROS_ERROR("Unknown exception while setting empty list for '%s'", param_name.c_str());
        }

    } else {
        // 收到响应并且成功解析出航点
        ROS_INFO("Response received with %zu waypoints. Setting ROS parameter...", context.waypoints.size());

        // <<< MODIFICATION START >>>
        // 手动构建 XmlRpcValue 对象
        XmlRpc::XmlRpcValue waypoints_param_value; 
        waypoints_param_value.setSize(context.waypoints.size()); // 设置为数组类型，并指定大小

        for (size_t i = 0; i < context.waypoints.size(); ++i) {
            const auto& wp = context.waypoints[i];
            XmlRpc::XmlRpcValue waypoint_struct; // 每个航点是一个 Struct 类型
            waypoint_struct["latitude"] = wp.latitude;   // XmlRpcValue 可以直接赋值 double
            waypoint_struct["longitude"] = wp.longitude;
            waypoint_struct["altitude"] = wp.altitude;
            waypoint_struct["delay"] = wp.delay;
            // 如果有其他字段如 yaw，也在这里添加
            // waypoint_struct["yaw"] = wp.yaw; 

            waypoints_param_value[i] = waypoint_struct; // 将构建好的 Struct 放入数组的对应位置
        }

        ROS_INFO("Attempting to set ROS parameter '%s' with constructed XmlRpcValue (TypeArray)...", 
                 param_name.c_str());

        // 直接将构建好的 XmlRpcValue 对象设置到参数服务器
        try {
            ros::param::set(param_name, waypoints_param_value); // <<< MODIFICATION: 传递 XmlRpcValue 对象

            ROS_INFO("Successfully loaded %zu waypoints into ROS parameter server '%s'.",
                     context.waypoints.size(), param_name.c_str());

        } catch (const ros::Exception& e) { // <<< MODIFICATION: 使用 ros::param::...
            // 捕获可能的异常 (理论上我们构建了正确的类型，不太会触发这个)
            ROS_ERROR("ROS InvalidParameterTypeException while setting parameter '%s': %s", param_name.c_str(), e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("Standard exception while setting parameter '%s': %s", param_name.c_str(), e.what());
        } catch (...) {
            ROS_ERROR("Unknown exception while setting parameter '%s'", param_name.c_str());
        }
        // <<< MODIFICATION END >>>
    }
    
    // 清理
    ROS_INFO("Stopping MQTT loop and disconnecting...");
    mosquitto_loop_stop(mosq, true); // true 表示等待当前操作完成
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    ROS_INFO("Waypoints loader node finished.");
    return 0; // 正常结束
}