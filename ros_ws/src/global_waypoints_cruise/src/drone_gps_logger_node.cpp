// drone_gps_logger_node.cpp
// 从MAVROS订阅GPS信息，并通过MQTT发送到MySQL数据库

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mosquitto.h>
#include <string>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <jsoncpp/json/json.h>  // 使用jsoncpp库
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>

#include <serial/serial.h>
#include <vector>
#include <algorithm> // for std::remove_if

#define HOST "192.168.137.1"

// MQTT相关变量
struct mosquitto *mosq = NULL;
std::string mqtt_topic_send;
int vehicle_id;

//串口相关变量 
serial::Serial ser;
std::string serial_port;
int serial_baudrate;
std::string csv_delimiter; // CSV分隔符，默认为逗号

// 格式化时间戳为MySQL格式
std::string formatTimestamp(const ros::Time& time) {
    std::time_t t = time.sec;
    std::tm* tm = std::gmtime(&t);
    
    std::stringstream ss;
    ss << std::put_time(tm, "%Y-%m-%d %H:%M:%S");
    
    return ss.str();
}

// GPS回调函数
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // 检查GPS信息有效性
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_THROTTLE(10, "No GPS fix available, not logging position");
        return;
    }
    
    // 使用jsoncpp准备GPS数据
    Json::Value gps_data;
    gps_data["action"] = "insert_gps";
    gps_data["vehicle_id"] = vehicle_id;
    //gps_data["time_stamp"] = formatTimestamp(msg->header.stamp);
    gps_data["latitude"] = msg->latitude;
    gps_data["longitude"] = msg->longitude;
    gps_data["altitude"] = msg->altitude;
    
    Json::FastWriter writer;
    std::string gps_msg = writer.write(gps_data);
    
    // 发送到MQTT
    int rc = mosquitto_publish(mosq, NULL, mqtt_topic_send.c_str(), 
                             gps_msg.length(), gps_msg.c_str(), 0, false);
    if(rc == MOSQ_ERR_SUCCESS){
        ROS_INFO("Succeed to publish GPS data: %s", gps_msg.c_str());
    } else if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Failed to publish GPS data: %s", mosquitto_strerror(rc));
    } else {
        ROS_DEBUG("Published GPS data: lat=%f, lon=%f, alt=%f", 
                 msg->latitude, msg->longitude, msg->altitude);
    }
}

// --- 解析CSV行的函数 ---
std::vector<std::string> parseCSVLine(const std::string& line, char delimiter = ',') {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        // 移除可能的换行符或回车符
         token.erase(std::remove_if(token.begin(), token.end(), [](char c){return c == '\r' || c == '\n';}), token.end());
        tokens.push_back(token);
    }
    return tokens;
}

// --- 新增：读取并处理串口数据的函数 ---
    
void readAndPublishEnvironmentData(const ros::TimerEvent& event) {
    if (!ser.isOpen()) {
        ROS_WARN("Serial port is not open. Attempting to reconnect...");
        try {
            ser.open(); // 尝试重新打开
             if(ser.isOpen()){
                 ROS_INFO("Serial port %s re-opened successfully.", serial_port.c_str());
             }
        } catch (serial::IOException& e) {
            ROS_ERROR("Failed to re-open serial port: %s", e.what());
            return;
        }
        if(!ser.isOpen()) {
             ROS_ERROR("Serial port %s is still not open.", serial_port.c_str());
             return;
        }
    }

    std::string line;
    try {
        // 读取一行数据
        line = ser.readline(); // 默认以 '\n' 结尾
    } catch (serial::IOException& e) {
        ROS_ERROR("Error reading from serial port: %s", e.what());
        // 可以选择关闭串口，下次定时器触发时尝试重连
        ser.close();
        return;
    } catch (serial::PortNotOpenedException& e) {
         ROS_ERROR("Serial port not open when trying to read: %s", e.what());
         return;}

    if (line.empty()) {
        // ROS_DEBUG("No data received from serial port.");
        return; // 没有数据可读
    }


    ROS_DEBUG("Received serial data: %s", line.c_str());

    // 解析CSV
    std::vector<std::string> data_fields = parseCSVLine(line, csv_delimiter.empty() ? ',' : csv_delimiter[0]);

    // 基本验证：需要10个字段
    if (data_fields.size() != 10) {
        ROS_WARN("Received CSV line with incorrect number of fields (%zu), expected 10. Line: %s", data_fields.size(), line.c_str());
        return;
    }

    // --- 根据CSV格式解析字段 ---
    // 0: 温度(°C), 1: 湿度(%), 2: 光照(lux), 3: 甲醛(ppm), 4: TVOC(ppb), 5: CO2(ppm),
    // 6: 烟雾(*1000), 7: 温度2(*100), 8: 气压(*1000), 9: 海拔(*100)
    try {
        // 假设原始数据是整数或浮点数，直接转换
        float temperature = std::stof(data_fields.at(0));
        float humidity = std::stof(data_fields.at(1));
        float illuminance = std::stof(data_fields.at(2));
        float formaldehyde = std::stof(data_fields.at(3));
        float tvoc = std::stof(data_fields.at(4));
        float co2 = std::stof(data_fields.at(5));
        // 对于缩放过的数据，需要反向缩放
        float smoke = std::stof(data_fields.at(6)) / 1000.0f;
        float temperature2 = std::stof(data_fields.at(7)) / 100.0f;
        float pressure = std::stof(data_fields.at(8)) / 1000.0f;
        float altitude = std::stof(data_fields.at(9)) / 100.0f;

        // 使用jsoncpp准备环境数据
        Json::Value env_data;
        env_data["action"] = "insert_environment"; // 新的 action 类型
        env_data["vehicle_id"] = vehicle_id;
        env_data["temperature"] = temperature;
        env_data["humidity"] = humidity;
        env_data["illuminance"] = illuminance;
        env_data["formaldehyde"] = formaldehyde;
        env_data["tvoc"] = tvoc;
        env_data["co2"] = co2;
        env_data["smoke"] = smoke;
        env_data["temperature2"] = temperature2;
        env_data["pressure"] = pressure;
        env_data["altitude"] = altitude;

        Json::FastWriter writer;
        std::string env_msg = writer.write(env_data);

        // 发送到MQTT
        int rc = mosquitto_publish(mosq, NULL, mqtt_topic_send.c_str(),
                                 env_msg.length(), env_msg.c_str(), 0, false);
        if(rc == MOSQ_ERR_SUCCESS){
            ROS_INFO("Succeed to publish Environment data: %s", env_msg.c_str());
        } else {
            ROS_ERROR("Failed to publish Environment data: %s", mosquitto_strerror(rc));
        }
    } catch (const std::invalid_argument& e) {
        ROS_ERROR("Failed to convert CSV fields to float. Invalid data? Line: %s", line.c_str());
    } catch (const std::out_of_range& e) {
         ROS_ERROR("Float conversion out of range. Line: %s", line.c_str());
    } catch (const std::exception& e) {
        // 捕获其他可能的 Json::Exception 等
        ROS_ERROR("Error processing environment data: %s. Line: %s", e.what(), line.c_str());
    }

}

// MQTT连接回调
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        ROS_INFO("Connected to MQTT broker");
    } else {
        ROS_ERROR("Failed to connect to MQTT broker, return code: %d", rc);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_gps_logger");
    ros::NodeHandle nh("~");
    
    // 获取参数
    std::string mqtt_broker;
    int mqtt_port;
    std::string gps_topic;
    double publish_rate;
    
    nh.param<std::string>("mqtt_broker", mqtt_broker, HOST);
    nh.param<int>("mqtt_port", mqtt_port, 8899);
    nh.param<std::string>("mqtt_topic_send", mqtt_topic_send, "ros/to/mysql");
    nh.param<std::string>("gps_topic", gps_topic, "/mavros/global_position/global");
    nh.param<int>("vehicle_id", vehicle_id, 1001);
    nh.param<double>("publish_rate", publish_rate, 1.0); // Hz

    // --- 新增：获取串口参数 ---
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyS2"); // 默认串口
    nh.param<int>("serial_baudrate", serial_baudrate, 115200);           // 默认波特率
    nh.param<std::string>("csv_delimiter", csv_delimiter, ",");       // 默认分隔符
    
    // 初始化Mosquitto库
    mosquitto_lib_init();
    
    // 创建MQTT客户端实例
    mosq = mosquitto_new("ros_drone_gps_logger", true, NULL);
    if (!mosq) {
        ROS_ERROR("Failed to create MQTT client instance");
        mosquitto_lib_cleanup();
        return 1;
    }
    
    // 设置连接回调
    mosquitto_connect_callback_set(mosq, on_connect);
    
    // 连接到MQTT broker
    int rc = mosquitto_connect(mosq, mqtt_broker.c_str(), mqtt_port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Unable to connect to MQTT broker: %s", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }
    
    // 启动MQTT网络循环
    rc = mosquitto_loop_start(mosq);
    if (rc != MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Failed to start MQTT network loop: %s", mosquitto_strerror(rc));
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // --- 新增：初始化并打开串口 ---
    try {
        ser.setPort(serial_port);
        ser.setBaudrate(serial_baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 1秒超时
        ser.setTimeout(timeout);
        ser.open();
        if(ser.isOpen()) {
            ROS_INFO("Serial port %s opened successfully at %d baud.", serial_port.c_str(), serial_baudrate);
        } else {
             ROS_ERROR("Failed to open serial port %s.", serial_port.c_str());
             // 注意：这里可以选择退出或继续运行（但环境数据将不可用）
             // return 1; // 如果串口是必须的，可以退出
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("Failed to open serial port %s: %s", serial_port.c_str(), e.what());
        // 注意：这里可以选择退出或继续运行（但环境数据将不可用）
        // return 1; // 如果串口是必须的，可以退出
    }
    
    // 订阅GPS话题
    ros::Subscriber gps_sub;
    if (publish_rate <= 0) {
        // 直接订阅，不限制频率
        gps_sub = nh.subscribe(gps_topic, 10, gpsCallback);
    } else {
        // 使用节流器限制发布频率
        // 注意这里使用 subscribe<sensor_msgs::NavSatFix> 显式指定消息类型
        gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 10, 
            [publish_rate](const sensor_msgs::NavSatFix::ConstPtr& msg) {
                static ros::Time last_pub_time = ros::Time(0);
                ros::Time now = ros::Time::now();
                
                if ((now - last_pub_time).toSec() >= 2.0/publish_rate) {
                    gpsCallback(msg);
                    last_pub_time = now;
                }
            });
    }
    
    // --- 新增：创建ROS定时器来定期读取串口数据 ---
    // 例如，每2秒读取一次 (0.5 Hz)
    ros::Timer env_timer = nh.createTimer(ros::Duration(2.0), readAndPublishEnvironmentData);

    // 主循环
    ros::spin();
    
    // 清理
    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    return 0;
}
