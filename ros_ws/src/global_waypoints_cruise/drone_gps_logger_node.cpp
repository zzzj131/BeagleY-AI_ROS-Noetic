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

// MQTT相关变量
struct mosquitto *mosq = NULL;
std::string mqtt_topic_send;
int vehicle_id;

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
    
    nh.param<std::string>("mqtt_broker", mqtt_broker, "192.168.31.81");
    nh.param<int>("mqtt_port", mqtt_port, 8899);
    nh.param<std::string>("mqtt_topic_send", mqtt_topic_send, "ros/to/mysql");
    nh.param<std::string>("gps_topic", gps_topic, "/mavros/global_position/global");
    nh.param<int>("vehicle_id", vehicle_id, 1001);
    nh.param<double>("publish_rate", publish_rate, 1.0); // Hz
    
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
    
    // 主循环
    ros::spin();
    
    // 清理
    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    return 0;
}