//包含ROS和MAVROS相关头文件 
#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>
#include <geometry_msgs/Twist.h>

#define ALTITUDE  2 // 定义飞行高度常量

int flag  = 1;   // 航点状态标志

mavros_msgs::State current_state; // 无人机状态变量
void state_cb(const mavros_msgs::State::ConstPtr& msg);// 状态回调函数声明

//定义变量，用于接收无人机的里程计信息
// 定义坐标系转换变量
tf::Quaternion quat; 
double roll, pitch, yaw;  // 欧拉角

// 起飞初始位置记录
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false; // 初始化标志

// 本地位置信息
nav_msgs::Odometry local_pos;

// 本地位置回调函数声明
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);

// 状态回调函数实现
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;// 更新无人机状态
}

//回调函数接收无人机的里程计信息
// 本地位置回调函数实现
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg; // 更新本地位置

	 // 记录初始起飞位置（仅第一次有效）
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;// 标记已初始化    		    
    }
	// 将四元数转换为欧拉角
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}


int main(int argc, char **argv)
{
	// ROS节点初始化
    ros::init(argc, argv, "offboard_multi_position");
    ros::NodeHandle nh;

	// 订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

	// 发布目标位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
	 // 订阅本地位置
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
	
	// 初始化服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
	// 设置控制频率2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
	// 等待FCU连接
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

	// 初始化目标位置（相对起飞点）
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;
     

    //send a few setpoints before starting
	// 预热：发送100个初始位置设定点
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

	// 准备Offboard模式请求
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 准备解锁请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
 
 	//此处满足一次请求进入offboard模式即可，官方例成循环切入offboard会导致无人机无法使用遥控器控制
	// 主控制循环：进入Offboard模式并解锁
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
		// 尝试进入Offboard模式（每5秒一次）
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			// 尝试解锁（每5秒一次）
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
	    
		// 检查是否到达起飞高度
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -ALTITUDE)<0.2)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				break;// 起飞完成，跳出循环
			}
		}

		//发布期望位置信息
		// 持续发布目标位置
		pose.pose.position.x =init_position_x_take_off + 0;
		pose.pose.position.y =init_position_y_take_off + 0;
		pose.pose.position.z =init_position_z_take_off + ALTITUDE;
		local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }   
    
    // 多航点飞行循环
    while(ros::ok())
    {
		 // 航点1（起点）
        if((flag == 1)  && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{ 
			ROS_INFO("Position_1");
            pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;                     
			last_request = ros::Time::now();
            flag=2;
        }
		// 航点2（X+2）
		if((flag ==2) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
			ROS_INFO("Position_2 ");
		    pose.pose.position.x =init_position_x_take_off + 2;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     
			last_request = ros::Time::now();
			flag=3;
		}
		// 航点3（X+2, Y+2）                   
		if((flag ==3) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
		    ROS_INFO("Position_3 ");
		    pose.pose.position.x =init_position_x_take_off + 2;
			pose.pose.position.y =init_position_y_take_off + 2;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;      
		    last_request = ros::Time::now();
			flag=4;
		}
		// 航点4（Y+2）
		if((flag ==4) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
		    ROS_INFO("Position_4 ");
		    pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 2;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     
		    last_request = ros::Time::now();
			flag=5;
		}
		// 航点5（返回起点）
		if((flag ==5) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
            ROS_INFO("Position_1 ");
            pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     		
            last_request = ros::Time::now();
			flag=6;
        }

		 // 降落指令
       	if((flag ==6) && (ros::Time::now() - last_request > ros::Duration(8.0)))
        {
			ROS_INFO("LAND");
 			pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off - 1;    // 降低高度
        }
		// 持续发布目标位置
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



