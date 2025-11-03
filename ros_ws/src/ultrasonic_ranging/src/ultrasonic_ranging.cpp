#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>       
#include <unistd.h>      
#include <serial/serial.h>
#include <std_msgs/Float32.h>
serial::Serial Robot_Serial; //声明串口对象
ros::Publisher SonarPub;
ros::Timer timer1;
int EnLog = 0;
void SendTimerOut_Callback(const ros::TimerEvent&){
	unsigned char SendBuff[1] ;
	SendBuff[0] = 0x55;
	Robot_Serial.write(SendBuff, 1);
}
int main(int argc, char** argv)
{	
	std::string usart_port;
	int baud_data = 9600;
	int Freq_t=0,RecLength=0;
	std::string topic_name;
	unsigned char RecBuff[5];
	std_msgs::Float32 sonarMsg;
    ros::init(argc, argv, "us100_to_ros");
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("port", usart_port, "/dev/SonarUs100"); 
	nh_private.param<std::string>("SonarPubName",topic_name, "Sonar1"); 
 	nh_private.param<int>("baud", baud_data, 9600);
 	nh_private.param<int>("freq", Freq_t, 10);//max 40 Hz
 	nh_private.param<int>("log" , EnLog, 0);//max 40 Hz
 	ros::NodeHandle node;
 	timer1 = node.createTimer(ros::Duration(1/Freq_t), SendTimerOut_Callback); 
 	SonarPub = node.advertise<std_msgs::Float32>(topic_name, 10);
 	ROS_INFO("Port:%s",usart_port.c_str());
 	ROS_INFO("SonarPubName:%s",topic_name.c_str());
 	ROS_INFO("baud:%d",baud_data);
 	ROS_INFO("freq:%d",Freq_t);
	/**open seril device**/
	try{
		 Robot_Serial.setPort(usart_port);
		 Robot_Serial.setBaudrate(baud_data);
		 serial::Timeout to = serial::Timeout::simpleTimeout(200);
		 Robot_Serial.setTimeout(to);
		 Robot_Serial.open();
  	}
	catch (serial::IOException& e){
		ROS_ERROR_STREAM("[SonarUs100] Unable to open serial port, please check device or permission");
	}
	if(Robot_Serial.isOpen()){
		ROS_INFO_STREAM("[SonarUs100] Successful opening of the serial port, data transmission started");
	}else{
 		ROS_ERROR_STREAM("[SonarUs100] Unable to open serial port, please check device or permission");
	}
	while(ros::ok()){
		RecLength = Robot_Serial.read(RecBuff,2);
		if(RecLength ==2){
			sonarMsg.data = RecBuff[0]*256 + RecBuff[1];
			if(EnLog){
				ROS_INFO("Sonar:%d mm",(int)sonarMsg.data);
			}
			SonarPub.publish(sonarMsg);
			RecLength = 0;
		}
		ros::spinOnce();
	}
    return 0;
}


