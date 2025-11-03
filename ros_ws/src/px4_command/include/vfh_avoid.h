#ifndef __VFH_AVOID__
#define __VFH_AVOID__

//ROS 头文件
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>
#include <vector>

//topic 头文件
#include <iostream>
#include<stdio.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros_msgs/WaypointList.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/HomePosition.h>
#include <GeographicLib/Geocentric.hpp>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <px4_command/position.h>
#include <px4_command/Vector2.h>

#define PI 3.141592

#define arrived_radius 0.5

using namespace std;

int flag_offboard = 0;
int flag_armed    = 0;
int flag_cb       = 1;

//Point2D ;
struct Point2D
{
	float x;
	float y;
	float z;
}Uavp;
float scan_distance_max= 2.1;
float scan_distance_min= 0.1;
float angle_resolution= 1.0;
float heading= 90;
float sector_value= 30;
float sector_scale= 10;


vector<float> map_cv;
vector<double> ranges;
float desire_z = 1.5; //期望高度

uint32_t init_mask = 0;

Eigen::Vector3d vel_sp_body;        

Eigen::Vector3d current_local_pos;
bool local_pos_updated = false;

bool scan_updated = false;

bool heading_updated = false;

Eigen::Vector3d current_gps;

vector<vector<float>> position_list; //导航坐标点，可自行添加 
vector<float>  v(1, 1);

	

 

class vfh_avoid
{
	public:
	
	vfh_avoid(ros::NodeHandle &nh, ros::Rate &rate);
	
	~vfh_avoid();
	
	ros::NodeHandle    nh;
	
	ros::Rate rate; 

	ros::Subscriber gps_sub;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber homePos_sub;
    ros::Subscriber head_sub;
    ros::Subscriber position_sub;

    ros::Publisher local_position_pub;
    ros::Publisher local_pos_pub;
    
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

	mavros_msgs::State current_state;  

	geometry_msgs::PoseStamped local_position;
	
	geometry_msgs::PoseStamped local_pos;

	mavros_msgs::HomePosition home_pos;
	
	geometry_msgs::PoseStamped pose;
  
	
	void SetUavPosition(Point2D& uav) ;
	void SetUavHeading(float hd);
	void ComputeMV(vector<float> r);
	bool IsFrontSafety();
	float CalculDirection(Point2D& goal);		
	void state_cb(const mavros_msgs::State::ConstPtr& msg); 
	void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg); 
	void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void heading_cb(const std_msgs::Float64::ConstPtr &msg);
	void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void position_cb(const px4_command::position::ConstPtr& msg);
	void vfh_func(vector<vector<float>> &pos_list);
};


vfh_avoid::vfh_avoid(ros::NodeHandle &nh, ros::Rate &rate):nh(nh),rate(rate)
{
    //ros::Rate rate(20.0);
	scan_distance_max = 2.1;
	scan_distance_min = 0.1;
	angle_resolution = 1.0;
    heading = 90;
	sector_value = 30;
	sector_scale = 10;
	Uavp.x = 0;
	Uavp.y = 0;
	

    gps_sub = nh.subscribe("/mavros/global_position/global",10, &vfh_avoid::gps_cb,this);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &vfh_avoid::state_cb,this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &vfh_avoid::local_pos_cb,this);
    lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/lidar2Dscan", 20, &vfh_avoid::scan_cb,this);
    homePos_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, &vfh_avoid::home_pos_cb,this);
    head_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, &vfh_avoid::heading_cb,this);
    position_sub = nh.subscribe<px4_command::position>("/position_pub", 10, &vfh_avoid::position_cb,this);

    local_position_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 20);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

vfh_avoid::~vfh_avoid() 
{
}

                               

void vfh_avoid::SetUavPosition(Point2D& uav) 
{
	Uavp.x = uav.x;
	Uavp.y = uav.y;
	Uavp.z = uav.z;
}

void vfh_avoid::SetUavHeading(float hd) {
	heading = hd;
	heading += 270;
	heading = (int)heading % 360;
	heading = 360 - heading;
}

void vfh_avoid::ComputeMV(vector<float> r) {
	float dist[360] = { 0 };
	ranges.clear();
	map_cv.clear();
	int range_size = r.size(); //有多少条激光

	for (size_t i = 0; i < range_size; i++)
	{
		//A non-zero value (true) if x is a NaN value; and zero (false) otherwise.

		//isinf A non-zero value (true) if x is an infinity; and zero (false) otherwise.
		if (!std::isnan(r[i]) && !std::isinf(r[i]))//取出有效值
		{
			float scan_distance = r[i];
			int sector_index = std::floor((i*angle_resolution) / sector_value);//(i*1)/30 sector_index:[1 12]
			if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
				scan_distance = 0;
			else
				scan_distance = scan_distance_max - scan_distance;

			dist[sector_index] += scan_distance;
		}
		ranges.push_back(r[i]);
	}

	for (int j = 0; j < (int)(360 / sector_value); j++)//把uav四周分为12个sector，每个sector的值越小，代表越安全。
	{
		map_cv.push_back(dist[j]);
//		printf("dist[%d]=%f-",j,dist[j]);
	}
//	printf("##############################################################################################\n");
}

/*检测前方[340 360],[0 20]范围内是否安全*/
bool vfh_avoid::IsFrontSafety()
{
	float goal_sector = (int)(0 - (sector_value - sector_scale) + 360) % 360;//(0-(30-10)+360)%360 = 340,goal_sector = 340
	int start_index = goal_sector / angle_resolution;//start_index = 340
	float scan_distance = 0;
	for (int i = 0; i < (sector_value - sector_scale) * 2 / angle_resolution; i++)//for(i=0;i<40;i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.3)
	{
		return true;
	}

	return false;
}

/*输入期望坐标点，输出期望机头方向*/
float vfh_avoid::CalculDirection(Point2D& goal) {
	float ori;
	//Compute arc tangent with two parameters
	//return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
	//One radian is equivalent to 180/PI degrees.
	float G_theta = atan2((goal.y - Uavp.y), (goal.x - Uavp.x));
	float goal_ori = G_theta * 180 / PI; //目标点相对uav的方向信息，即与y轴的夹角，正值代表在第一三现象，负值代表在二四现象。ENU坐标系下
	if (goal_ori < 0)
	{
		goal_ori += 360;
	}
	goal_ori -= heading;
	goal_ori += 360;
	goal_ori = (int)goal_ori % 360; //把目标点方向转化成机体坐标系下的方向

	float goal_sector = (int)(goal_ori - sector_value + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < sector_value * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	
	
	//机头转向目标，并且转化为弧度
	if (scan_distance < 0.1)//判断目标方向处是否安全
	{
		ori = goal_ori;
		ori += heading;
		ori = (int)ori % 360;

		return ori;//若安全，则把机头指向目标处
	}


	vector<int> mesh;
	for (int i = 0; i < map_cv.size(); i++) //确定栅格中CV值 共有12个栅格，CV值越大，存在障碍物可能性越大
	{
		if (map_cv[i] < 0.1)
			mesh.push_back(0);
		else if (map_cv[i] >= 0.1 && map_cv[i] < 0.3)
			mesh.push_back(2);
		else
			mesh.push_back(4);
	}

	vector<float> cand_dir;
	for (int j = 0; j < mesh.size(); j++)
	{
		if (j == mesh.size() - 1) //if(j == 11)
		{
			if (mesh[0] + mesh[mesh.size() - 1] == 0)
				cand_dir.push_back(0.0);
		}
		else
		{
			if (mesh[j] + mesh[j + 1] == 0)
				cand_dir.push_back((j + 1)*sector_value);//寻找安全角度，机体坐标系下；即确定波谷
		}
	}

	if (cand_dir.size() != 0) {
		vector<float> delta;
		for (auto &dir_ite : cand_dir) {
			float delte_theta1 = fabs(dir_ite - goal_ori);
			float delte_theta2 = 360 - delte_theta1;
			float delte_theta = delte_theta1 < delte_theta2 ? delte_theta1 : delte_theta2;
			delta.push_back(delte_theta);
		}//寻找接近目标区域的波谷
		int min_index = min_element(delta.begin(), delta.end()) - delta.begin();
		ori = cand_dir.at(min_index);

		ori += heading;
		ori = (int)ori % 360;

		return ori;//确定机头方向
	}

	return -1;
}


void vfh_avoid::state_cb(const mavros_msgs::State::ConstPtr& msg) {
	init_mask |= 1 << 1;
	current_state = *msg;
}


void vfh_avoid::home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
	init_mask |= 1 << 2;
	home_pos = *msg;
}


void vfh_avoid::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	init_mask |= 1 << 3;
	current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
	local_pos = *msg;
	Point2D pt2d;
	pt2d.x = current_local_pos.x();
	pt2d.y = current_local_pos.y();
	SetUavPosition(pt2d);
	local_pos_updated = true;
}


void vfh_avoid::scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//激光雷达数据回调函数
	init_mask |= 1 << 5;
	ComputeMV(msg->ranges);
	scan_updated = true;
}


void vfh_avoid::heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
	init_mask |= 1 << 6;
	SetUavHeading(msg->data);
	heading_updated = true;
}

void vfh_avoid::gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_mask |= 1;
	current_gps = { msg->latitude, msg->longitude, msg->altitude };
}
 
 

 // 接收到订阅的消息后，会进入消息回调函数
void vfh_avoid::position_cb(const px4_command::position::ConstPtr& msg)
{
    printf("flag_cb = %d\n",flag_cb);
    flag_cb = 0;
	px4_command::position position_msg = *msg;
	for(int i=0; i<sizeof(position_msg)/8; i++)
	{
		vector<float>  v{position_msg.Vector2[i].x, position_msg.Vector2[i].y};	
		position_list.push_back(v); 
		ROS_INFO("x=%f",position_msg.Vector2[i].x);
		ROS_INFO("y=%f",position_msg.Vector2[i].y); 
		
		ROS_INFO("xxx=%f",position_list[i+1][0]);
		ROS_INFO("yyy=%f",position_list[i+1][1]);
		ROS_INFO("sizeof= %f",sizeof(position_msg)/8);  
	}	
}




void vfh_avoid::vfh_func(vector<vector<float>> &pos_list)
{
	//**********************************************pose vextor容器***********************************************/
		
		std::vector<geometry_msgs::PoseStamped> pose;

		geometry_msgs::PoseStamped p;
		ROS_INFO("sizeof= %f",sizeof(pos_list)/8);  
		for (int i = 0; i < sizeof(pos_list)/8+1; i++)
		{
			p.pose.position.x = pos_list[i][0];
			p.pose.position.y = pos_list[i][1];
			pose.push_back(p);
			printf("pos%d = : %f    %f\n", i+1, p.pose.position.x,  p.pose.position.y);

		}
	for (int i = 0; i < pose.size(); i++)
		{
		
		    printf("333333\n");
			while (ros::ok()) 
			{
				local_pos_updated = false;
				scan_updated = false;
				heading_updated = false;

				while (ros::ok())
				{
					ros::spinOnce();//等待订阅更新完成
					if (local_pos_updated && scan_updated && heading_updated)
						break;
					rate.sleep();
				}

				if (!current_state.guided)
					break;

				if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < arrived_radius &&
					fabs(local_pos.pose.position.y - pose[i].pose.position.y) < arrived_radius)
				{
					break;//当前位置与期望航点距离相差不到1m,则退出此次航点任务
				}

				Point2D goal;
				goal.x = pose[i].pose.position.x;//读取期望位置
				goal.y = pose[i].pose.position.y;
				float direction = CalculDirection(goal);//根据障碍物的分布与期望位置，得到期望的航向

				if (direction >= -0.5)
				{
					if (direction > 180)
					{
						direction -= 360;
					}
					float arc = 3.1415 / 180 * direction;

					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0.5* cos(arc);
					pos_target.velocity.y = 0.5* sin(arc);
					pos_target.position.z = desire_z;
					local_position_pub.publish(pos_target);

					ros::Time last_request = ros::Time::now();
					while (ros::ok()) {
						local_pos_updated = false;
						scan_updated = false;
						heading_updated = false;

						while (ros::ok())
						{
							ros::spinOnce();
							if (local_pos_updated && scan_updated && heading_updated)
								break;
							rate.sleep();
						}

						if (!current_state.guided)
							break;

						if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < arrived_radius &&
							fabs(local_pos.pose.position.y - pose[i].pose.position.y) < arrived_radius)
						{
							break;
						}

						if (ros::Time::now() - last_request > ros::Duration(4.0))
							break;

						if (IsFrontSafety() == false)
							break;

						local_position_pub.publish(pos_target);
					}
				}
				else
				{
					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0;
					pos_target.velocity.y = 0;
					pos_target.position.z = desire_z;
 					local_position_pub.publish(pos_target);
				}
			}
		}
}

#endif


