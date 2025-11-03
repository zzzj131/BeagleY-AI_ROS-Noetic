#ifndef __DWA_AVOID__
#define __DWA_AVOID__

//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>
#include <vector>

//topic 头文件
#include <iostream>
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

#include "tf/transform_datatypes.h"//转换函数头文件
#include <nav_msgs/Odometry.h>//里程计信息格式
using namespace std;


#define PI 3.1415926




float init_position_x,init_position_y,init_position_z;
int flag_init_position = 0;

//用于解决坐标转问题
//double init_yaw = 0;
//int flag_init_yaw = 0;
//时间间隔
double dt = 0.1;

//状态变量
class State
{
public:
	//构造函数，即相关变量初始化，设置了五个无人车变量，初始化全部为0
    State()
    {
        x = 0;
        y = 0;
        yaw = 0;
        v = 0;
        w = 0;
    }
    double x;
    double y;
    double yaw;
    double v;
    double w;
};

class GTreturn
{
public:
    //traj % 机器人轨迹
    std::vector<State> traj;
    State state;
};

//输入参数： 当前状态、vt当前速度、ot角速度、evaldt 前向模拟时间、机器人模型参数（没用到）
class UU
{
public:
    UU()
    {
        vt = 0;
        ot = 0;
    }
    double vt;
    double ot;
};

//模型参数
class KModel
{
public:
    KModel()
    {
        MD_MAX_V = 0.6;//最大速度
        MD_MAX_W = 0.8;//最大角速度
        MD_ACC = 1.0;//加速度
        MD_VW = 1.0;//角加速度
        MD_V_RESOLUTION = 0.05;//速度分辨率
        MD_W_RESOLUTION = 0.05;//角速度分辨率
    }
    double MD_MAX_V;//最大速度
    double MD_MAX_W;//最大角速度
    double MD_ACC;//加速度
    double MD_VW;//角加速度
    double MD_V_RESOLUTION;//速度分辨率
    double MD_W_RESOLUTION;//角速度分辨率
};


class VR
{
 
public:
    VR()
    {
        min_v = 0;
        max_v = 0;
        min_w = 0;
        max_w = 0;
    }
    double min_v;
    double max_v;
    double min_w;
    double max_w;


};

class OB
{
public:
    OB()
    {
        x = 0;
        y = 0;
    }
    double x;
    double y;
};


//评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
class EvalDB_cell
{
public:
    EvalDB_cell()
    {
        vt = 0;
        ot = 0;
        heading = 0;
        dist = 0;
        vel = 0;

    }
    double vt;
    double ot;
    double heading;
    double dist;
    double vel;
};

class SumDB
{
public:

    std::vector<EvalDB_cell> EvalDB;
    std::vector<State> trajDB;

};

//DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
class DWAreturn
{
public:
    std::vector<State> trajDB;
    UU u;

};

std::vector<OB> obs;
OB obstacle;
 

   
nav_msgs::Odometry local_pos;
//geometry_msgs::PoseStamped local_pos;
mavros_msgs::State current_state;
tf::Quaternion quat;  

sensor_msgs::Imu  imu_data;

float desire_z = 1.5; //期望高度

double roll, pitch, yaw;//定义存储r\p\y的容器

uint32_t init_mask = 0;

//当前的坐标位置信息
float map_local_x,map_local_y,map_local_z;

float current_linear_x, current_angular_z;

//转换后的障碍物固定位置，相当于全局位置
float map_x,map_y;

double toDegree(double radian);
double toRadian(double degree);
State f(State state, UU u);
VR CalcDynamicWindow(State state, KModel model);
double CalcHeadingEval(State state, double goal[2]);
double CalcDistEval(State state, std::vector<OB> &obs, double R);
double CalcBreakingDist(double vel, KModel model);
GTreturn GenerateTrajectory(State state, UU u, double evaldt, KModel model);
void NormalizeEval(std::vector<EvalDB_cell> &EvalDB);
SumDB Evaluation(State state, VR vr, double goal[2], std::vector<OB> &obs, double R, KModel model, double evalParam[4]);
DWAreturn DynamicWindowApproach(State state, KModel model, double goal[2], double evalParam[4], std::vector<OB> &obs, double R);
double max(double a, double b);
double min(double a, double b);
void ComputeMV(vector<float> r); 
void state_cb(const mavros_msgs::State::ConstPtr& msg);
//void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); 
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg); 
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg);

void ComputeMV(vector<float> r) 
{
	float scan_distance;
	int range_size = r.size(); //有多少条激光  
	
//	printf("range_size = %d",range_size);
    
    int count = 0;
    obs.clear();
	for (int i = 0; i < range_size; i++)
	{
		//if (!std::isnan(r[i]) && !std::isinf(r[i]))//取出有效值
		if (!std::isnan(r[i]) && !std::isinf(r[i]) && fabs(r[i])<12 && fabs(r[i])>0.3)//取出有效值
		{
			scan_distance = r[i];
			
	
			//imu
			tf::quaternionMsgToTF(imu_data.orientation, quat);
			
			//GPS
			//tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
	
	
	
	    //M10
		map_x = scan_distance* cos((i/1009.00)*2*PI+yaw)+ map_local_x;
		map_y = scan_distance* sin((i/1009.00)*2*PI+yaw)+ map_local_y; 
	    float base_laser_position_x	= scan_distance* float(cos((float(i)/1009.00)*2*PI));
 	    float base_laser_position_y	= scan_distance* float(sin((float(i)/1009.00)*2*PI));	
		
		
		//M10_P
		/*map_x = scan_distance* cos((i/1681.00)*2*PI+yaw)+ map_local_x;
		map_y = scan_distance* sin((i/1681.00)*2*PI+yaw)+ map_local_y; 
	    float base_laser_position_x	= scan_distance* float(cos((float(i)/1681.00)*2*PI));
 	    float base_laser_position_y	= scan_distance* float(sin((float(i)/1681.00)*2*PI));*/	
 	    //    printf("i = %f", i/4.67);
 	    
        
        
        	//450对应ld06激光雷达
		/*map_x = scan_distance* cos((i/450.00)*2*PI+yaw)+ map_local_x;
		map_y = scan_distance* sin((i/450.00)*2*PI+yaw)+ map_local_y; 
	    float base_laser_position_x	= scan_distance* float(cos((float(i)/450.00)*2*PI));
 	    float base_laser_position_y	= scan_distance* float(sin((float(i)/450.00)*2*PI));
 	    printf("i = %f", i/1.25);*/
	 		   
            obstacle.x = map_x;
 	    obstacle.y = map_y;
        
		
		
			    
		//printf(" laser_yaw = %f", float((i/450.00)*2*PI));
	 	//printf("init_yaw = %f",init_yaw);
	//	printf("  dis = %f\n", scan_distance);
		//printf("  yaw = %f", yaw);

  //              printf("  base_x = %f",base_laser_position_x);
//	 	printf("  base_y = %f\n",base_laser_position_y);
		//printf("  map_local_x = %f",map_local_x);
	 	//printf("  map_local_y = %f",map_local_y);	   
		//printf("  map_x = %f", map_x);
		//printf("  map_y = %f\n\n\n", map_y);	
	   	//printf("  obstacle.x = %f", obstacle.x);
	   	//printf("  obstacle.y = %f\n\n\n", obstacle.y);	
	    
        obs.push_back(obstacle);         
	    count++;
		}
	}
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	local_pos = *msg;
    map_local_x       = local_pos.pose.pose.position.x;
    map_local_y       = local_pos.pose.pose.position.y;
    map_local_z       = local_pos.pose.pose.position.z;

   // printf("z = %f\n ", local_pos.pose.pose.position.z);
    //任意高度x或者x，y位置不为0，则表示已经接收到定点位置信息，记录第一次位置作为初始位置，防止上电时刻漂移过大
    if (flag_init_position==0 && (local_pos.pose.pose.position.z!=0))
    {
	   init_position_x = local_pos.pose.pose.position.x;
	   init_position_y = local_pos.pose.pose.position.y;
	   init_position_z = local_pos.pose.pose.position.z;
       flag_init_position = 1;		    
     }

    current_linear_x  = local_pos.twist.twist.linear.x;
    current_angular_z = local_pos.twist.twist.angular.z;
}

void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_data = *msg;
   
}



void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	ComputeMV(msg->ranges);
}


//////////////////////////////////////////////////////
//DWA核心算法
DWAreturn DynamicWindowApproach(State state, KModel model, double goal[2], double evalParam[4], std::vector<OB> &obs, double R)
{
    DWAreturn dwareturn;
    
    //计算当前采样的速度范围（动态窗口）
    VR vr = CalcDynamicWindow(state, model);
    
    /* %% 评价函数 内部负责产生可用轨迹
	% 输入参数 ：当前状态、参数允许范围（窗口）、目标点、障碍物位置、障碍物半径、评价函数的参数
	% 返回参数：
	%           evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
	%           trajDB      每5行一条轨迹 每条轨迹包含 前向预测时间/dt + 1 = 31 个轨迹点（见生成轨迹函数）*/
    SumDB DB = Evaluation(state, vr, goal, obs, R, model, evalParam);
    
    
    //如果所有的方案模拟后都找不到路径，则表示无法通过，让无人车保持速度为0确保安全
    if (DB.EvalDB.empty())
    {
        cout << "no path to goal!" << endl;
        dwareturn.u.vt = 0;
        //dwareturn.u.vt = 0;
        dwareturn.u.ot= 0.1;
        return dwareturn;

    }
    
    //%% 归一化处理，对所有模拟出的可以通过的路径进行归一化，最终选择最优的路径通过
    //% 每一条轨迹的单项得分除以本项所有分数和
    NormalizeEval(DB.EvalDB);


	//% 最终评价函数的计算，
    double result1;
    std::vector<double> feval;
    for (int id = 0; id < DB.EvalDB.size(); id++)
    {
    	//%根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
        result1 = evalParam[0] * DB.EvalDB[id].heading + evalParam[1] * DB.EvalDB[id].dist + evalParam[2] * DB.EvalDB[id].vel;
        feval.push_back(result1);
    }
    //对所有归一化后的评分进行比较，选择最高的得分，并且记录序号为k
    int k = 0;
    for (int ii = 1;ii < feval.size();ii++)
    {
        if (feval[ii]>feval[ii-1])
        {
            k = ii;
        }
    }

    //把最高的得分变量作为返回值给出，包含线速度和角速度，还有
    dwareturn.u.vt = DB.EvalDB[k].vt;
    dwareturn.u.ot = DB.EvalDB[k].ot;
    dwareturn.trajDB = DB.trajDB;
    return dwareturn;

}


// 评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
//               trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
//[evalDB,trajDB]= Evaluation(x,Vr,goal,ob,R,model,evalParam);  %evalParam 评价函数参数 [heading,dist,velocity,predictDT]
//% Vr 参数 = Dynamic Window [vmin,vmax,wmin,wmax] 最小速度 最大速度 最小角速度 最大角速度速度
//对速度和角速度进行全部的模拟，根据评价函数给出速度和角速度
SumDB Evaluation(State state, VR vr, double goal[2], std::vector<OB> &obs, double R, KModel model, double evalParam[4])
{
    SumDB DB;
    GTreturn GT;
    double heading, dist, vel, stopDist;
    EvalDB_cell evaldb;
    UU u;
    for (double vt = vr.min_v;vt < vr.max_v;vt = vt + model.MD_V_RESOLUTION)
    {
        for (double ot = vr.min_w;ot < vr.max_w;ot = ot + model.MD_W_RESOLUTION)
        {
            u.vt = vt;
            u.ot = ot;

            GT = GenerateTrajectory(state, u, evalParam[3], model);
         
            heading = CalcHeadingEval(GT.state, goal);
            dist = CalcDistEval(GT.state, obs, R);
            vel = abs(vt);
            stopDist = CalcBreakingDist(vel, model);

            if (dist > stopDist)
            {
                evaldb.vt = vt;
                evaldb.ot = ot;
                evaldb.heading = heading;
                evaldb.dist = dist;
                evaldb.vel = vel;
                DB.EvalDB.push_back(evaldb);
                
             /*   DB.trajDB.push_back(traj);*/

            }
        }
    }

    return DB;

}

//%% 归一化处理 
//% 每一条轨迹的单项得分除以本项所有分数和
void NormalizeEval(std::vector<EvalDB_cell> &EvalDB)
{
    int n = EvalDB.size();
    double sum1 = 0, sum2 = 0, sum3 = 0;
    int i;
 
    for (i = 0;i < n;i++) 
    {
    	sum1 = sum1 + EvalDB[i].heading; 
    }
    for (i = 0;i < n;i++) 
    { 
   		EvalDB[i].heading = EvalDB[i].heading / sum1; 
    }

    for (i = 0;i < n;i++) { sum2 = sum2 + EvalDB[i].dist; }
    for (i = 0;i < n;i++) { EvalDB[i].dist = EvalDB[i].dist / sum2; }

    for (i = 0;i < n;i++) { sum3 = sum3 + EvalDB[i].vel; }
    for (i = 0;i < n;i++) { EvalDB[i].vel = EvalDB[i].vel / sum3; }

}


/*%% 单条轨迹生成、轨迹推演函数
% 输入参数： 当前状态、vt当前速度、ot角速度、evaldt 前向模拟时间、机器人模型参数（没用到）
% 返回参数; 
%           x   : 机器人模拟时间内向前运动 预测的终点位姿(状态); 
%           traj: 当前时刻 到 预测时刻之间 过程中的位姿记录（状态记录） 当前模拟的轨迹  
%                  轨迹点的个数为 evaldt / dt + 1 = 3.0 / 0.1 + 1 = 31
%*/  
GTreturn GenerateTrajectory(State state, UU u, double evaldt, KModel model)
{
    GTreturn GT;
    double time = 0;
    GT.traj.push_back(state);
    while (time <= evaldt)
    {
        time = time + dt;
        state = f(state, u);
        GT.traj.push_back(state);
    }
    GT.state = state;
    return GT;

}

//%% 计算制动距离 
//%根据运动学模型计算制动距离, 也可以考虑成走一段段圆弧的累积 简化可以当一段段小直线的累积
double CalcBreakingDist(double vel, KModel model)
{
    double stopDist = 0;
    //给定加速度的条件下 速度减到0所走的距离
    while (vel > 0) 
    {
        stopDist = stopDist + vel * dt; //制动距离积分 
        vel = vel - model.MD_ACC * dt;

    }
    return stopDist;
}



/*%% 障碍物距离评价函数  （机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
% 输入参数：位姿、所有障碍物位置、障碍物半径
% 输出参数：当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离 如果大于设定的最大值则等于最大值
% 距离障碍物距离越近分数越低*/
double CalcDistEval(State state, std::vector<OB>& obs, double R)
{
    //Define an upper distance limit
    double dist = 100;
    for (int io = 0;io < obs.size();io++)
    {
    	//到第io个障碍物的距离
        double disttmp = sqrt(pow(obs[io].x - state.x, 2) + pow(obs[io].y - state.y, 2)) - R;
        if (dist > disttmp)
        {
            dist = disttmp;
        }

    }
    // 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
    if (dist >= 2 * R)
    {
        dist = 2 * R;
    }
    return dist;
}

//%% heading的评价函数计算
//% 输入参数：当前位置、目标位置
//% 输出参数：航向参数得分  当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 最大180分
double CalcHeadingEval(State state, double goal[2])
{
    //% 机器人朝向
    double theta = toDegree(state.yaw);
    
    //目标点相对于机器人本身的方位
    double goalTheta = toDegree(atan2(goal[1] - state.y, goal[0] - state.x));
    double targetTheta;
    if (goalTheta > theta)
    {
        targetTheta = goalTheta - theta;
    }
    else
    {
        targetTheta = theta - goalTheta;
    }
    double heading = 90 - targetTheta;
    return heading;



}

//%% 计算动态窗口
//% 返回 最小速度 最大速度 最小角速度 最大角速度速度
VR CalcDynamicWindow(State state, KModel model)
{
    VR vr;
    vr.min_v = max(0, state.v - model.MD_ACC * dt);
    vr.max_v = min(model.MD_MAX_V, state.v + model.MD_ACC * dt);
    vr.min_w = max(-model.MD_MAX_W, state.w - model.MD_VW * dt);
    vr.max_w = min(model.MD_MAX_W, state.w + model.MD_VW * dt);

    return vr;

}

State f(State state, UU u)
{
    State state2;

    state2.x = state.x + u.vt * dt * cos(state.yaw);
    state2.y = state.y + u.vt * dt * sin(state.yaw);
    state2.yaw = state.yaw + dt * u.ot;
    state2.v = u.vt;
    state2.w = u.ot;

    return state2;
}



//角度转为弧度
double toRadian(double degree)
{
    double radian = degree / 180 * PI;
    return radian;

}
//弧度转为角度
double toDegree(double radian)
{

    double degree = radian / PI * 180;
    return degree;

}

//取大值
double max(double a, double b)
{
    if (a < b) { a = b; };
    return a;

}

//取小值
double min(double a, double b)
{
    if (a > b) { a=b; };
    return a;

}

#endif
