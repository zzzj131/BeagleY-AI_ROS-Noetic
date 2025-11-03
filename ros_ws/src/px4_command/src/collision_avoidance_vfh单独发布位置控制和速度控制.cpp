#include <dwa_avoid.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int    flag_offboard = 0;
int    flag_armed    = 0;
float  delta_x, delta_y,delta_z;

mavros_msgs::PositionTarget setpoint_raw;

mavros_msgs::PositionTarget setpoint_raw_altitude;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{

    ros::init(argc, argv, "collision_avoidance_vfh");
    ros::NodeHandle nh;

	 cout << "Dynamic Window Approach sample program start!!" << endl;
    
    //航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
    double evalParam[4] = { 0.3,0.5,0.2, 3};
    
    //实例化无人车状态变量，如位置x，y，速度等
    State state;
    double goal[2] = {0, 0}; //给定目标点
    
    //设定障碍物半径0.5米
    double obstacleR = 0.6;
    //创建模型参数变量，如最大速度，加速度等
    KModel model;
    
    //创建无人车变量计算的返回结果
    std::vector<State> result;
    
    //输入参数： 当前状态、vt当前速度、ot角速度、evaldt 前向模拟时间、机器人模型参数（没用到）
    UU u1;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    //ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_data_cb);
    
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, scan_cb);

	ros::Publisher  init_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	
    ros::Publisher  local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);   
    
	
	//原生话题控制无人机飞行，此处以函数形式调用	
	ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	
	ros::Publisher  mavros_setpoint_altitude_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
     // 频率 [20Hz]
    ros::Rate rate(20.0);
    
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


	//期望的飞行位置，相对于上电时刻，其中x表示正东方向，y表示正北方向
    delta_x = -10; 
    delta_y = -10;
    delta_z = 2; 

    setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
    setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = 0;
	setpoint_raw.position.y = 0;
	setpoint_raw.position.z = init_position_z+delta_z;
	mavros_setpoint_pos_pub.publish(setpoint_raw);


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
	mavros_setpoint_pos_pub.publish(setpoint_raw);
        //init_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    //等待连接飞控
	while (ros::ok() && !current_state.connected) 
	{
		ros::spinOnce();
		rate.sleep();
	}

	ros::Time last_request = ros::Time::now();

	while(ros::ok())
		{
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
           		 if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
	   			 {
               		 if( arming_client.call(arm_cmd) && arm_cmd.response.success)
						{
                   	 ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		//if( arm_cmd.response.success && offb_set_mode.response.mode_sent)
		//	break;
		    if(ros::Time::now() - last_request > ros::Duration(10.0))
		    	break;
		//	ROS_INFO("1111");
			//pose.pose.position.x = init_position_x + 0;
			//pose.pose.position.y = init_position_y + 0;
			//pose.pose.position.z = init_position_z + 2;
	//		printf("   init_position_z   = %f", init_position_z);
	//		printf("   expect_position_z = %f\n", init_position_z+delta_z);
		    	
		    mavros_setpoint_pos_pub.publish(setpoint_raw);
		  //  init_pos_pub.publish(pose);
		    ros::spinOnce();
		    rate.sleep();
	}
     //给出最终期望目标位置
     goal[0] = init_position_x + delta_x;
     goal[1] = init_position_y + delta_y;
     
	while (ros::ok())
    	{
    			

        DWAreturn dwareturn = DynamicWindowApproach(state, model, goal, evalParam, obs, obstacleR);
        
        //只用到返回的速度和角速度即可
        u1 = dwareturn.u;
        
        state.x = map_local_x;
        state.y = map_local_y;
        state.yaw = yaw;
        state.v = current_linear_x;
        state.w = current_angular_z;
        
 
		geometry_msgs::Twist vel_msg;   
		//是否到达目的地，直接用三角定理判断，到达目的地则跳出，否则继续路径规划流程
        if (sqrt(pow(state.x - goal[0], 2) + pow(state.y - goal[1], 2)) < 1)
        {
            cout << "Arrive Goal" << endl;
			u1.vt = 0;
			u1.ot = 0;
			//goal[0] = 10;
			//goal[1] = 10;
			
	    	// 发布消息
			//cmd_vel_pub.publish(vel_msg);
            
            //break;
        }
	    
	/*    printf("init_position_x = %f",init_position_x);
	    printf("init_position_y = %f",init_position_y);
	    printf("init_position_z = %f",init_position_z);
	    printf("goal[0] = %f",goal[0]);
	    printf("goal[1] = %f\n",goal[1]);*/
	    
	    printf("  u1.vt = %f",u1.vt);
	    printf("  u1.ot = %f",u1.ot);
		printf("  state.yaw = %f",state.yaw);
		printf("  state.x = %f",state.x);
		printf("  state.y = %f\n",state.y); 
		

		

		//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
		//Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
		//Bit 10 should set to 0, means is not force sp
		setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024/* + 2048*/;
		setpoint_raw.coordinate_frame = 8;
		setpoint_raw.velocity.x = u1.vt;
		//setpoint_raw.velocity.y = 0;
		setpoint_raw.position.z = init_position_z+delta_z;
		setpoint_raw.yaw_rate = u1.ot;
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		
		
		
		setpoint_raw_altitude.type_mask = 1 + 2 +/* 4 +*/ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_raw_altitude.coordinate_frame = 1;
		setpoint_raw_altitude.position.z = init_position_z+delta_z;
		mavros_setpoint_altitude_pub.publish(setpoint_raw_altitude);


        //cout << "第"<<iii<<"s到达的位置为" << "x = " << state.x << " y = " << state.y << endl;
        ros::spinOnce();
		rate.sleep();
    }
    return 0;
}










