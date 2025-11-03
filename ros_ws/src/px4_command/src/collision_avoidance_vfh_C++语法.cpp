#include"vfh_avoid.h"


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_avoidance_vfh");
    ros::NodeHandle nh;
    
	ros::Rate rate(10.0);

    vfh_avoid vfh_avoid(nh,rate);
    
    //二维vector不可以为空，先插入一个初始值
    position_list.push_back(v);
    
	while (ros::ok())
	{
		//7F ‭0111 1111‬
		printf("111\n");
		if (init_mask == 0x7f)
			printf("222");
			break;

		ros::spinOnce();
		rate.sleep();
	}
	printf("init ok45\n");
	int flag_ros_ok = 1; 
    while (ros::ok() && flag_ros_ok == 1)
    {
		if(flag_cb)
		{
		    printf("wait_position\n");
		}
		else
		{
			vfh_avoid.vfh_func(position_list);
			flag_ros_ok  = 0;
			break ;
		}
		ros::spinOnce();
		rate.sleep();
		
	}
    return 0;
}

