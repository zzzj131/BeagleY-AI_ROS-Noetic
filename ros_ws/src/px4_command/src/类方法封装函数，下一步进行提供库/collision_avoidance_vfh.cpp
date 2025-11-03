#include"vfh_avoid.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_avoidance_vfh");
    ros::NodeHandle nh;
	
	ros::Rate rate(10.0);

    vfh_avoid vfh_avoid(nh,rate); 
 
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
	    
	
	vector<vector<float>> position_list; //导航坐标点，可自行添加
	vector<float>  v1 (3, 3);
	vector<float>  v2 (9, 9);
	//vector<float>  v3 (80,80);
	position_list.push_back(v1);
	position_list.push_back(v2); 
	//while (ros::ok())
	//{
		vfh_avoid.vfh_func(position_list);
	//}
 
    return 0;
}

