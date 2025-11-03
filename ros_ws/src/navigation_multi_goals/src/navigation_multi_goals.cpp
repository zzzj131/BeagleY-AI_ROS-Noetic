#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "multi_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal first_goal;
  move_base_msgs::MoveBaseGoal second_goal;
  move_base_msgs::MoveBaseGoal third_goal;
  move_base_msgs::MoveBaseGoal fourth_goal;

  // set up the frame parameters
  first_goal.target_pose.header.frame_id = "map";
  first_goal.target_pose.header.stamp = ros::Time::now();
  second_goal.target_pose.header.frame_id = "map";
  second_goal.target_pose.header.stamp = ros::Time::now();
  third_goal.target_pose.header.frame_id = "map";
  third_goal.target_pose.header.stamp = ros::Time::now();
  fourth_goal.target_pose.header.frame_id = "map";
  fourth_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  first_goal.target_pose.pose.position.x = 0.8;
  first_goal.target_pose.pose.position.y = 0.0;
  first_goal.target_pose.pose.position.z = 0.2;
  first_goal.target_pose.pose.orientation.w = 1;

  second_goal.target_pose.pose.position.x = 0.8;
  second_goal.target_pose.pose.position.y = -0.8;
  second_goal.target_pose.pose.position.z = 0.2;
  second_goal.target_pose.pose.orientation.w = 1;

  third_goal.target_pose.pose.position.x = 1.0;
  third_goal.target_pose.pose.position.y = -1.0;
  third_goal.target_pose.pose.position.z = 0.2;
  third_goal.target_pose.pose.orientation.w = 1;

  fourth_goal.target_pose.pose.position.x = 0.0;
  fourth_goal.target_pose.pose.position.y = 0.0;
  fourth_goal.target_pose.pose.position.z = 0.2;
  fourth_goal.target_pose.pose.orientation.w = 1;


  // Send the goal 1
  ROS_INFO("Sending first_goal");
  ac.sendGoal(first_goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Report, the base moved to the first goal");
  }
  else {
    ROS_INFO("The base failed to move to the first goal");
    return 0;
  }

  // wait for 5 seconds
  ros::param::set("/Robot_pos", "Moving");
  ros::Duration(10.0).sleep();
  // send the goal 2
  ROS_INFO("Sending second_goal");
  ac.sendGoal(second_goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Report, the base moved to the second goal");
 
  }
  else {
    ROS_INFO("The base failed to move to the second goal");
    return 0;
  }

// wait for 5 seconds
  ros::param::set("/Robot_pos", "Moving");
  ros::Duration(10.0).sleep();
  // send the goal 3
  ROS_INFO("Sending third_goal");
  ac.sendGoal(third_goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Report, the base moved to the third goal");
 
  }
  else {


    ROS_INFO("The base failed to move to the third goal");
    return 0;
  }

// wait for 5 seconds
  ros::param::set("/Robot_pos", "Moving");
  ros::Duration(10.0).sleep();
  // send the goal 4
  ROS_INFO("Sending fourth_goal");
  ac.sendGoal(fourth_goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Report, the base moved to the fourth goal");
 
  }
  else {
    ROS_INFO("The base failed to move to the fourth goal");
    return 0;
  }
  
  return 0;
}
