#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int create_goal(double x, double y, MoveBaseClient &ac) 
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal %f %f %f", x, y);
  ac.sendGoal(goal);

  return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the pick_objects action server to come up");
  }

  create_goal(-1.0,-1.0, ac);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal - at pick_up location!");
  else
    ROS_INFO("The bot failed to move to the goal");
 
  ros::Duration(5).sleep();

  create_goal(4.0, -1.0, ac);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal - at drop off location!");
  else
    ROS_INFO("The bot failed to move to the goal");

  
  

  return 0;
}
