#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int create_goal(double x, double y, double w, std::string success_message, MoveBaseClient &ac) 
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  { 
    // ROS_INFO("goal - %s", success_message.c_str());
    return 1;
  }

  else {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
  return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("pick_objects", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the pick_objects action server to come up");
  }

  create_goal(5.0,5.0,1.0, "at pick_up location!", ac);
  ros::Duration(5).sleep();
  create_goal(10.0, 10.0, 1.0, "at drop off location!", ac);

  return 0;
}
