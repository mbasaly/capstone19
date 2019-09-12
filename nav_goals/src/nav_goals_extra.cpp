#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<client_node::

/**
direction specified by STIM: 1 = left, 2 = right.
distance specified by ??? (default 1 for now). Look for parameter in navigation that rates accuracy/reliability of target??
orientation should default to 1 for now. Leaving variable in for later use.
**/
void trajectoryMove(int direction, int distance, int orientation){
  move_base_msgs::MoveBaseGoal goal;
  //
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
switch(direction){
  case 1:
    goal.target_pose.pose.position.x = -1.0 * distance;
    break;
  case 2:
    goal.target_pose.pose.position.x = 1.0* distance;
    break;
  default: break;
}
  goal.target_post.pose.position.w = 1.0 * orientation;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Action successful");
  else
    ROS_INFO("Action failed.");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "nav_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base_simple action server to come up");
  }
  //while client_node cpp is active, listen for stims
  while()

  return 0;
}
