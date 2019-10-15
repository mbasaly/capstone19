#include <nav_goals/navigator.h>



Navigator::Navigator(ros::NodeHandle nh) : nh_navigator_(nh), ac("move_base",true)
{

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base_simple action server to come up");
    }
    sub_planner_ = nh.subscribe("/move_base/NavfnROS/plan",2,&Navigator::plannerCallback,this);

}

void Navigator::trajectoryMove(int direction, double distance, double orientation, MoveBaseClient &ac)
{
    //wait for the action server to come up
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    switch(direction){
    case 1:
        goal.target_pose.pose.position.y = DIST_MODIFIER * distance;
        break; //Left
    case 2:
        goal.target_pose.pose.position.y = -1 * DIST_MODIFIER * distance;
        break; //Right
    case 3:
        goal.target_pose.pose.position.x = -1 * DIST_MODIFIER * distance;
        break; //Down
    case 4:
        goal.target_pose.pose.position.x = DIST_MODIFIER * distance;
        break; //Up

    default: break;
    }
    goal.target_pose.pose.orientation.w = orientation;



    ROS_INFO("Sending goal ");
    ac.sendGoal(goal);
    ac.waitForResult();
    on_the_move = 1;

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {   ROS_INFO("Goal reached");
    }
    else{ROS_INFO("Goal not reachable by robot. Stopping.");}
    on_the_move = 0;
}

void Navigator::plannerCallback(const nav_msgs::Path &msg)
{
 long current_stamp = 0;
 long next_stamp = 0;
 current_stamp = msg.header.stamp.toNSec();
 ros::Duration(0.25).sleep();
 next_stamp = msg.header.stamp.toNSec();
 if(current_stamp == next_stamp && on_the_move){
     plan_found = false;
     ac.cancelAllGoals();
     ROS_INFO_STREAM("Goal was cancelled because path could not be calculated. Wait for new trial.");
 }
 else if (current_stamp != next_stamp && on_the_move){
     plan_found = true;
 }

}

void Navigator::cancelAll(Navigator::MoveBaseClient &ac)
{
    ac.cancelAllGoals();
}

bool Navigator::planFoundStatus()
{
    return plan_found;
}


