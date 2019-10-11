#include <nav_goals/navigator.h>



Navigator::Navigator(ros::NodeHandle nh) : nh_navigator_(nh), ac("move_base",true)
{
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base_simple action server to come up");
    }


}

void Navigator::trajectoryMove(int direction, double distance, int orientation, MoveBaseClient &ac)
{
    //wait for the action server to come up
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    switch(direction){
    case 1:
        goal.target_pose.pose.position.y = -DIST_MODIFIER * distance;
        break; //Left
    case 2:
        goal.target_pose.pose.position.y = DIST_MODIFIER * distance;
        break; //Right
    case 3:
        goal.target_pose.pose.position.x = -DIST_MODIFIER * distance;
        break; //Down
    case 4:
        goal.target_pose.pose.position.x = DIST_MODIFIER * distance;
        break; //Up

    default: break;
    }
    goal.target_pose.pose.orientation.w = orientation;



    //we'll send a goal to the robot to move 1 meter forward
    //    goal.target_pose.header.frame_id = "base_link";
    //    goal.target_pose.header.stamp = ros::Time::now();

    //    goal.target_pose.pose.position.x = 1.0;
    //    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal ");
    ac.sendGoal(goal);
    //this is where we check for the DWA callback bool, after waiting a couple of seconds.
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {   ROS_INFO("Goal reached");
    }
    else{ROS_INFO("Goal not reachable by robot. Stopping.");}
}

void Navigator::cancelAll(Navigator::MoveBaseClient &ac)
{
    ac.cancelAllGoals();
}


