#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <nav_goals/commandreader.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/*ros::NodeHandle nh_command;
ros::Publisher command_pub = nh_command.advertise<std_msgs::Int32>("commands",1000); */




//void commandCallback(const std_msgs::Int64::ConstPtr& message)
//{

//    bool begin_trial_flag = false;
//    bool end_trial_flag = false;
//    int left_stim_counter = 0;
//    int right_stim_counter = 0;


//   //Wait for '0' (start trial) stims to stop coming through
//    while (!begin_trial_flag && left_stim_counter + right_stim_counter == 0)
//    {
//       if(message->data ==0)
//       {begin_trial_flag = true;}
//       switch(message->data){
//        case 0: break;
//        case 1: left_stim_counter++; break;
//        case 2: right_stim_counter++; break;
//        case 32: end_trial_flag = true; break;
//        default: ROS_INFO("Error: invalid stimulation detected"); break;
//       }

//    //Begin processing stimulations to determine command that will be sent
//    //If '1' (left stim), increment left counter, If '2' (right stim), increment right counter. Wait for 'End of trial' stim
//    }
//    while(!end_trial_flag){
//        switch(message->data){
//        case 1: left_stim_counter++; break;
//        case 2: right_stim_counter++; break;
//        case 32: end_trial_flag = true; break;
//        }
//    }
//  //compare left stim counter to right counter. If left > right, send left command. If right > left, send right command.
//   //Command is weighted - more difference between counters means larger distance is sent
//    int counter_weight;
//    if(left_stim_counter > right_stim_counter)
//    {
//        counter_weight = left_stim_counter/(left_stim_counter+right_stim_counter);
//       command_pub.publish(counter_weight);
//    }
//    else if (right_stim_counter < left_stim_counter)
//    {
//        counter_weight = right_stim_counter/(left_stim_counter+right_stim_counter);
//        command_pub.publish(counter_weight);
//    }
//    else command_pub.publish(0);
//}

void trajectoryMove(int direction, int distance, int orientation, MoveBaseClient& ac){


    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base_simple action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
  switch(direction){
    case 1:
      goal.target_pose.pose.position.x = -1.0 * distance;
      break;
    case 2:
      goal.target_pose.pose.position.x = 1.0 * distance;
      break;
    default: break;
  }
    goal.target_pose.pose.orientation.w = 1.0 * orientation;



    //we'll send a goal to the robot to move 1 meter forward
//    goal.target_pose.header.frame_id = "base_link";
//    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = 1.0;
//    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal ");
    ac.sendGoal(goal);



}


int main(int argc, char** argv){
    //tell the action client that we want to spin a thread by default
  ros::init(argc, argv, "nav_goals");
    MoveBaseClient ac("move_base", true);




  ros::NodeHandle nh;


  while(ros::ok()){
      CommandReader commands;  //We instantiate the class with each loop, making sure the constructor runs every time to flush previous values.
ros::Subscriber command_read = nh.subscribe("chatter",1000,&CommandReader::commandCallback,&commands);

    trajectoryMove(commands.getDirection(),commands.getCounterWeight(),1,ac);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Goal reached"); //unlock a thread that allows command callback to continue
      ROS_INFO("Goal not reachable by robot. Stopping.");

  }



  return 0;
}
