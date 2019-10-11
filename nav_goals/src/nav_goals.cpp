#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <nav_goals/commandreader.h>
#include <nav_goals/navigator.h>



#include <sstream>
#include <iostream>
#include <string>

#include <thread>

#include <mutex>
#include <random>
#include <memory>

using namespace std;


//! @todo:
//! * Fix callback/service call so that the entire process completes before serving back a direction and a distance. Direction and distance return as a struct might be an easy way to encapsulate. This should live inside the navigator class, and pull commands from the commandreader process.
//!     Process ->> Unlock, CommandReader subs to the chatter topic, populates distance and direction values, lock, push to navigator
//!
//!
//!        Publisher ==========Topic============Subscriber
//!        OVibe_to_tcp        "chatter"        CommandReader
//!
//!
//!        Server ==================== Client
//!        CommandReader               Navigator
//!        Direction/Dist
//!
//!
//!
//!
//! * Create a callback that listens for the DWAPlanner's global_map. If no callback, then stop the robot/cancel goals.
//!
//! Process ->> navigator constructor has bool callback_detected = false. Becomes true if callback function DWAcallback receives a callback. If false, navigator cancels goals (check is made a number of seconds after goal is sent).
//!
//!         Publisher ==========Topic============Subscriber
//!         Navstack        DWA/global        Navigator
//!
//!
//!
//!* Create a callback within the TCP Node - this listens for the navigator's driving status and won't read more packets until stop
//!
//!
//!


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int main(int argc, char** argv){
    //tell the action client that we want to spin a thread by default
    ros::init(argc, argv, "nav_goals");
    MoveBaseClient ac_("move_base", true);
    ros::NodeHandle nh;
    Navigator turtle(nh);
    CommandReader commands(nh);

    ROS_INFO_STREAM("outide ros ok "<<commands.getEndFlag());



    while(ros::ok()){
        ROS_INFO_STREAM("inside ros ok "<<commands.getEndFlag());



        while(commands.getEndFlag() == false) {
            ros::spinOnce();
            if(commands.getStimState() == true){
                commands.processCommand();
            }

        }
        ROS_INFO_STREAM("Current goal: Direction - "<<commands.getDirection()<< " Counterweight - "<<commands.getCounterWeight());

        turtle.trajectoryMove(commands.getDirection(),commands.getCounterWeight(),commands.getOrientation(),ac_);
        commands.nextTrial();
    }


    //   std::shared_ptr<CommandReader> commands(new CommandReader(nh));



    return 0;
}
