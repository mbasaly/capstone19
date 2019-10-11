#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include <ros/ros.h>

#include <sstream>
#include <iostream>
#include <string>
#include <chrono>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class Navigator
{

public:
    Navigator(ros::NodeHandle nh);
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    void trajectoryMove(int direction, double distance, int orientation, MoveBaseClient& ac);
    void cancelAll(MoveBaseClient& ac);

protected:

private:
    move_base_msgs::MoveBaseGoal goal;
    MoveBaseClient ac;
    const double DIST_MODIFIER = 0.5;


    ros::NodeHandle nh_navigator_;
    ros::Subscriber sub_navigator_;
    ros::Subscriber sub_dwa_;



};

#endif // NAVIGATOR_H
