#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include <ros/ros.h>

#include <sstream>
#include <iostream>
#include <string>
#include <chrono>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class Navigator
{

public:
    Navigator(ros::NodeHandle nh);
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    void trajectoryMove(int direction, double distance, double orientation, MoveBaseClient& ac);
    void plannerCallback(const nav_msgs::Path &msg);
    void cancelAll(MoveBaseClient& ac);
    bool planFoundStatus();
    const double DIST_MODIFIER = 0.5;
protected:

private:
    move_base_msgs::MoveBaseGoal goal;
    MoveBaseClient ac;

    ros::NodeHandle nh_navigator_;
    ros::Subscriber sub_navigator_;
    ros::Subscriber sub_planner_;


    bool plan_found;
    bool on_the_move;



};

#endif // NAVIGATOR_H
