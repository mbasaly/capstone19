#ifndef COMMANDREADER_H
#define COMMANDREADER_H
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <nav_msgs/Odometry.h>
#include <mutex>



class CommandReader
{
public:
    CommandReader(ros::NodeHandle nh);
    void commandCallback(const std_msgs::Int64ConstPtr &message);
    int getCounterWeight();
    int getDirection();
    int getOrientation();
    bool getEndFlag();
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
protected:

private:
    bool begin_trial_flag;
    bool end_trial_flag;
    int left_stim_counter;
    int right_stim_counter;
    int counter_weight;
    int direction;
    int orientation;

    std::mutex counter_mtx_;
    std::mutex direction_mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Subscriber odom_sub_;

};

#endif // COMMANDREADER_H
