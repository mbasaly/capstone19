#ifndef COMMANDREADER_H
#define COMMANDREADER_H
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <nav_msgs/Odometry.h>
#include <mutex>

#define START_STIM 30
#define END_STIM 32
#define LEFT_STIM 1
#define RIGHT_STIM 2

class CommandReader
{
public:
    CommandReader(ros::NodeHandle nh);
    void commandCallback(const std_msgs::Int64ConstPtr &message);
    int getCounterWeight();
    int getDirection();
    int getOrientation();
    bool getEndFlag();
    bool getStimState();
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void processCommand();
    void nextTrial();

protected:

private:
    bool begin_trial_flag;
    bool end_trial_flag;
    bool stim_received;

    int current_stim;
    int left_stim_counter;
    int right_stim_counter;
    double counter_weight;
    int direction;
    int orientation;

    std::mutex counter_mtx_;
    std::mutex direction_mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Subscriber odom_sub_;

};

#endif // COMMANDREADER_H
