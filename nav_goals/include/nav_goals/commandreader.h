#ifndef COMMANDREADER_H
#define COMMANDREADER_H
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <vector>



class CommandReader
{
public:
    CommandReader();
    void commandCallback(const std_msgs::Int64::ConstPtr& message);
    int getCounterWeight();
    int getDirection();
    int getOrientation();

protected:

private:
    bool begin_trial_flag;
    bool end_trial_flag;
    int left_stim_counter;
    int right_stim_counter;
    int counter_weight;
    int direction;
    int orientation;

    ros::NodeHandle nh;

};

#endif // COMMANDREADER_H
