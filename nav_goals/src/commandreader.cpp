#include <nav_goals/commandreader.h>


CommandReader::CommandReader()
{
    begin_trial_flag = false;
    end_trial_flag = false;

    left_stim_counter = 0;
    right_stim_counter = 0;
    counter_weight = 0;
    direction = 0;
    orientation = 0;
}

int CommandReader::getCounterWeight()
{
    return counter_weight;
}

int CommandReader::getDirection()
{
    return direction;
}

int CommandReader::getOrientation()
{
    return orientation;
}


void CommandReader::commandCallback(const std_msgs::Int64::ConstPtr& message)
{

   //Wait for '0' (start trial) stims to stop coming through
    while (!begin_trial_flag && left_stim_counter + right_stim_counter == 0)
    {
       if(message->data ==0)
       {begin_trial_flag = true;}
       switch(message->data){
        case 0: break;
        case 1: left_stim_counter++; ROS_INFO("Left detected"); break;
        case 2: right_stim_counter++; ROS_INFO("Right detected"); break;
        case 32: end_trial_flag = true; ROS_INFO("End of command"); break;
        default: ROS_INFO("Error: invalid stimulation detected"); break;
       }

    //Begin processing stimulations to determine command that will be sent
    //If '1' (left stim), increment left counter, If '2' (right stim), increment right counter. Wait for 'End of trial' stim
    }
    while(!end_trial_flag){
        switch(message->data){
        case 1: left_stim_counter++; break;
        case 2: right_stim_counter++; break;
        case 32: end_trial_flag = true; break;
        }
    }
  //compare left stim counter to right counter. If left > right, send left command. If right > left, send right command.
   //Command is weighted - more difference between counters means larger distance is sent

    if(left_stim_counter > right_stim_counter)
    {
        counter_weight = left_stim_counter/(left_stim_counter+right_stim_counter);
        direction = 1;

    }
    else if (right_stim_counter > left_stim_counter)
    {
        counter_weight = right_stim_counter/(left_stim_counter+right_stim_counter);
        direction = 2;
    }
    else direction = 0;
}


