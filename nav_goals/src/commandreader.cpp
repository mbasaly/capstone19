#include <nav_goals/commandreader.h>

CommandReader::CommandReader(ros::NodeHandle nh) : nh_(nh)
{
    begin_trial_flag = false;
    end_trial_flag = false;

    current_stim = -1;
    stim_received = 0;

    left_stim_counter = 0;
    right_stim_counter = 0;
    counter_weight = 0.0;
    direction = 0;
    orientation = 1; //180deg
    command_sub_ = nh_.subscribe("/chatter",1000,&CommandReader::commandCallback, this);
    //    odom_sub_ = nh_.subscribe("/odom",1,&CommandReader::odomCallback, this);
    //    ROS_INFO_STREAM("Odom sub topic " << odom_sub_.getTopic());
    ROS_INFO_STREAM("Command sub topic "<< command_sub_.getTopic());

}

double CommandReader::getCounterWeight()
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

bool CommandReader::getEndFlag()
{
    return end_trial_flag;
}

bool CommandReader::getStimState()
{
    return stim_received;
}


void CommandReader::commandCallback(const std_msgs::Int64ConstPtr& message)
{
    if(message->data == START_STIM || message->data == LEFT_STIM || message->data == RIGHT_STIM || message->data == END_STIM){
        ROS_INFO_STREAM("New command received: " << message->data);
        current_stim = message -> data;
        stim_received = true;
    }
    else ROS_INFO_STREAM("Callback waiting for correct command.");
}

void CommandReader::nextTrial(){
    end_trial_flag = false;
    begin_trial_flag = false;
    stim_received = false;
    left_stim_counter = 0;
    right_stim_counter = 0;
}


void CommandReader::processCommand(){
    if(stim_received)
    {

        if(begin_trial_flag == false)
        {
            if(current_stim == START_STIM){begin_trial_flag = true; ROS_DEBUG_STREAM("Begin trial flag(should be 1): " << begin_trial_flag);}

            else{ROS_INFO_STREAM("Command not accepted. Waiting for start of trial. Received: " << current_stim);
            }
        }

        ROS_INFO_STREAM("Trial begins");
        ROS_DEBUG_STREAM("end trial flag is currently(should be 0): "<<end_trial_flag);
        //Begin processing stimulations to determine command that will be sent
        //If '1' (left stim), increment left counter, If '2' (right stim), increment right counter. Wait for 'End of trial' stim

        if(end_trial_flag == false){
            switch(current_stim){
            case START_STIM: break;
            case LEFT_STIM: left_stim_counter++;ROS_INFO_STREAM("Left Counter: " << left_stim_counter); break;
            case RIGHT_STIM: right_stim_counter++;ROS_INFO_STREAM("Right Counter: " << right_stim_counter); break;
            case END_STIM: end_trial_flag = true; ROS_INFO_STREAM("End of command. Calculating goal."); break;
            default: ROS_INFO_STREAM("Invalid command detected"); break;
            }
        }
        //compare left stim counter to right counter. If left > right, send left command. If right > left, send right command.
        //Command is weighted - more difference between counters means larger distance is sent

        if(left_stim_counter > right_stim_counter)
        {


            counter_weight = static_cast<double>(left_stim_counter)/(left_stim_counter+right_stim_counter);

            direction = 1;

        }
        else if (right_stim_counter > left_stim_counter)
        {
            counter_weight = static_cast<double>(right_stim_counter)/(left_stim_counter+right_stim_counter);
            direction = 2;
        }
        else direction = 0;

    }
    else {ROS_INFO_STREAM("No valid stimulations received. Waiting...");
        ros::Duration(2).sleep(); //Wait 2 seconds

    }
    ROS_INFO_STREAM("Direction bias: " << counter_weight);
    stim_received = 0;
}



