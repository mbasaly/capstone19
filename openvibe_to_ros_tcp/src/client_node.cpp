//Based on http://www.linuxhowtos.org/C_C++/socket.htm
//
//This code is designed to receive TCP packets from the OpenVibe environment and publish them to the nav_goals node.

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "std_msgs/Int64.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
//#include "std_msgs/Header.h"
#include <sstream>
#define MESSAGE_FREQ 100

bool trial_begin = false;
bool action_complete = true;

void error(const char *msg) {
    perror(msg);
    exit(0);
}

void resultCallback(const move_base_msgs::MoveBaseActionResult& msg){
    int status = unsigned(msg.status.status);
    std::string result = msg.status.text;
    ROS_INFO_STREAM("Goal status: " << result);
    if(status > 0){
        action_complete = 1;
        trial_begin = 0;
    }
    else{action_complete = 0;}
}


void stateCallback(const move_base_msgs::MoveBaseActionGoal& msg){
    int status = msg.goal.target_pose.header.stamp.toSec();
    ROS_INFO_STREAM("Goal stamp: " << status);
    if (status > 0){
        action_complete = 0;
    }
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    int zero_counter = 0;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("chatter", 1000);
    ros::Subscriber nav_result_state = nh.subscribe("/move_base/result",1,resultCallback);
    ros::Subscriber nav_goal_state = nh.subscribe("/move_base/goal",1,stateCallback);
    //    ros::ServiceServer service = nh.advertiseService();
    ros::Rate loop_rate(MESSAGE_FREQ); // Set rate as defined in the macro MESSAGE_FREQ

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];

    if (argc < 3) {
        fprintf(stderr,"Usage: $ rosrun openvibe_to_ros_tcp client_node <hostname> <port>\n");
        exit(0);
    }

    // Open socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

    portno = atoi(argv[2]);
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    // Configure socket parameters
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);

    // Connect to socket
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    std_msgs::Int64 message;
    // std::stringstream ss;

    while(ros::ok()) {
        // ss.str(std::string()); // Clear contents of string stream
        bzero(buffer, 256);
        n = read(sockfd,buffer, 255); // Read msg from buffer
        if (n < 0)
            error("ERROR reading from socket");
        //        int data_received = 0;
        //        data_received << static_cast<int>(buffer[0]);
        if(buffer[0] == 0){
            message.data = 30;
            zero_counter++;
        }
        else {
            message.data = buffer[0]; //data is stored in first cell of the buffer array
            zero_counter = 0; //reset counter for next instance error-checking
        }
        if(zero_counter > 4){
            ROS_INFO_STREAM("Lost connectivity with TCP Server, TCP node exiting");
            //todo: Publish an error command?
            ros::shutdown();
        }
        //Shows signals being received on the terminal when node is run
        switch(message.data){
        case 30: ROS_DEBUG("Start of Trial"); trial_begin = true; break;
        case 1: ROS_DEBUG("Left Stimulation"); break;
        case 2: ROS_DEBUG("Right Stimulation"); break;
        case 32:ROS_DEBUG("End of Trial"); break;
        default: ROS_DEBUG("invalid packet"); break;
        }

        //ROS_INFO("I heard: %s", message.data.c_str());
        if(trial_begin && action_complete)
        {
            chatter_pub.publish(message); // Publish msg to chatter
            ROS_INFO_STREAM("Sending: " << message.data);
            ROS_DEBUG_STREAM("Only " << message.data << " was sent to the chatter topic");
            if (message.data == 32){action_complete = 0;}
        }
        ros::spinOnce();
    }

    close(sockfd);
    return 0;
}
