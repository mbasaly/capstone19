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
#include <sstream>
#define MESSAGE_FREQ 100

void error(const char *msg) {
    perror(msg);
    exit(0);
}




int main(int argc, char *argv[]) {
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;


    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("chatter", 1000);
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
        message.data = buffer[0]; //data is stored in first cell of the buffer array

//Shows signals being received on the terminal when node is run
        switch(message.data){
          case 0: ROS_INFO("Start of Trial"); break;
          case 1: ROS_INFO("Left Stimulation"); break;
          case 2: ROS_INFO("Right Stimulation"); break;
          case 32:ROS_INFO("End of Trial"); break;
          default: ROS_INFO("invalid packet"); break;

        }

        //ROS_INFO("I heard: %s", message.data.c_str());
        chatter_pub.publish(message); // Publish msg to chatter


	    ros::spinOnce();
	}

    close(sockfd);
	return 0;
}
