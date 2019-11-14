# NavMind - Integrating BCI technology with the ROS Navigation Stack
Control of Robotic Interface using high-level BCI commands that allow ROS-enabled robotic interfaces to traverse complex environments.
This project was completed as a Capstone Project as part of a degree at the University of Technology Sydney.


## Environment
Robotic Interface: ROS Kinetic running on Ubuntu 16.04, configured as an NTP server

EEG Interface: OpenVibe runing on Windows 10

Turtlebot: Raspian Stretch (debian), configured as an NTP client of the ROS machine.

## Usage
Usage of the project is broken into 3 distinct phases - signal acquisition and training, robot preparation, and online commands control. The acquisition phase of the project also acts as a testbed for tuning ROS Navigation, as it sends goals that are the direction of the current trial (i.e. Right Arrow being shown on the screen will send the robot right). 

This project presupposes a basic understanding of using OpenVibe to perform classification of MotorImagery signals with an EEG interface. The project uses a Cognionics Quick-20 headset with 6 active channels and 1 reference channel, as can be seen in the OpenVIBE scenarios. If you want to use a different set of electrodes, you should modify the associated Channel Selector, Reference Channel, Laplacian, and Temporal Filter boxes to match. 

Refer to the OpenVibe tutorials on MI here for more information: <http://openvibe.inria.fr/motor-imagery-bci/>

### Acquisition
First things first - train up a model on OpenVibe. Follow the instructions in the above link for details on training a model. If you use the same headset (Cognionics Quick-20), the Acquisition scenario in this git repo should work as is. 

Run the classifier to train the signal accordingly.

### Robot Preparation
Ensure you have followed the ROS tutorials for setting up a catkin workspace and have installed all relevant Turtlebot3 packages. Follow the tutorials on the ROBOTIS site and verify you can make the robot navigate (follow them start to finish without using the EEG for anything). This is easiest if the gateway device (router) you use has an internet connection so that the TB doesn't experience issues with NTP time sync. If not, make sure the TB is set up as an NTP client of the ROS Machine, which is the NTP server.

#### Step 1:
Bring up ROS on your Ubuntu machine.

```roscore```

#### Step 2:
The Turtlebot being used should have the script ``tb3_bringup.sh ``in its home directory for this to work well. Run this script, or the standard TB bringup command:

```$ roslaunch turtlebot3_bringup turtlebot3_robot.launch```

#### Step 3:
If you have already built a map using the robot's SLAM abilities, go to Step 4. 

Use ``tb3_slam_bringup.sh`` on the Ubuntu machine to bring up the SLAM mapping functionality of ROS (visualised using RVIZ) and then use the teleoperation node or Robot Steering plugin in ``rqt``. Build a full map of the environment. When you are happy with the map that's been created (refer to the RVIZ screen), use the ``tb3_savemap.sh`` to export the map to your home directory (an archived/timestamped version will be added to tb_maps as well).

#### Step 4:
Launch the saved map:

```tb3_launchmap.sh```

Test the map is working by setting the initial pose of the robot and sending a couple of navigation goals. If you're happy with the performance, proceed. If not, refer to ROBOTIS and ROS navigation documentation to tweak the parameters of the navigation node to be tuned for your requirements.

#### You are now ready for Online Usage.

### Online Command Control
Now that you have a classifer trained and the robot ready, you should be able to run the rosnodes that enable TCP communication and control of your robot. Make sure the navigation package is still running.

#### Step 1:
Open the nav_goals node:

```rosrun nav_goals nav_goals```

You should see ``Welcome to DriveMind``.

#### Step 2:
Open the OpenVibe TCP node:

```rosrun ov_ros_tcp client_node `ip-address` `tcp-port` ```
For example, if we're publishing our OpenVibe commands from 10.0.0.20 on TCP port 5680, then the command would be:

```rosrun ov_ros_tcp client_node 10.0.0.20 5680```

#### Step 4:
Run the OpenVibe Online scenario, making sure it has been adapted to reflect any changes made in the acquisition and classification scenarios. Most errrors tend to occur when you have failed to use the same number of channels or used the wrong frequency range in the filter.

#### You should now be controlling the robot using MI commands! 


