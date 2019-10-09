echo "Launching Gazebo Simulation Node of Turtlebot in Headless Mode"
read -p "Press [Enter] to continue"
roslaunch turtlebot3_gazebo turtlebot3_world_headless.launch recording:=true gui:=false headless:=true

