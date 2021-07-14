#! /bin/sh
SETUP="source devel/setup.bash;"
xterm -e "${SETUP} roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "${SETUP} roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "${SETUP} roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "${SETUP} rosrun pick_objects pick_objects" &
