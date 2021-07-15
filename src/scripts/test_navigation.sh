#! /bin/sh
SETUP="source devel/setup.bash;"
xterm -e "${SETUP} roslaunch my_robot world.launch" &
sleep 5
xterm -e "${SETUP} roslaunch localised_bot amcl_demo.launch" &
sleep 5
xterm -e "${SETUP} roslaunch turtlebot_rviz_launchers view_navigation.launch" &
