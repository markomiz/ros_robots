#! /bin/sh

xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 1
xterm -e "roslaunch localised_bot teleop.launch" 
