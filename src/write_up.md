# Write Up for Home Service Project


## Packages

### Localisation

For localisation my launch file (localised_bot)/acml_demo.launch is run this launches a node which utilises the ros amcl package, taking in map, laser scans and transform messages and outputting an estimated pose.

### Navigation

The ROS navigation stack is used to find paths between goals - the node responsible for this is also launched in (localised_bot)/acml_demo.launch and relies on tuning of the parameters in the launch file itself and the config files (e.g. base_local_planner_params.yaml).

### Mapping

The gmapping package was used to deveop a 2d occupancy grid map from laser and pose data.

### Other
Rviz is used for visualisation, Gazebo for simulating the world and teleop_twist_keyboard when manual navigation was required. 

Info about packages I wrote below.


## Stages

Each stage relies on some of the packages above (obvious which ones from scripts and launch files)

### SLAM Testing - test_slam.sh
My robot is launched inside a building model with keyboard navigation enabled - driven around to map the world and generate a map

### Navigation Testing - navigation_test.sh
My robot is launched inside the same building model now with access to the map that was generated - use rviz to generate markers for the robot to navigate to - this was useful for tuning parameters for reliable path following of the robot 

### Pick Ojects - pick_objects.sh
Created a pick_objects node which would generate markers for the robot to follow and display success messages when they were reached

### Add Markers - add_markers.sh
Created an add_markers node to generate markers which would roughly coincide with the timings of the pick objects node arriving at thise positions but not be able to communicate or sync

### Home Service - home_service.sh
This home_service is similar to the add_markers one, however it subscribes to pose data from acml to detect when the robot has reached the markers so that the markers can dissapear when the robot reaches them

#### Future Improvements
Nodes would be a bit nicer written as objects and would be more flexible if they read positions and pauses from a file rather than having these values hard coded in C++ 
