# Write Up for Home Service Project


## Packages

### Localisation

For localisation my launch file (localised_bot)/acml_demo.launch is run this launches a node which utilises the ros amcl package, taking in map, laser scans and transform messages and outputting an estimated pose. 

How does it work though? AMCL (Adaptive Monte Carlo Localisation) is a probabalistic, particle based approach - particles are generated around the robot with poses - these particles are then each given a probability of being resampled based on how likely their poses are to give the sensor readings the actual robot is getting. The ones that are most likely or closest to the robots actual pose are more likely to be sampled. After the robot has moved a set minimum distance the odometry data is passed to the particles to update their location (with some error factor that can be adjusted). Then the process of resampling repeats. After a few iterations of this the surviving particles converge on the actual robot location. 

### Navigation

The ROS navigation stack is used to find paths between goals - the node responsible for this is also launched in (localised_bot)/acml_demo.launch and relies on tuning of the parameters in the launch file itself and the config files (e.g. base_local_planner_params.yaml).

The navigation is comprised roughly into two main steps. The first is discretisation (in this case creating a graph by sampling the space and creating edges between points where there in no obstacle in between) and the second being a search of that graph for a 'good' path. To make this search efficient and effective weights can be assigned to the edges based on their proximity to the goal and the time taken to traverse the edge (other factors can also be counted in). Then the graph search prioritises the paths with shortest distance such that the first solution found by the algorithm will be the shortest path (given that graph, not neccessarily the actual shortest possible path between the points).

### Mapping

The gmapping package was used to deveop a 2d occupancy grid map from laser and pose data.
The occupancy grid approach discretises space into a grid of squares ( cubes in 3d version) or *cells* (voxels in 3d). Ideally each of these cells take a value of being unknown, occupied or unoccupied. But in the real world (or simulations) we aren't sure exacly if something is occupied or not. So we store a probility of a cell being occupied or not, then based on sensor data we can update the probabilities based on a binary bayes filter. 

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
