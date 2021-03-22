# way points manager

## how to
1. clone package into a ros catkin workspace
2. run catkin_make under that workspace
3. source devel/setup.bash from that workspace
4. run ```roslaunch way_points_manager sample.launch```

## config
Edit ```config/sample.yaml``` to edit/add/remove waypoints. For each waypoint, pose information has to be provided, including point and orientation.

## create new mission
1. Copy ```config/sample.yaml``` and edit the waypoints.
2. Copy ```launch/sample.launch``` and edit the launch file to load the new yaml file you just created.
3. source devel/setup.bash from that workspace
4. run ```roslaunch way_points_manager the_new_file_you_just_created.launch```
