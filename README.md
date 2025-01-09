# Robot Programming Assesment 
## Summary
### In this solution, the robot is equipped with a camera, lidar and odometry sensor to runs autonomously using the waypoints navigation and detects/report the coloured objects and their locations in the environment.

## 1.Using this command to run the world in Gazebo
### 1.1 For running the Default Gazebo world 
```bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
```
### 1.2 For running My_world (saved my world)
```bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/cmp9767_tutorial/worlds/my_testworld.world
```

## 2.Using this command to run the navigation 
### 2.1 For running the Defauft map 
```bash
ros2 launch limo_navigation limo_navigation.launch.py
```
### 2.2 For running My_map (saved my map)
```bash
ros2 launch limo_navigation limo_navigation.launch.py map:=src/cmp9767_tutorial/maps/my_map.yaml use_sim_time:=true
```
## 3.Using this code - update the file and run the python file (run this code before running any python files)
```bash
colcon build --symlink-install
```
```bash
source install/setup.bash
```
### 3.1 Using the demo_inspection.py file for running the Robot Autonomously 
```bash
ros2 run cmp9767_tutorial demo_inspection
```
### 3.2 Using the detector_3d.py file for detecting the colour of the objects
```bash
ros2 run cmp9767_tutorial detector_3d
```
### 3.3 Using the counter_3d.py file for counting the coloured objects
```bash
ros2 run cmp9767_tutorial counter_3d 
```
