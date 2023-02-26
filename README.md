# Implementation of A* on ROS Turtlebot
This repository contains code to find a path between the start node and the goal node for ROS Turtlebot using A* algorithm. The action-set is defined such that it satisfies the non-holonomic constraints of Turtlebot. The program takes left and right wheel velocities for Turtlebot as input. The action set is defined as below:  

[0, RPM1] [RPM1, 0] [RPM1, RPM1] [0, RPM2] [RPM2, 0] [RPM2, RPM2] [RPM1, RPM2] [RPM2, RPM1]

## Requirements: 
1. Python 3
2. ROS 1

## Required Libraries:
1. pygame
2. math
3. heapq
4. time
5. functools

## Instructions to run the code
### For point robot with non-holonomic constraints:
```
python A-star_point_robot.py
```

Test case 1: start co-ordinates = (1, 1, 0), end co-ordinates = (5, 1) 

Test case 2: start co-ordinates = (1, 1, 0), end co-ordinates = (9, 9) 

Radius = 0.105, Clearance = 0.3  

<img src = "https://github.com/AbhijitMahalle/A-star_on_turtlebot/blob/master/gif/output1.gif">  

### For ROS Turtlebot with holonomic constraints:
To spawn the robot in Gazebo:
```
roslaunch A-star_on_turtlebot turtlebot_map.launch x_pos:=-4 y_pos:=-4 z_pos:=-0
```
 
To run the publisher file
```
cd ~/A-star_on_turtlebot/src
python turtlebot.py
```
Test case 1: start co-ordinates = (1, 1, 0), end co-ordinate = (9, 9)  
Radius = 0.105, Clearance = 0.3, RPM1 = 15, RPM1 = 10  

<img src = "https://github.com/AbhijitMahalle/A-star_on_turtlebot/blob/master/gif/turtlebot.gif" width = "500" height = "500">  

