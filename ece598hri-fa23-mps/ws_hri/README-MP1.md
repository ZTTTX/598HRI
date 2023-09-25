# ECE598HRI-Fa23-MP1
Mini Project 1 of ECE598HRI at University of Illinois at Urbana-Champaign, Fall 2023.

## Introduction
You will implement a vision safety human-robot interaction system. Here are a list of scripts that need your attention.
1.  ROS node for vision safety (to be done)
```
ws_hri/src/hri/scripts/mp1.py
```
2. source file of human (green block) detection algorithm
```
ws_hri/src/hri/scripts/blob_search.py
```
- You may need to tune color thresholds to have reliable detection.

## Setup Instructions
Build the packages in the catkin workspace.
```
cd ece598hri-fa23-mps/
cd ws_hri/
catkin_make
```

## How to Run
1. Turn on UR3. Make sure Emergency button and brakes are released.
2. Open the first terminal, and run
```
cd ws_hri && source devel/setup.bash
roslaunch ur3_driver vision_driver.launch
```
When you see the messages as below, the robot is connected to the computer.
```
[ INFO] [TIME]: Realtime port: Got connection
[ INFO] [TIME]: Secondary port: Got connection
```
3. Open the second terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hri mp1.py
```
The robot will start moving, and a window will pop up with a top-down view of the shared space. To terminate `mp1.py`, you need to press 'Ctrl+Z', and then run `kill -9 %1` multiple times until you see
```
[1]+  Killed                  rosrun hri mp1.py
```