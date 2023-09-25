# ECE598HRI-Fa23-MP3
Mini Project 3 of ECE598HRI at University of Illinois at Urbana-Champaign, Fall 2023.

## Introduction
You will implement a robot system with human in the loop to reach a goal while avoiding obstacles. Here are a list of scripts that need your attention.
1. ROS node which controls the robot using a shared control scheme (to be done)
```
ws_hri/src/hri/scripts/mp3.py
```
2. ROS node which receives human input from keyboard
```
ws_hri/src/hri/scripts/keyboard_pub.py
```
3. ROS node which publishes end-effector position (auxiliary node)
```
ws_hri/src/hri/scripts/ee_position_publisher.py
```
4. ROS node which controls the robot to reach a goal defined by user (auxiliary node)
```
ws_hri/src/hri/scripts/ee_position_control.py
```

## Setup Instructions
1. If you encounter error `No module named 'pynput'`, install `pynput` in the global setting.
```
pip install pynput
```
If you still encounter errors, contact TA.

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
rosrun hri keyboard_pub.py
```
4. Open the third terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hri mp3.py
```
The robot will start moving with the robot policy. Go back to the second terminal and start human input to perform human-in-the-loop control.


## How to Use Auxiliary ROS Nodes
1. Turn on UR3. Make sure Emergency button and brakes are released.
2. Open the first terminal, and run
```
cd ws_hri && source devel/setup.bash
roslaunch ur3_driver vision_driver.launch
```
3. Open the second terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hri ee_position_publisher.py
```
If you run `rostopic echo ur3/ee_position`, you can check the current robot end-effector position.
4. Open the third terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hri ee_position_control.py
```
Input your desired end-effector XYZ position, for example `0.2 0.1 0.1`, hit ENTER key, and the robot will reach to the corresponding position.