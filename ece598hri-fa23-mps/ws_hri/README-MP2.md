# ECE598HRI-Fa23-MP2
Mini Project 2 of ECE598HRI at University of Illinois at Urbana-Champaign, Fall 2023.

## Introduction
You will implement a robot system to play rock-paper-scissors game with human. Here are a list of scripts that need your attention.
1. Jupyter Notebook used to train the hand gesture recognition model (to be done)
```
ws_hri/src/hand_gesture_classifier/hand_data_analysis.ipynb
```
2. ROS node which employs the trained hand gesture recognition model (to be done)
```
ws_hri/src/hand_gesture_classifier/scripts/rock_paper_scissors_classifier.py
```
3. ROS node which controls the robot to play against human (to be done)
```
ws_hri/src/hri/scripts/mp2.py
```
4. ROS node which publishes the recorded human joint positions in a test csv file
```
ws_hri/src/hand_gesture_classifier/scripts/hand_joint_talker.py
```
5. Script which uses Leap Motion to record human hand gestures and save as a csv file
```
/home/ur3/ws_hri/src/hand_gesture_classifier/LeapSDK/samples/hand_data_collection.py
```

## Setup Instructions
1. Create a Python3 virtual environment. If `virtualenv` is not found in the machine, contact TA.
```
cd ws_hri/
virtualenv -p python env
source env/bin/activate
pip install jupyter
pip install pandas
pip install scikit-learn
pip install matplotlib
pip install rospkg
```
2. Change the first line of `ws_hri/src/hand_gesture_classifier/scripts/rock_paper_scissors_classifier.py` by
```
#!{absolute path of ws_hri}/env/bin/python
```
For example, if you run
```
cd ws_hri/
pwd
```
and the output is `/home/ur3/abc/ece598hri-fa23-mps/ws_hri`, change the first line to
```
#!/home/ur3/abc/ece598hri-fa23-mps/ws_hri/env/bin/python
```

## How to Run
1. You need to train and save a hand gesture recognition model using `ws_hri/src/hand_gesture_classifier/hand_data_analysis.ipynb`. Run
```
cd ws_hri/
source env/bin/activate
jupyter notebook
```
and edit and run the notebook. A model file `ws_hri/src/hand_gesture_classifier/models/lr_rock_paper_scissors.pkl` will be generated after running the whole notebook.
2. Turn on UR3. Make sure Emergency button and brakes are released.
3. Open the first terminal, and run
```
cd ws_hri && source devel/setup.bash
roslaunch ur3_driver vision_driver.launch
```
When you see the messages as below, the robot is connected to the computer.
```
[ INFO] [TIME]: Realtime port: Got connection
[ INFO] [TIME]: Secondary port: Got connection
```
4. Open the second terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hri mp2.py
```
and robot will move towards the default position. When you see the messages as below, you can go to the next step.
```
[INFO] [TIME]: Robot rock-paper-scissors player initialized.
```
5. Open the third terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hand_gesture_classifier rock_paper_scissors_classifier.py
```
6. Open the fourth terminal, and run
```
cd ws_hri && source devel/setup.bash
rosrun hand_gesture_classifier hand_joint_talker.py
```

## How to Record Human Gestures
1. Connect Leap Motion to the computer.
2. Open the first terminal, and run
```
sudo leapd
```
If you see messages as below, Leap Motion is connected to the computer.
```
[Info] Leap Motion Controller detected: [Serial Number]
```
Note that when you finish recordings, make sure you kill the program. Press `Ctrl+Z` to stop the program, and then run `ps -ef | grep leapd`, where the left five-digit number is `{PID}`. Finally run `sudo kill -9 {PID}` to kill the program.
1. Open the second terminal, and run
```
Visualizer
```
Put one hand on top of Leap Motion, and you will see visualization of the tracked hand skeleton.
1. Open the third terminal, and run
```
cd ws_hri
cd src/hand_gesture_classifier/LeapSDK/samples
python2 hand_data_collection.py
```
When your hand is detected, you should see messages like below.
```
Frame id: 1603, timestamp: 1693004275344646
Framerate: 87
  Right hand
    Thumb finger, id: 20, length: 36.076557mm, width: 14.017673mm
    Index finger, id: 21, length: 40.708370mm, width: 13.389682mm
    Middle finger, id: 22, length: 46.383923mm, width: 13.150447mm
    Ring finger, id: 23, length: 44.599380mm, width: 12.513484mm
    Pinky finger, id: 24, length: 34.965088mm, width: 11.115455mm
```
To finish the recording, press 'Ctrl+C' to kill the program. You will see `ws_hri/src/hand_gesture_classifier/LeapSDK/samples/mp2_custom_data.csv` generated. You can change the name of the generated file by modifying the variable `indx_path` in `ws_hri/src/hand_gesture_classifier/LeapSDK/samples/hand_data_collection.py`. To train your custom hand gesture recognition models, move the generated csv files into `ws_hri/src/hand_gesture_classifier/datasets/raw`, name the csv files accordingly, and rerun the jupyter notebook.