#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from blob_search import blob_search
from ur3_driver.msg import command
from ur3_driver.msg import position

import numpy as np

PI = 3.1415926535

class VisionSafety:

    def __init__(
        self,
        vel,
        accel,
    ):
        if vel > 2 or accel > 2:
            raise RuntimeError('vel and accel cannot be set larger than 2.')
        self.last_seen = [(100000, 100000)]
        self.update_state = 0
        self.base_pos = [360, 10]

        self.vel = vel
        self.accel = accel
        self.bridge = CvBridge()
        self.SPIN_RATE = 20
        self.loop_rate = rospy.Rate(self.SPIN_RATE)
        self.thetas = [0,0,0,0,0,0]
        self.dest1 = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
        self.dest2 = [90*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
        self.goal_index = 2
        self.break_flag = False
        self.curr_dest = self.dest2
        self.at_goal = 0
        self.command_pub = rospy.Publisher('ur3/command', command, queue_size=10)
        rospy.Subscriber('ur3/position', position, self.position_callback)
        rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        rospy.sleep(1)
        rospy.loginfo("Vision safety initialized.")
        


 
    def image_callback(self, data):
        # ****************************************
        # *** You may modify this part of code ***
        try:
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
        cv_image = cv2.flip(raw_image, -1)
        blob_image_center, im_with_keypoints = blob_search(cv_image, "green") # * you may need to modify blob_search function.
        self.object_pos = blob_image_center
        cv2.circle(im_with_keypoints, (360,10), 141, (0,0,255), 2)
        cv2.circle(im_with_keypoints, (360,10), 282, (0,255,0), 2)
        cv2.imshow("Vision Safety", im_with_keypoints)
        cv2.waitKey(2)    
        self.break_flag = self.safety_function()
        # ****************************************

    def position_callback(self, msg):
        self.thetas[0] = msg.position[0]
        self.thetas[1] = msg.position[1]
        self.thetas[2] = msg.position[2]
        self.thetas[3] = msg.position[3]
        self.thetas[4] = msg.position[4]
        self.thetas[5] = msg.position[5]
        
    def safety_function(self):
        # ****************************************
        # *** You may modify this part of code ***
        self.base_pos = [360, 10]
        
        if self.object_pos == []:
            self.object_pos = self.last_seen

        self.last_seen = self.object_pos
        dis_to_base_squared = (self.object_pos[0][0] - self.base_pos[0])**2 + (self.object_pos[0][1] - self.base_pos[1])**2
        print(dis_to_base_squared)
        if dis_to_base_squared < 20000:
            return True
        elif dis_to_base_squared < 80000:
            if self.vel > 0.5:
                self.vel = 0.25
                self.accel = 0.25   
                self.update_state = 1             
            return False
        else:
            if self.vel < 0.9:
                self.vel = 1.0
                self.accel = 1.0
                self.update_state = 1
            return False
        # ****************************************
        
    def move_arm(self, goal_index):
        driver_msg = command()
        if goal_index == 1:
            driver_msg.destination = self.dest1
            self.curr_dest = self.dest1
        elif goal_index == 2:
            driver_msg.destination = self.dest2
            self.curr_dest = self.dest2
        else:
            raise RuntimeError
        driver_msg.v = self.vel
        driver_msg.a = self.accel
        self.command_pub.publish(driver_msg)
        self.loop_rate.sleep()
        while self.at_goal==0:
            if self.break_flag:
                break
            if self.update_state == 1:
                self.update_state = 0
                break
            if  abs(self.thetas[0]-self.curr_dest[0]) < 0.0005 and \
                abs(self.thetas[1]-self.curr_dest[1]) < 0.0005 and \
                abs(self.thetas[2]-self.curr_dest[2]) < 0.0005 and \
                abs(self.thetas[3]-self.curr_dest[3]) < 0.0005 and \
                abs(self.thetas[4]-self.curr_dest[4]) < 0.0005 and \
                abs(self.thetas[5]-self.curr_dest[5]) < 0.0005:
                self.at_goal = 1
                return 1
            self.loop_rate.sleep()
        return 0

    def run(self, goal_index):
        while self.at_goal != 1 and not rospy.is_shutdown():
            self.move_arm(goal_index)
            self.loop_rate.sleep()
        self.at_goal = 0

def main():
    rospy.init_node('mp1_node')
    # ****************************************
    # *** You may modify this part of code ***
    vel = 0.25 # set a value between 0 and 2
    accel = 0.25 # set a value between 0 and 2
    # ****************************************
    vs = VisionSafety(vel, accel)
    while(not rospy.is_shutdown()):
        rospy.loginfo("go to goal 2")
        vs.run(2)
        rospy.loginfo("go to goal 1")
        vs.run(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass