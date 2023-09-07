#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

from ur3_kinematics import forward_k
from ur3_driver.msg import position

PI = 3.1415926535

class EndEffectorPositionPublisher:
    def __init__(self):
        self.ee_pos_pub = rospy.Publisher('ur3/ee_position', Float64MultiArray, queue_size=10)
        rospy.init_node('ee_pos_publisher', anonymous=True)
        rospy.Subscriber('ur3/position', position, self.joint_position_callback)

    def joint_position_callback(self, msg):
        thetas = msg.position
        curr = forward_k(thetas[0]-PI, thetas[1], thetas[2], thetas[3]+0.5*PI, thetas[4], thetas[5])
        ee_xyz = list(curr[0:3,3]*0.001) # get curr_pos in meters
        msg = Float64MultiArray()
        msg.data = ee_xyz
        self.ee_pos_pub.publish(msg)
        

if __name__ == '__main__':
    try:
        eepp = EndEffectorPositionPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass