#!/usr/bin/env python3
from os.path import join

import rospy
import rospkg
import numpy as np

from hand_gesture_classifier.msg import HandJointPos


class HandjointTalker:
    def __init__(self, rate=10, loop=False):
        """
        Initialize a hand joint talker.
        inputs:
            - rate
                # int
                # Publishing frequency. Unit: hz.
            - loop
                # bool
                # Replay the saved hand joint data in loop (True) or just once (False).
        outputs:
            - None
        """
        self.loop = loop
        # Load hand joint data.
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('hand_gesture_classifier')
        self.data = np.genfromtxt(join(package_path, 'datasets/raw/test_combined.csv'), delimiter=",")[1:,1:]
        self.data_len = len(self.data)
        self.curr_index = 0
        rospy.init_node('hand_joint_talker', anonymous=True)
        self.pub = rospy.Publisher('hand_joint_position', HandJointPos, queue_size=10)
        self.rate = rospy.Rate(rate)
        self.printed_finish_flag = False
        rospy.loginfo("Hand gesture replay starts.")
        return

    def publish(self):
        """
        Publish a hand joint message.
        """
        if not self.loop and self.curr_index >= self.data_len:
            if not self.printed_finish_flag:
                rospy.loginfo("Hand gesture replay is finished.")
                self.printed_finish_flag = True
            return
        if self.loop and self.curr_index == self.data_len:
            self.curr_index = 0
        hand_joint_pos_msg = HandJointPos()
        hand_joint_pos_msg.t = rospy.get_time()
        hand_joint_pos_msg.x = list(self.data[self.curr_index])
        self.pub.publish(hand_joint_pos_msg)
        self.curr_index += 1
        return
    
    def sleep(self):
        """
        Sleep at a frequency.
        """
        self.rate.sleep()
        return

if __name__ == '__main__':
    hand_joint_talker = HandjointTalker(rate=100, loop=False)
    try:
        while not rospy.is_shutdown():
            hand_joint_talker.publish()
            hand_joint_talker.sleep()
    except rospy.ROSInterruptException:
        pass
