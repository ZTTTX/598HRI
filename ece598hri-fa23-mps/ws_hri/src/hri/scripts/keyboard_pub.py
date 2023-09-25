#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard

class KeyboardPublisher:

    def __init__(self):
        rospy.init_node('keyboard_publisher', anonymous=True)
        self.pub = rospy.Publisher('keyboard_input', String, queue_size=10)
        # *******************************************************
        # ************ Modify based on what you need ************
        self.keys_of_interest = ['w', 'a', 's', 'd', 'r', 'e', 'i', 'o']
        # *******************************************************
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char in self.keys_of_interest:
                self.pub.publish(key.char)
        except AttributeError:
            pass

if __name__ == '__main__':
    try:
        KeyboardPublisher()
    except rospy.ROSInterruptException:
        pass

