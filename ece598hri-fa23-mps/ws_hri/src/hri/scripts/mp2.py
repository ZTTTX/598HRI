#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from ur3_kinematics import ur_invk
from ur3_driver.msg import command
from ur3_driver.msg import position

class RobotPlayer:
    def __init__(self):
        self.SPIN_RATE = 20
        self.at_goal = 0
        self.robot_gestures = {} # end effector positions corresponding to rock/paper/scissors gestures of the robot
        self.robot_gestures['default'] = [0.189, 0.111, 0.18]
        self.robot_gestures['rock'] = [0.189, -0.074, 0.06]
        self.robot_gestures['paper'] = [0.189, 0.111, 0.06]
        self.robot_gestures['scissors'] = [0.189, 0.296, 0.06]
        self.goal = None
        self.thetas = [0,0,0,0,0,0]
        self.loop_rate = rospy.Rate(self.SPIN_RATE)
        self.pub_cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        rospy.Subscriber('ur3/position', position, self.position_callback)
        rospy.Subscriber("hand_gesture", String, self.gesture_callback)
        rospy.sleep(1)
        self.goal = self.robot_gestures['default'] 
        self.move_to_goal()
        rospy.loginfo("Robot rock-paper-scissors player initialized.")

    def gesture_callback(self, data):
        # *****************************************
        # ************ To be done here ************
        pass
        # *****************************************
            
    def position_callback(self, msg):
        self.thetas[0] = msg.position[0]
        self.thetas[1] = msg.position[1]
        self.thetas[2] = msg.position[2]
        self.thetas[3] = msg.position[3]
        self.thetas[4] = msg.position[4]
        self.thetas[5] = msg.position[5]

    def move_to_goal(self):
        spin_count = 0
        at_goal = 0
        if self.goal is None:
            rospy.loginfo("goal is None.")
            raise RuntimeError
        joint_state = ur_invk(self.goal[0], self.goal[1], self.goal[2], 0)
        driver_msg = command()
        driver_msg.destination = joint_state
        driver_msg.v = 1
        driver_msg.a = 1
        self.pub_cmd.publish(driver_msg)
        self.loop_rate.sleep()
        while(at_goal == 0):
            if( abs(self.thetas[0]-driver_msg.destination[0]) < 0.0005 and \
                abs(self.thetas[1]-driver_msg.destination[1]) < 0.0005 and \
                abs(self.thetas[2]-driver_msg.destination[2]) < 0.0005 and \
                abs(self.thetas[3]-driver_msg.destination[3]) < 0.0005 and \
                abs(self.thetas[4]-driver_msg.destination[4]) < 0.0005 and \
                abs(self.thetas[5]-driver_msg.destination[5]) < 0.0005 ):
                at_goal = 1
            self.loop_rate.sleep()
            if(spin_count >  self.SPIN_RATE*5):
                self.pub_cmd.publish(driver_msg)
                rospy.loginfo("Just published again driver_msg")
                spin_count = 0
            spin_count = spin_count + 1

def main():
    rospy.init_node('mp2_robot_player_node')
    rp = RobotPlayer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
