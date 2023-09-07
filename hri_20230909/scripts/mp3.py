#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_kinematics import forward_k, ur_invk

PI = 3.1415926535

class SharedControl:
    def __init__(
        self,
        ee_start_xy=(0.304, -0.074),
        ee_goal_xy=(0.284, 0.296),
        ee_z=0.05,
    ):
        # ***********************************************
        # ************ Add whatever you need ************
        # recommended end-effector motion range: x [0.189, 0.304], y [-0.074, 0.296], z [0.05, 0.20]
        self.iter = 0
        self.path_len = 0
        self.scale = 1
        self.thetas = [0,0,0,0,0,0]
        self.ee_next_pos = np.zeros(3)
        self.ee_curr_pos = np.zeros(3)
        self.human_policy = np.zeros(2) # on XY plane
        self.robot_policy = np.zeros(2) # on XY plane
        self.SPIN_RATE = 20
        self.ee_start_pos = np.array([ee_start_xy[0], ee_start_xy[1], ee_z])
        self.ee_goal_pos = np.array([ee_goal_xy[0], ee_goal_xy[1], ee_z])
        self.path = [self.ee_start_pos]
        self.ee_z = ee_z
        self.pub_cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        self.pub_control_clock = rospy.Publisher('control_clock', String, queue_size=10)
        rospy.init_node('shared_control', anonymous=True)
        self.loop_rate = rospy.Rate(self.SPIN_RATE)
        rospy.Subscriber('keyboard_input', String, self.human_policy_callback)
        rospy.Subscriber('control_clock', String, self.shared_control_callback)
        rospy.Subscriber('ur3/position', position, self.position_callback)
        rospy.sleep(1)
        rospy.loginfo("Shared control is initialized.")
        self.go_to_start_pos()
        # ***********************************************

    def human_policy_callback(self, msg):
        # *****************************************
        # ************ To be done here ************
        self.human_policy = np.array([0,0])
        # *****************************************
        
    def shared_control_callback(self, dummy_msg):
        if np.sqrt((self.ee_curr_pos[0]-self.ee_goal_pos[0])**2+(self.ee_curr_pos[1]-self.ee_goal_pos[1])**2)<0.01:
            rospy.loginfo("Task completed. iter: {}, path length: {}".format(self.iter, self.path_len))
            return
        # *****************************************
        # ************ To be done here ************
        # * policy is a weighted sum of robot policy and human policy
        policy = np.zeros(2)
        # *****************************************
        if np.linalg.norm(policy) > 0.05:
            rospy.loginfo("Policy is too fast. Robot is paused.")
            policy = np.zeros(2)
        self.iter += 1  
        self.ee_next_pos[0] = self.ee_next_pos[0] + policy[0]
        self.ee_next_pos[1] = self.ee_next_pos[1] + policy[1]
        self.ee_next_pos[2] = self.ee_z
        self.path.append(self.ee_next_pos)
        self.path_len += np.linalg.norm(policy)
        self.move_to_position()

    def get_robot_policy(self):
        # *****************************************
        # ************ To be done here ************
        robot_policy = np.zeros(2)
        # *****************************************
        return robot_policy
    
    def go_to_start_pos(self):
        self.ee_next_pos = self.ee_start_pos
        self.move_to_position()

    def position_callback(self, msg):
        self.thetas[0] = msg.position[0] 
        self.thetas[1] = msg.position[1]
        self.thetas[2] = msg.position[2]
        self.thetas[3] = msg.position[3] 
        self.thetas[4] = msg.position[4]
        self.thetas[5] = msg.position[5]
        curr = forward_k(self.thetas[0]- PI, self.thetas[1], self.thetas[2], self.thetas[3]+ (0.5*PI), self.thetas[4], self.thetas[5])
        self.ee_curr_pos = curr[0:3,3]*0.001 # get end-effector current position in meters

    def move_to_position(self):
        spin_count = 0
        at_goal = 0
        joint_state = ur_invk(self.ee_next_pos[0],self.ee_next_pos[1],self.ee_next_pos[2],0)
        driver_msg = command()
        driver_msg.destination = joint_state
        driver_msg.v = 0.5
        driver_msg.a = 0.5
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
                self.pub_control_clock.publish("dummy message")
                return
            self.loop_rate.sleep()
            if(spin_count >  self.SPIN_RATE*5):
                self.pub_cmd.publish(driver_msg)
                rospy.loginfo("Just published again driver_msg")
                spin_count = 0
            spin_count = spin_count + 1

def main():
    sc = SharedControl()
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass