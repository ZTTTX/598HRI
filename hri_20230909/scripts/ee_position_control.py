#!/usr/bin/env python3
import rospy

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_kinematics import ur_invk

class EndEffectorPositionControl:
    def __init__(
        self,
    ):
        # recommended end-effector motion range: x [0.189, 0.304], y [-0.074, 0.296], z [0.05, 0.20]
        self.default_ee_pos = [0.2, 0.1, 0.2]
        rospy.init_node('ee_pos_control', anonymous=True)
        self.pub_cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        self.SPIN_RATE = 20
        self.loop_rate = rospy.Rate(self.SPIN_RATE)
        self.thetas = None
        self.pos_call_back = rospy.Subscriber('ur3/position', position, self.position_callback)
        rospy.sleep(0.5)
        rospy.loginfo("End-effector position control is initialized.")
        self.robot_init()
    
    def robot_init(self):
        print("start")
        self.move_to_position(self.default_ee_pos)

    def position_callback(self, msg):
        self.thetas = msg.position

    def move_to_position(self, goal_pos):
        spin_count = 0
        at_goal = 0
        joint_state = ur_invk(goal_pos[0], goal_pos[1], goal_pos[2], 0)
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
                return
            self.loop_rate.sleep()
            if(spin_count >  self.SPIN_RATE*5):
                self.pub_cmd.publish(driver_msg)
                rospy.loginfo("Just published again driver_msg")
                spin_count = 0
            spin_count = spin_count + 1
    
    def run(self):
         while not rospy.is_shutdown():
            print("=====================================================")
            command_input = input("Input end-effector position \nexample 0.2 0.1 0.1\
                \nrecommended end-effector motion range \nx: [0.189, 0.304] \ny: [-0.074, 0.296] \nz: [0.05, 0.20]\n")
            ee_goal_pos = [float(entry) for entry in command_input.split(" ")]
            if len(ee_goal_pos) == 3 and \
                (0.189 <= ee_goal_pos[0] <= 0.304) and \
                (-0.074 <= ee_goal_pos[1] <= 0.296) and \
                (0.05 <= ee_goal_pos[2] <= 0.2):
                self.move_to_position(ee_goal_pos)
            else:
                print("Please input valid end-effector position.")

if __name__ == '__main__':
    try:
        eepc = EndEffectorPositionControl()
        eepc.run()
    except rospy.ROSInterruptException:
        pass