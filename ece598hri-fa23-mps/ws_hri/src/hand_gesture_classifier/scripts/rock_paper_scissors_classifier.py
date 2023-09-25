#!/home/kang/anaconda3/envs/HRI/bin/python
# #!/home/ur3/ws_hri/env/bin/python
# *** change the first line to your virtual environment python path ***
import pickle
from os.path import join, exists

import rospy
import rospkg
import numpy as np
from std_msgs.msg import String

from hand_gesture_classifier.msg import HandJointPos


class RockPaperScissorsClassifier:
    def __init__(self, model_filename='lr_rock_paper_scissors.pkl'):
        """
        Initialize a rock-paper-scissors gesture classifier.
        inputs:
            - model_filename
        outputs:
            - None
        """
        # Load the hand gesture recognition model you trained.
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('hand_gesture_classifier')
        model_filepath = join(package_path, 'models', model_filename)
        if not exists(model_filepath):
            raise RuntimeError(model_filename+" is not found.")     
        with open(model_filepath, 'rb') as f:
            self.model = pickle.load(f)
            rospy.loginfo(model_filename+" is loaded.")
        rospy.init_node('hand_gesture_classifier')
        rospy.Subscriber('hand_joint_position', HandJointPos, self.callback)
        self.pub = rospy.Publisher('hand_gesture', String, queue_size=10)
        # **************************************
        # *** Feel free to add anything here ***
        pass
        # **************************************

    def callback(self, data):
        """
        Callback for recognizing the hand gesture given the hand position message.
        
        Publish 'rock', 'paper' or 'scissors' to the topic /hand_gesture.
        inputs:
            - data
                # HandJointPos message.
                - t
                    # time
                - x
                    # list of length (75, )
        outputs:
            - None
        """
        # *************************************
        # ********** To Be Done Here **********
        pass
        # *************************************
        

    def process_features(self, data):
        """
        Process the features of the message.
        inputs:
            - data
                # HandJointPos message.
                - t
                    # time
                - x
                    # list of length (75, )
        outputs:
            - X
                # numpy array.
                # size: (1, num_features)
        """
        # *************************************
        # ********** To Be Done Here **********
        X = None
        # *************************************
        return X
        

if __name__ == '__main__':
    model_filename='lr_rock_paper_scissors.pkl'
    rock_paper_scissors_classifier = RockPaperScissorsClassifier(model_filename=model_filename)
    rospy.spin()