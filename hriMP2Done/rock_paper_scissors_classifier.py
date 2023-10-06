#!/home/ur3/598HRI/ece598hri-fa23-mps/ws_hri/env/bin/python
# *** change the first line to your virtual environment python path ***
import pickle
from os.path import join, exists
import time

import rospy
import rospkg
import numpy as np
from std_msgs.msg import String

from hand_gesture_classifier.msg import HandJointPos

from scipy.stats import mode


class RockPaperScissorsClassifier:
    def __init__(self, model_filename='new_game.pkl'):
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
        # pass
        self.label_mapping = {'rock': 0, 'paper': 1, 'scissors': 2, 'lizard' : 3, 'spock' : 4}
        self.reverse_label_mapping = {v: k for k, v in self.label_mapping.items()}

        self.timer = 0
        self.window_size = 150
        self.rock_probs = [0 for i in range(self.window_size)]
        self.paper_probs = [0 for i in range(self.window_size)]
        self.scissors_probs = [0 for i in range(self.window_size)]
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
        features = self.process_features(data)
    
        predicted_labels_encoded = self.model.predict(features)

        # # Get probability estimates
        probabilities = self.model.predict_proba(features)

        # Extract the probabilities for each label
        self.rock_probs[self.timer] = probabilities[0][0]
        self.paper_probs[self.timer] = probabilities[0][1]
        self.scissors_probs[self.timer] = probabilities[0][2]
        if self.timer <= self.window_size - 5:
            self.timer = self.timer + 1
        else:
            self.timer = 0
        # Rolling windows


        self.rock_prob_mean = np.mean(self.rock_probs)
        self.paper_prob_mean = np.mean(self.paper_probs)
        self.scissors_prob_mean = np.mean(self.scissors_probs)

        combined_window = [self.rock_prob_mean, self.paper_prob_mean, self.scissors_prob_mean]
        max_index = combined_window.index(max(combined_window))

        
        hand_gesture = self.reverse_label_mapping[max_index]
        
        # result = mode(predicted_labels_encoded)
        
        # if np.isscalar(result.mode):
        #     most_common_label_encoded = result.mode
        # else:
        #     most_common_label_encoded = mode(predicted_labels_encoded).mode[0]
  
        # hand_gesture = self.reverse_label_mapping[most_common_label_encoded]

        print("[DEBUG]", time.time(), predicted_labels_encoded, hand_gesture)
        # time.sleep(0.1)
        self.pub.publish(hand_gesture)
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
        
        X = np.array(data.x)
        thumb0_x = X[0]
        thumb0_y = X[1]
        thumb0_z = X[2]
        for i in range(0, X.shape[0], 3):
            X[i] -= thumb0_x
            X[i + 1] -= thumb0_y
            X[i + 2] -= thumb0_z
        X = X[6:].reshape(1, -1)
        # *************************************
        return X
        

if __name__ == '__main__':
    model_filename='original_game.pkl'
    rock_paper_scissors_classifier = RockPaperScissorsClassifier(model_filename=model_filename)
    rospy.spin()