import os, sys, inspect
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))
arch_dir = '../lib'
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))
print(os.path.abspath(os.path.join(src_dir, arch_dir)))
import Leap
import ctypes
import csv

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        global indx_array, indx_path, timestamp
        frame = controller.frame()
        timestamp.append(frame.timestamp)
        serialized_tuple = frame.serialize
        serialized_data = serialized_tuple[0]
        serialized_length = serialized_tuple[1]
        data_address = serialized_data.cast().__long__()
        buffer = (ctypes.c_ubyte * serialized_length).from_address(data_address)
        print "Frame id: %d, timestamp: %d" % (
              frame.id, frame.timestamp)
        framerate = frame.current_frames_per_second
        print "Framerate: %d" %(framerate)
        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"
            print "  %s" % (handType)
            frame_hand_data = []
            for finger in hand.fingers:
                print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                    self.finger_names[finger.type],
                    finger.id,
                    finger.length,
                    finger.width)
                for b in range(0, 4):
                    bone = finger.bone(b)
                    for i in range(3):
                        frame_hand_data.append(bone.prev_joint[i])
                    if b == 3:
                        for i in range(3):
                            frame_hand_data.append(bone.next_joint[i])
            indx_array.append(frame_hand_data)

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"
        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"
        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"
        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():
    global indx_array, indx_path, timestamp
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    dim_names = 'xyz'
    joint_names = []
    for f in range(5):
        for b in range(5):
            for d in range(3):
                joint_name = finger_names[f]+'-'+str(b)+'-'+dim_names[d]
                joint_names.append(joint_name)
    table_title = ['time']+joint_names
    indx_array = []
    timestamp = []
    listener = SampleListener()
    controller = Leap.Controller()
    controller.add_listener(listener)
    frame = controller.frame()  
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        controller.remove_listener(listener)
        with open(indx_path, 'wb') as f:
            wr = csv.writer(f, delimiter=',')  
            wr.writerow(table_title)       
            for stamp, row in zip(timestamp, indx_array):
                wr.writerow([stamp]+row)


if __name__ == "__main__":
    # *************************************
    # ** Modify the filename as you need **
    indx_path = 'mp2_custom_data.csv'
    # *************************************
    main()
