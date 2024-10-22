#!/usr/bin/env python

import rospy
import threading
from sensor_msgs.msg import JointState

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # Adjust the rate as needed
        self.current_positions = [0.0, 0.0]  # Initial positions
        self.lock = threading.Lock()

    def update_positions(self):
        while not rospy.is_shutdown():
            try:
                # User input for new positions
                new_positions = input("Enter new shoulder and elbow positions (e.g., 1.0 0.5): ").split()
                new_positions = [float(pos) for pos in new_positions]
                with self.lock:
                    self.current_positions = new_positions
            except Exception as e:
                rospy.logwarn("Invalid input: {}".format(e))

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.name = ['shoulder_joint', 'elbow_joint']

        while not rospy.is_shutdown():
            with self.lock:
                joint_state.position = self.current_positions
            joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = JointStatePublisher()
    # Start input thread
    threading.Thread(target=publisher.update_positions, daemon=True).start()
    # Start publishing joint states
    publisher.publish_joint_states()

