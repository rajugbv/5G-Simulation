#!/usr/bin/env python

import rospy
import threading
from sensor_msgs.msg import JointState
from time import sleep

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # Adjust the rate as needed
        self.current_positions = [0.1, 0.0, 0.0]  # Default positions to avoid immediate reversion
        self.lock = threading.Lock()
        self.update_needed = False  # Flag to indicate when an update is queued

    def enqueue_positions(self):
        # Simulate receiving 5 sets of positions for demonstration; replace with actual input logic as needed
        position_sets = [
            [0.1, 0.2, 0.3],
            [0.2, 0.1, 0.0],
            [0.3, 0.3, 0.3],
            [0.4, 0.0, 0.1],
            [0.5, 0.2, 0.4]
        ]
        for positions in position_sets:
            with self.lock:
                self.current_positions = positions
                self.update_needed = True
            sleep(1)  # Wait for 1 second before moving to the next set
            while self.update_needed:  # Wait until the current set is processed
                sleep(0.1)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.name = ['base_rotation_joint', 'shoulder_joint', 'elbow_joint']

        while not rospy.is_shutdown():
            if self.update_needed:
                with self.lock:
                    joint_state.position = self.current_positions
                    self.update_needed = False  # Reset update flag after applying the new positions
                joint_state.header.stamp = rospy.Time.now()
                self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = JointStatePublisher()
    # Start thread for enqueuing and processing positions
    threading.Thread(target=publisher.enqueue_positions, daemon=True).start()
    # Start publishing joint states
    publisher.publish_joint_states()

