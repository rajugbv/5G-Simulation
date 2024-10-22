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
        # Initialize with a non-zero position to avoid immediate reversion
        self.current_positions = [0.1, 0.0, 0.0]
        self.lock = threading.Lock()

    def enqueue_positions(self):
        print("Enter 5 sets of positions (base_rotation shoulder elbow), separated by ';':")
        input_sets = input("Format for each set: base_rotation shoulder elbow. Use ';' to separate sets.\n").split(';')
        if len(input_sets) != 5:
            rospy.logwarn("Exactly 5 sets of positions are required. Received: {}".format(len(input_sets)))
            return

        for set_str in input_sets:
            with self.lock:
                try:
                    # Convert each set of positions from string to float and update current_positions
                    self.current_positions = [float(pos) for pos in set_str.split()]
                    if len(self.current_positions) != 3:
                        raise ValueError("Each set must contain exactly 3 values.")
                    rospy.loginfo("Moving to new positions: {}".format(self.current_positions))
                except Exception as e:
                    rospy.logwarn("Invalid input: {}".format(e))
                    return
                # Publish the current set of positions for 1 second before moving to the next set
                for _ in range(10):  # 10 iterations * 0.1 second sleep = 1 second
                    self.publish_current_state()
                    sleep(0.1)

    def publish_current_state(self):
        joint_state = JointState()
        joint_state.name = ['base_rotation_joint', 'shoulder_joint', 'elbow_joint']
        joint_state.header.stamp = rospy.Time.now()
        with self.lock:
            joint_state.position = self.current_positions
        self.pub.publish(joint_state)

    def maintain_position(self):
        # Continuously publish the current position to maintain it
        while not rospy.is_shutdown():
            self.publish_current_state()
            self.rate.sleep()

if __name__ == '__main__':
    publisher = JointStatePublisher()
    # Start thread for maintaining position
    threading.Thread(target=publisher.maintain_position, daemon=True).start()
    # Process positions entered by the user
    publisher.enqueue_positions()

