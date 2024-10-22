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
        # A list to hold multiple sets of positions
        self.positions_queue = []
        self.current_positions = [0.0, 0.0, 0.0]  # Default positions
        self.lock = threading.Lock()

    def enqueue_positions(self):
        while not rospy.is_shutdown():
            try:
                # User input for new positions: base rotation, shoulder, elbow for 5 sets
                print("Enter 5 sets of positions (base_rotation shoulder elbow), separated by ';':")
                input_sets = input("Format: set1;set2;set3;set4;set5\n").split(';')
                if len(input_sets) != 5:
                    raise ValueError("Exactly 5 sets of positions are required.")
                
                with self.lock:
                    self.positions_queue = []  # Reset the queue
                    for set in input_sets:
                        positions = [float(pos) for pos in set.split()]
                        if len(positions) != 3:
                            raise ValueError("Each set must contain exactly 3 values.")
                        self.positions_queue.append(positions)
            except Exception as e:
                rospy.logwarn("Invalid input: {}".format(e))

    def process_queue(self):
        while not rospy.is_shutdown():
            if self.positions_queue:
                with self.lock:
                    self.current_positions = self.positions_queue.pop(0)
                sleep(1)  # Wait for 1 second before moving to the next set

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.name = ['base_rotation_joint', 'shoulder_joint', 'elbow_joint']

        while not rospy.is_shutdown():
            with self.lock:
                joint_state.position = self.current_positions
            joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = JointStatePublisher()
    # Start thread for enqueuing positions
    threading.Thread(target=publisher.enqueue_positions, daemon=True).start()
    # Start thread for processing the queue
    threading.Thread(target=publisher.process_queue, daemon=True).start()
    # Start publishing joint states
    publisher.publish_joint_states()

