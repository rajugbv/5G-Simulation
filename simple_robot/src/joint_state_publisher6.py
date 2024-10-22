#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from simple_robot.srv import UpdatePositions, UpdatePositionsResponse

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # Adjust the rate as needed
        self.current_positions = [0.0, 0.0]  # Initial positions

    def handle_update_positions(self, req):
        self.current_positions = [req.shoulder_position, req.elbow_position]
        return UpdatePositionsResponse(True)

    def publish_joint_states(self):
        service = rospy.Service('update_positions', UpdatePositions, self.handle_update_positions)
        rospy.loginfo("Service server has been started.")

        joint_state = JointState()
        joint_state.name = ['shoulder_joint', 'elbow_joint']

        while not rospy.is_shutdown():
            joint_state.position = self.current_positions
            joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = JointStatePublisher()
    publisher.publish_joint_states()
