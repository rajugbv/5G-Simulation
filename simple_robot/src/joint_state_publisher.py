#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def publish_joint_states(positions):
    rospy.init_node('joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz

    joint_state = JointState()
    joint_state.name = ['shoulder_joint', 'elbow_joint']
    joint_state.position = positions

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Initial positions, e.g., [0.5, 1.2]
        positions = [0.5, 0.0]  
        publish_joint_states(positions)
    except rospy.ROSInterruptException:
        pass

