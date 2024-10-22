#!/usr/bin/env python

import rospy
import argparse
from sensor_msgs.msg import JointState

def publish_joint_states(positions):
    rospy.init_node('joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    joint_state = JointState()
    joint_state.name = ['shoulder_joint', 'elbow_joint']
    joint_state.position = positions

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publish joint state positions.')
    parser.add_argument('shoulder', type=float, help='Position of the shoulder joint')
    parser.add_argument('elbow', type=float, help='Position of the elbow joint')
    args = parser.parse_args()

    positions = [args.shoulder, args.elbow]
    try:
        publish_joint_states(positions)
    except rospy.ROSInterruptException:
        pass

