#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from simple_pid import PID
import threading

class RobotArmPIDController:
    def __init__(self):
        rospy.init_node('robot_arm_pid_controller')

        # PID controllers for each joint
        self.pid_shoulder = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_shoulder.output_limits = (-1.57, 1.57)  # Example limits, adjust based on your actuator's capabilities

        # Publishers for each joint command
        self.pub_shoulder = rospy.Publisher('/simple_3d_arm/shoulder_joint_position_controller/command', Float64, queue_size=10)
        
        # Subscriber for the joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Desired positions for each joint, initialized to None
        self.desired_positions = {'shoulder_joint': None}
        self.lock = threading.Lock()

    def joint_state_callback(self, msg):
        with self.lock:
            try:
                # Assuming 'shoulder_joint' is the second joint in the JointState message
                index = msg.name.index('shoulder_joint')
                actual_position = msg.position[index]

                # Update PID controller and publish new command if desired position is set
                if self.desired_positions['shoulder_joint'] is not None:
                    correction = self.pid_shoulder(actual_position)
                    self.pub_shoulder.publish(correction)
            except ValueError:
                pass  # Joint not found in the message

    def update_desired_positions(self):
        while not rospy.is_shutdown():
            try:
                input_str = input("Enter desired shoulder position (radians): ")
                with self.lock:
                    self.desired_positions['shoulder_joint'] = float(input_str)
                    self.pid_shoulder.setpoint = self.desired_positions['shoulder_joint']
            except Exception as e:
                rospy.logwarn("Invalid input: {}".format(e))

if __name__ == '__main__':
    controller = RobotArmPIDController()
    # Start a thread for updating desired positions from user input
    threading.Thread(target=controller.update_desired_positions, daemon=True).start()
    rospy.spin()

