#! /usr/bin/env python

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import time
from math import pi
import numpy as np
import zmq
import os

#HOST = "10.204.77.105"
#PORT = 8888

# ZMQ setup
context = zmq.Context()
footage_socket = context.socket(zmq.SUB)
footage_socket.bind("tcp://*:9991")
#footage_socket.bind("tcp://10.45.0.1:9991")
footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):

        #Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        #define the movegoup for the robotic 
        #Replace this value with your robots planning group name that you had set in Movit Setup Assistant
        self._planning_group = Group_Name
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        #We create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Create action client for the Execute Trajectory action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')
        
        
        # ZMQ setup
        #self.context = zmq.Context()
        #self.footage_socket = self.context.socket(zmq.SUB)
        #self.footage_socket.bind("tcp://*:5556")  
        #self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))


    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        

        #Set the predefined position as the named joint configuration as the goal to plan for the move group. The predefined positions are defined in the Moveit Setup Assistant
        self._group.set_named_target(arg_pose_name)
        #Plan to the desired joint-space goal using the default planner
        plan_success, plan, planning_time, error_code = self._group.plan()

        #Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #Update the trajectory in the goal message
        goal.trajectory = plan
        #Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        #Print the current pose
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        
    #ZMQ receiving method
    def receive_pose(self):
        """Receive pose from ZMQ and return it."""
        print("Receive pose from ZMQ and return it.")
        try:
            #frame_text = footage_socket.recv_string()  # Non-blocking receive
            frame1 = footage_socket.recv()  # Non-blocking receive
            print(frame1)
            frame_text = str(frame1)
            list_recv = frame_text.split("Helal")
            frame = list_recv[0]
            #time parameters
            time_received = list_recv[1]
            print("command fired time from remote computer: "+ time_received)
            print("time received at t2: "+ str(time.time()))
            frame2=frame[2:]
            return frame2,float(time_received)
        except zmq.Again as e:
            print("Waiting for pose...")
            return None  # Return None if no message is received

    

    # Class Destructor
    def __del__(self):
        self.context.destroy()  # Properly close the ZMQ context
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    #Record the start time
    
    #Create a new arm object from the MyRobot class
    arm = MyRobot("arm_group")
    hand =  MyRobot("hand")
    start_time_count = 1
    #Here, we will repeat the cycle of setting to various positions, simulating the pick and place action
    #while not rospy.is_shutdown():
    while True:
        
        frame,time_r = arm.receive_pose()  # Use arm's method to receive pose, could be separate if hand needs its own
        if frame:
            start_time_Ac = time.time()
            print ("Actuation start time: "+ str(time.time()))
            arm.set_pose(frame)
            end_time = time.time()
            #finaltime = time_r - end_time
            #actuation_time = start_time_Ac - end_time
            
            print ("Actuation done(t3): " +str(time.time()))
            
            #print ("time for ",finaltime)
            #print("time for command fire to robot execute",finaltime)
            # If hand commands are also expected to be received, add handling here
            
        else:
            rospy.sleep(1)  # Wait a bit before trying to receive again

        print("Go and wait again")
    
    #delete the arm object at the end of code
    del arm
    del hand
	


if __name__ == '__main__':
    main()



