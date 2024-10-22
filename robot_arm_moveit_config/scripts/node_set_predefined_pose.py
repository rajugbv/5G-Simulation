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
    

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    #Record the start time
    start_time = time.time()
    #Create a new arm object from the MyRobot class
    arm = MyRobot("arm_group")
    hand =  MyRobot("hand")
    start_time_count = 1
    #Here, we will repeat the cycle of setting to various positions, simulating the pick and place action
    while not rospy.is_shutdown():
        #call the function to set the position to "zero_pose"
        if(start_time_count == 1):
        	arm.set_pose("zero_state")
        	T1 = time.time() - start_time
        	print (f"time difference from start to current pose: {T1} seconds")
        	#Wait for 2 seconds
        	rospy.sleep(2)
        
        else:
        	arm.set_pose("zero_state")
        	T1 = time.time()
        	print (f"time difference from start to current pose: {T1} seconds")
        	#Wait for 2 seconds
        	rospy.sleep(2)
        
        
        arm.set_pose("straight_up")
        T2 = time.time() - T1 - 2
        print (f"time difference from previous pose to current pose: {T2} seconds")
        rospy.sleep(2)
        
        #Open the gripper or end effector
        hand.set_pose("hand_open")
        T3 = time.time() - T2 - 2
        print (f"time difference from previous pose to current pose: {T3} seconds")
        rospy.sleep(1)
        
        arm.set_pose("pick_object_pose")
        T4 = time.time() - T3 - 2
        print (f"time difference from previous pose to current pose: {T4} seconds")
        rospy.sleep(2)
        #Close the gripper or end effector
        hand.set_pose("hand_close")
        T5 = time.time() - T4 - 2
        print (f"time difference from previous pose to current pose: {T5} seconds")
        rospy.sleep(1)
        
        arm.set_pose("lift_object_pose")
        T6 = time.time() - T5 - 2
        print (f"time difference from previous pose to current pose: {T6} seconds")
        rospy.sleep(2)
        
        #arm.set_pose("place_object_opposit_pose")
        #rospy.sleep(2)
        
        #Open the gripper or end effector
        hand.set_pose("hand_open")
        T7 = time.time() - T6 -2 
        print (f"time difference from previous pose to current pose: {T7} seconds")
        rospy.sleep(1)
        
        arm.set_pose("opposite_pose")
        T8 = time.time() - T7 - 1
        print (f"time difference from previous pose to current pose: {T8} seconds")
        rospy.sleep(2)
        
        #Close the gripper or end effector
        hand.set_pose("hand_close")
        T9 = time.time() - T8 - 2
        print (f"time difference from previous pose to current pose: {T9} seconds")
        rospy.sleep(1)
        
        if(start_time_count == 1):
        	# Total time difference to finish this current execution 
        	total_time = time.time() - start_time - 1
        	print (f"Time difference to execute the total task: {total_time} seconds")
        else:
        
        	# Total time difference to finish this current execution 
        	total_time = time.time() - T1 - 1
        	print (f"Time difference to execute the total task: {total_time} seconds")
        
        rospy.sleep(5)

    #delete the arm object at the end of code
    del arm
    del hand
	


if __name__ == '__main__':
    main()



