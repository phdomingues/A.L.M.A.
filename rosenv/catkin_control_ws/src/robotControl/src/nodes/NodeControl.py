#!/usr/bin/env python

import math
import rospy
import geometry_msgs
import moveit_commander
from Orientation import Orientation

def CalculateOrientation(angle):

    orient = Orientation()

    #calcular orientacao

    GoToPose(orient)


def GoToPose(object_orientation):
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    object_orientation = Orientation()

    #Setting the goal position
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = object_orientation.GetPositionX()
    pose_goal.position.y = object_orientation.GetPositionY()
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    #Plan the movement
    move_group.plan

    #Execute the moviment
    move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    
    #Clear the targets 
    move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose

    return current_pose