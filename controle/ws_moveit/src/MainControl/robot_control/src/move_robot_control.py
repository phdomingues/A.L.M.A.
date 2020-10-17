#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import cos
from math import sin
from math import tan
from math import pi 
from math import atan
from math import sqrt
from math import asin

#from std_msgs.msg import String
from std_msgs.msg import Float32

from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import tf.transformations 

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True
  
class MoveRobotControl(object):
  def __init__(self):
    #super(MoveGroupPythonIntefacerobotControl, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_robotControl', anonymous=True)
    rospy.init_node('move_robot_control', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this robotControl the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print "============ Planning frame: %s" % planning_frame
    
    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
 

  def go_to_joint_state(self):

    move_group = self.move_group

    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):

    move_group = self.move_group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.7                                           
    pose_goal.position.y = 0.0
    pose_goal.position.z = sqrt( (0.5**2) - (pose_goal.position.y**2) - ((pose_goal.position.x-1)**2) ) 

    alpha_goal = asin(pose_goal.position.z/0.5)    

    q = quaternion_from_euler(0, (alpha_goal + pi/2), 0)
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, angle):

    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    
    waypoints = []
    current_pose = move_group.get_current_pose().pose #Define object current pose
    goal_pose = move_group.get_current_pose().pose #Define object goal pose

    angle = angle*(pi)/180 #Define the angle in rad	   
    
    goal_pose.position.y = current_pose.position.y + (cos(angle)/20)*(-1)  # sideways (y)
    goal_pose.position.z = current_pose.position.z + sin(angle)/20 # sideways (z)
    goal_pose
    goal_pose.position.x = (sqrt( (0.5**2) - (goal_pose.position.y**2) - (goal_pose.position.z**2) ))*(-1)  + 1
    
    theta_goal = asin( goal_pose.position.y / 0.5) 
    theta_current = asin( current_pose.position.y / 0.5) 
    
    theta_rotation = theta_goal - theta_current

    
    alpha_current = asin(current_pose.position.z/0.5)
    alpha_goal = asin(goal_pose.position.z/0.5)
  
    alpha_rotation = (alpha_goal - alpha_current) 
      

    q_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w] 
    

    q_rotate = tf.transformations.quaternion_from_euler(theta_rotation, alpha_rotation, 0)

    q_new = tf.transformations.quaternion_multiply(q_current, q_rotate)  

    goal_pose.orientation.w = q_new[3] 
    goal_pose.orientation.z = q_new[2]
    goal_pose.orientation.y = q_new[1]
    goal_pose.orientation.x = q_new[0]

    waypoints.append(copy.deepcopy(goal_pose))
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
  
  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web robotControls more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=False)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

## Callback function 
def Callback(data):

    if data != None:
       ControlManager(data.data)

#
## Method To Manage the Control
#
def ControlManager(angle):

    global robotControl

    #Log
    rospy.loginfo("I'm receiving the angle [MOVE_ROBOT_CONTROL] = %f" % angle)

    
    #Control the robot
    if (angle >= 0):
       
        print "CONTROLING ROBOT..." 

        try:
          cartesian_plan, fraction = robotControl.plan_cartesian_path(angle)
          robotControl.execute_plan(cartesian_plan)
        
        except:
          print "WARNING: LIMIT OF WORKSPACE:" 

          angle = angle + 180

          cartesian_plan, fraction = robotControl.plan_cartesian_path(angle)
          robotControl.execute_plan(cartesian_plan)

        rospy.sleep(3)

robotControl = MoveRobotControl()
#
## Main Method
#
def main():
  try:
    
    print "INIT_MOVE_ROBOT_CONTROL"

    #Instance a Move Robot Control
    global robotControl    

    #Subscribe to the Control topic
    rospy.Subscriber("/move_robot", Float32, Callback, queue_size=10)

    #Initialize the robot
    robotControl.go_to_joint_state() #go to initial state
    
    robotControl.go_to_pose_goal()   #go to desired starting pose

    rospy.spin()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
