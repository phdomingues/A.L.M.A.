#!/usr/bin/env python

#Ros Libs
import rospy
from std_msgs.msg import Float32

angle = -1

def loop():

    global angle

    #Publish to the control topic
    pubMain = rospy.Publisher('move_robot', Float32, queue_size=10)

    rate = rospy.Rate(0.5)

    while(True):
        pubMain.publish(angle)
        rate.sleep()

### Callback function 
def Callback(data):
    
    global angle 

    #Log
    rospy.loginfo("I'm receiving the angle [MAIN_CONTROL] = %f" % data.data)

    angle = data.data

    # #Publish to the control topic
    # pubMain = rospy.Publisher('move_robot', Float32, queue_size=10)
    # pubMain.publish(data.data)

### Main Control
def main_control():

    #Init the node control
    rospy.init_node('main_control', anonymous=True)

    #Subscribe to the vision topic
    rospy.Subscriber("/move_angle", Float32, Callback, queue_size=10)

    loop()
    
    rospy.spin()

if __name__ == '__main__':
    main_control()