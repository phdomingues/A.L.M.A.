#!/usr/bin/env python

#Ros Libs
import rospy
from std_msgs.msg import Float32


def Callback(data):
    rospy.loginfo("I'm receiving the angle = %f" % data.data)
    
    #CalculateOrientation(data.data)

def MainControl():
    #Init the node control
    rospy.init_node('main_control')

    #Subscribe to the vision topic
    rospy.Subscriber("/move_angle", Float32, Callback, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    MainControl()