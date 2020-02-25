#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from image_functions import calibration

import cv2
import numpy as np
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/
import time
import math

pub = rospy.Publisher('calibration_done', Bool, queue_size=10)
detector = dlib.get_frontal_face_detector() # starts the detector
predictor = dlib.shape_predictor("/home/pedro/Documents/TCC/data_files/shape_predictor_68_face_landmarks.dat") # load the points file
c = 0

def callback(data):
    # c = only show the calibration process on first atempt (or it will bug)
    global c
    # publish rate (obligatory)
    rate = rospy.Rate(10) # 10hz
    print "INITIALIZING CALIBRATION..."
    # call the actual calibration method
    calibrationImg, patient_face, cropped_patient = calibration(0, detector, predictor, waitTime = data.data, display = not(c))
    # publish true on /calibration_done topic when finished
    pub.publish(True)
    rate.sleep()
    print "Done!"
    c += 1

def loop():
    rospy.init_node('calibration_node')
    rospy.Subscriber("calibration_start", Int16, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    loop()
