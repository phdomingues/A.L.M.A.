#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import Int16
import cv2

def loop():
    rospy.init_node('main_node')
    pub_calibration = rospy.Publisher('calibration_start', Int16, queue_size=10)
    cap = cv2.VideoCapture(0) # get the video (0 = notebook webcam)
    print "Please prepair for calibration, when ready press esc..."
    while True:
        # get data
        ret, frame = cap.read()
        if ret == True:
            cv2.imshow("camera", frame)
        key = cv2.waitKey(1)
        if key == 27:
            pub_calibration.publish(3)
            break
if __name__ == '__main__':
    loop()
