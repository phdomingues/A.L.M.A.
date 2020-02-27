#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import Float32

# image related
import cv2
import numpy as np
import matplotlib.pyplot as plt
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/

# custom libraries
from image_functions import *

MAXCOUNT = 40

# global variables
detector = dlib.get_frontal_face_detector() # starts the detector
predictor = dlib.shape_predictor("/home/pedro/Documents/TCC/data_files/shape_predictor_68_face_landmarks.dat") # load the points file

def find_angle(calibration, last_found):
    massCenter = []

    calibration = denoise(calibration)
    calib_height, calib_width = calibration.shape
    M = cv2.moments(calibration)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(calibration, (cX, cY), 3, (255, 255, 255), -1)
    cX = float(cX)/len(calibration[0])
    cY = float(cY)/len(calibration)
    massCenter.append((cX,cY))

    last_found = denoise(last_found)
    lf_height, lf_width = last_found.shape
    M = cv2.moments(last_found)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(last_found, (cX, cY), 3, (255, 255, 255), -1)
    cX = float(cX)/len(last_found[0])
    cY = float(cY)/len(last_found)
    massCenter.append((cX,cY))

    # make the data more usefull
    vX = massCenter[1][0] - massCenter[0][0]
    vY = massCenter[1][1] - massCenter[0][1]
    # + vX = go right / - vX = go left
    # + vY = go down  / - vY = go up

    alpha = math.degrees(math.atan(abs(vY)/abs(vX)))
    if vX >= 0 and vY <= 0: angle = (alpha)
    elif vX < 0 and vY <= 0: angle = (180 - alpha)
    elif vX < 0 and vY > 0:  angle = (180 + alpha)
    elif vX >= 0 and vY > 0: angle = (360 - alpha)

    return angle

def tracking(data):
    calibration_frame, patient_pos, calibration_cropped_face = data
    calibration_face = crop_img(cv2.cvtColor(calibration_frame,cv2.COLOR_BGR2GRAY),patient_pos[0],patient_pos[1])
    # auxiliar variables
    backup = [] # last patient_face seen by the algorithm
    lostCounter = 0
    # plot related
    plt.ion()
    # setups
    cap = cv2.VideoCapture(0) # get the video from 0, (0 = notebook webcam)
    # track loop
    while True:
        # data aquirement
        _, frame = cap.read()
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        height, width, channels = frame.shape
        faces = detector(gray)
        present = False # patient present on the current frame

        for face in faces:
            others, patient, mouth = mark_faces(frame_copy, predictor, face, (0,255,0), focus = patient_pos)
            if patient:
                patient_pos = patient
                mouth_pos = mouth
                present = True
                lostCounter = 0
                backup = gray
                break # don't even bother looking to the other faces

        # if patient not in frame
        if not present:
            lostCounter += 1
            print lostCounter

        if lostCounter == MAXCOUNT:
            last_face_found = crop_img(backup,patient_pos[0],patient_pos[1])
            cap.release()
            cv2.destroyAllWindows()
            return (calibration_face, last_face_found)

        cv2.imshow("live", frame_copy)
        key = cv2.waitKey(1)
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            break
    return

def loop():
    rospy.init_node('main_node')
    pub_move_arm = rospy.Publisher('move_angle', Float32, queue_size=10)
    # STEP 1 - INITIAL CALIBRATION
    cap = cv2.VideoCapture(0) # get the video (0 = notebook webcam)
    rospy.loginfo("Please prepair for calibration, when ready press esc...")
    while True:
        # get data
        ret, frame = cap.read()
        if ret == True:
            cv2.imshow("camera", frame)
        key = cv2.waitKey(1)
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            break
    starting_values = calibration(0, detector, predictor, waitTime = 5, display = True)

    # STEP 2 - TRACKING
    calib, last = tracking(starting_values)
    # STEP 3 - FIND ANGLE AND PUBLISH
    pub_move_arm.publish(find_angle(calib, last))
    # STEP 4 - ENTER SEEK NEW FACE MODE
    # .... DEVELOP STAGE
    rospy.spin()

if __name__ == '__main__':
    loop()
