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
VIDEO = 1

# global variables
detector = dlib.get_frontal_face_detector() # starts the detector
predictor = dlib.shape_predictor("/home/pedro/Documents/TCC/data_files/shape_predictor_68_face_landmarks.dat") # load the points file

def find_angle(calibration, last_found, show=True, gloves_color="light"):
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
    print "calibration center: {}".format(massCenter[0])

    last_found = denoise(last_found)
    M = cv2.moments(last_found)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(last_found, (cX, cY), 3, (255, 255, 255), -1)
    cX = float(cX)/len(last_found[0])
    cY = float(cY)/len(last_found)
    massCenter.append((cX,cY))
    print "last_found: {}".format(massCenter[1])

    # make the data more usefull
    if gloves_color == "light":
        vX = - massCenter[1][0] + massCenter[0][0]
        vY = - massCenter[1][1] + massCenter[0][1]
    else:
        vX = massCenter[1][0] - massCenter[0][0]
        vY = massCenter[1][1] - massCenter[0][1]

    # + vX = go right / - vX = go left
    # + vY = go down  / - vY = go up

    print "vX: {}".format(vX)
    print "vY: {}".format(vY)
    alpha = math.degrees(math.atan(abs(vY)/abs(vX)))
    if vX >= 0 and vY <= 0: angle = (alpha)
    elif vX < 0 and vY <= 0: angle = (180 - alpha)
    elif vX < 0 and vY > 0:  angle = (180 + alpha)
    elif vX >= 0 and vY > 0: angle = (360 - alpha)

    black_img = np.zeros((500,500,3), np.uint8)
    cv2.circle(black_img, (int(massCenter[0][0]*500), int(massCenter[0][1]*500)), 3, (255, 255, 255), -1)
    cv2.circle(black_img, (int(massCenter[1][0]*500), int(massCenter[1][1]*500)), 3, (0, 0, 255), -1)
    cv2.putText(black_img, "White: Calibration / Red: Last frame found", (10,400), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(black_img, "Angle: {}".format(angle), (10,450), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1, cv2.LINE_AA)

    # display images and wait for confirmation
    if show:
        while(cv2.waitKey(1) != 27):
            cv2.imshow("@calibration", calibration)
            cv2.imshow("faceLost", last_found)
            cv2.imshow("Mass Center", black_img)
    return angle

def tracking(data):
    calibration_frame, patient_pos, calibration_cropped_face = data
    calibration_face = crop_img(cv2.cvtColor(calibration_frame,cv2.COLOR_BGR2GRAY),patient_pos[0],patient_pos[1])
    # auxiliar variables
    backup = [] # last calibration_data[1] seen by the algorithm
    lostCounter = 0
    # plot related
    plt.ion()
    # setups
    cap = cv2.VideoCapture(VIDEO) # get the video from 0, (0 = notebook webcam)
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
            last_face_found = crop_img(gray,patient_pos[0],patient_pos[1])
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
    # Init a node called "main_node"
    rospy.init_node('main_node')
    # Instance to publish under "move_angle" a float32
    pub_move_arm = rospy.Publisher('move_angle', Float32, queue_size=10)
# STEP 1 - INITIAL CALIBRATION
    cap = cv2.VideoCapture(VIDEO) # get the video (0 = notebook webcam)
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
    calibration_data = calibration(VIDEO, detector, predictor, waitTime = 5, display = True)

    while True:
# STEP 2 - TRACKING
        # try:
        calib, last = tracking(calibration_data)
        # except Exception as e:
        #     print e
        #     print "Turning off ...."
        #     return
# STEP 3 - FIND ANGLE AND PUBLISH
        pub_move_arm.publish(find_angle(calib, last))
# STEP 4 - ENTER SEEK NEW FACE MODE
        calibration_data = calibration(VIDEO, detector, predictor, waitTime = 1.5, display = True)
        pub_move_arm.publish(-1)
    rospy.spin()

if __name__ == '__main__':
    loop()

