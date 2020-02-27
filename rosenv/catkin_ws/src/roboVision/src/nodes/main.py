#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import Int16
# creating a msg type in ros is too dificult, so for now quaternion will improvise as the data type
from geometry_msgs.msg import Quaternion # x=x1, y=y1, z=x2, w=y2 position of the face (https://github.com/ros/common_msgs/blob/jade-devel/geometry_msgs/msg/Quaternion.msg)

# image related
import cv2
import numpy as np
import matplotlib.pyplot as plt
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/

# custom libraries
from image_functions import mark_faces
from image_functions import crop_img
import time
def initial_calibration():
    pub_calibration = rospy.Publisher('calibration_start', Int16, queue_size=10)
    cap = cv2.VideoCapture(0) # get the video (0 = notebook webcam)
    rospy.loginfo("Please prepair for calibration, when ready press esc...")
    time.sleep(2)
    pub_calibration.publish(5)
    # while True:
    #     # get data
    #     ret, frame = cap.read()
    #     if ret == True:
    #         cv2.imshow("camera", frame)
    #     key = cv2.waitKey(1)
    #     if key == 27:
    #         cap.release()
    #         cv2.destroyAllWindows()
    #         pub_calibration.publish(5)
    #         break

def tracking(data):
    patient_face = ((int(data.x),int(data.y)),(int(data.z),int(data.w)))
    detector = dlib.get_frontal_face_detector() # starts the detector
    predictor = dlib.shape_predictor("/home/pedro/Documents/TCC/data_files/shape_predictor_68_face_landmarks.dat") # load the points file
    # auxiliar variables
    backup = [] # last patient_face seen by the algorithm
    lostCounter = 0
    useBackup = False
    detected = True
    # plot related
    plt.ion()
    # setups
    cap = cv2.VideoCapture(0) # get the video from 0, (0 = notebook webcam)
    # track loop
    while detected:
        # data aquirement
        _, frame = cap.read()
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        height, width, channels = frame.shape
        faces = detector(gray)
        # ensures that the patient is present
        if patient_face:
            # if the face was lost, we use the backup
            print patient_face
            if useBackup:
                print "using backup"
                for face in faces:
                    (_, patient_face, mouth_pos) = mark_faces(frame_copy, predictor, face, (0,255,0), focus = backup)
            # if it's not the backup beeing used, then the face have been found
            else:
                backup = patient_face
                # patient_face returns in a 1 dimension array, only for compatibility
                for face in faces:
                    (_, patient_face, mouth_pos) = mark_faces(frame_copy, predictor, face, (0,255,0), focus = patient_face)
                lostCounter = 0

##################################################################################################################################################
            if mouth_pos:
                # Crop the mouth only and generate a new image
                mouth = crop_img(frame,mouth_pos[0],mouth_pos[1])

            useBackup = False

        else:
            lostCounter = lostCounter + 1
            patient_face = backup
            useBackup = True
            print "no faces detected... ({})".format(lostCounter)

        # # face lost
        # if lostCounter == 40:
        #
        #     massCenter = []
        #     lostCounter = 0
        #     cv2.destroyAllWindows()
        #     print(backup)
        #     cropped_face = crop_img(gray,backup[0][0],backup[0][1])
        #
        #     # cropped_patient = calibration face
        #     # eliminate background noise
        #     cropped_patient = denoise(cropped_patient)
        #
        #     height, width, channels = frame.shape
        #     # find mass center
        #     M = cv2.moments(cropped_patient)
        #     # calculate x,y coordinate of center
        #
        #
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])
        #     cv2.circle(cropped_patient, (cX, cY), 3, (255, 255, 255), -1)
        #     cX = float(cX)/len(cropped_patient[0])
        #     cY = float(cY)/len(cropped_patient)
        #     massCenter.append((cX,cY))
        #     print massCenter
        #     print("@calibration: ({}, {})".format(cX,cY))
        #
        #     # cropped_face = current face
        #     # repeats for the other one
        #     cropped_face = denoise(cropped_face)
        #     height, width, channels = frame.shape
        #     M = cv2.moments(cropped_face)
        #     # calculate x,y coordinate of center
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])
        #     cv2.circle(cropped_face, (cX, cY), 3, (255, 255, 255), -1)
        #     cX = float(cX)/len(cropped_patient[0])
        #     cY = float(cY)/len(cropped_patient)
        #     massCenter.append((cX,cY))
        #     print("faceLost: ({}, {})".format(cX,cY))
        #     print(massCenter)
        #
        #     # make the data usefull for the robo arm
        #     vX = - massCenter[0][0] + massCenter[1][0]
        #     vY = - massCenter[0][1] + massCenter[1][1]
        #     # + vX = go right / - vX = go left
        #     # + vY = go down  / - vY = go up
        #
        #     alpha = math.degrees(math.atan(abs(vY)/abs(vX)))
        #     if vX >= 0 and vY <= 0: data = (alpha)
        #     elif vX < 0 and vY <= 0: data = (180 - alpha)
        #     elif vX < 0 and vY > 0:  data = (180 + alpha)
        #     elif vX >= 0 and vY > 0: data = (360 - alpha)
        #
        #     print(data)
        #
        #     # display images and wait for confirmation
        #     while(cv2.waitKey(1) != 27):
        #         cv2.imshow("@calibration", cropped_patient)
        #         cv2.imshow("faceLost", cropped_face)
        #         # cv2.imshow("last_frame", cropped_face)
        #
        #     calibrationImg, patient_face, cropped_patient = calibration(cap, detector)
        cv2.imshow("live", frame_copy)
        key = cv2.waitKey(1)
        if key == 27:
            # calibrationImg, patient_face, cropped_patient = calibration(cap, detector)
            break

##################################################################################################################################################
    return

def loop():
    rospy.init_node('main_node')
    rospy.Subscriber("calibration_done", Quaternion, tracking)
    # STEP 1 - INITIAL CALIBRATION
    initial_calibration()
    rospy.spin()

if __name__ == '__main__':
    loop()
