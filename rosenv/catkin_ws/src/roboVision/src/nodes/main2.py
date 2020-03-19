#!/usr/bin/env python3

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3

# image related
import cv2
import tensorflow as tf
from sklearn.svm import SVC
import numpy as np
import matplotlib.pyplot as plt
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

# ros
import rospy
from std_msgs.msg import Float32

# file managing
import csv
import os
import rospkg
import pickle

# custom libraries
from image_functions import *

MAXCOUNT = 40       # Maximum frames before considering the face lost
VIDEO = 1           # Video that will be captured (0 = webcam / 1 = external webcam)
COLECT_DATA = False  # Toggle the plot graphing for statistical data colection (TURNING THIS ON WILL MESS ALL THE PREVIOUS COLECTED DATA)
SHOW = True
LOG = False

# current folder
rospack = rospkg.RosPack()
package_path = rospack.get_path('roboVision')
# get abs path for csv file
path = os.path.join(package_path,"../../../../visao/data_files/collected_data/all_statistics.csv")

# global variables
detector = dlib.get_frontal_face_detector() # starts the detector
predictor_path = os.path.join(package_path,"../../../../visao/data_files/shape_predictor_68_face_landmarks.dat")
predictor = dlib.shape_predictor(predictor_path) # load the points file

# loading SVM
svm_path = os.path.join(package_path,"../../../../visao/data_files/gausian_svm.pkl")
file = open(svm_path, 'rb')
svm_classifier = pickle.load(file)

# loading tf model
model = tf.keras.models.load_model(os.path.join(package_path, "../../../../visao/tests_and_dev/model.h5"))

def find_angle(calibration, last_found, show=SHOW, log=LOG, gloves_color="light"):
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
    if log: print("calibration center: {}".format(massCenter[0]))

    last_found = denoise(last_found)
    M = cv2.moments(last_found)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(last_found, (cX, cY), 3, (255, 255, 255), -1)
    cX = float(cX)/len(last_found[0])
    cY = float(cY)/len(last_found)
    massCenter.append((cX,cY))
    if log: print("last_found: {}".format(massCenter[1]))

    # make the data more usefull
    if gloves_color == "light":
        pt1 = (int(massCenter[1][0]*500), int(massCenter[1][1]*500))
        pt2 = (int(massCenter[0][0]*500), int(massCenter[0][1]*500))
        vX = - massCenter[1][0] + massCenter[0][0]
        vY = - massCenter[1][1] + massCenter[0][1]
    else:
        pt1 = (int(massCenter[0][0]*500), int(massCenter[0][1]*500))
        pt2 = (int(massCenter[1][0]*500), int(massCenter[1][1]*500))
        vX = massCenter[1][0] - massCenter[0][0]
        vY = massCenter[1][1] - massCenter[0][1]

    # + vX = go right / - vX = go left
    # + vY = go down  / - vY = go up

    if log:
        print("vX: {}".format(vX))
        print("vY: {}".format(vY))
    alpha = math.degrees(math.atan(abs(vY)/abs(vX)))
    if vX >= 0 and vY <= 0: angle = (alpha)
    elif vX < 0 and vY <= 0: angle = (180 - alpha)
    elif vX < 0 and vY > 0:  angle = (180 + alpha)
    elif vX >= 0 and vY > 0: angle = (360 - alpha)

    # display images and wait for confirmation
    if show:
        black_img = np.zeros((500,500,3), np.uint8)
        cv2.arrowedLine(black_img, (0, pt1[1]), (500, pt1[1]), (255,255,255), 2, 8, 0, 0.1)
        cv2.arrowedLine(black_img, (pt1[0], 500), (pt1[0], 0), (255,255,255), 2, 8, 0, 0.1)
        cv2.arrowedLine( black_img, pt1, pt2, (255,0,0), 2, 8, 0, 0.3)
        cv2.circle(black_img, (int(massCenter[0][0]*500), int(massCenter[0][1]*500)), 3, (0, 255, 0), -1)
        cv2.circle(black_img, (int(massCenter[1][0]*500), int(massCenter[1][1]*500)), 3, (0, 0, 255), -1)
        cv2.putText(black_img, "green: Calibration / Red: Last frame found", (10,400), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(black_img, "Angle: {} degrees".format(angle), (10,450), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1, cv2.LINE_AA)
        while(cv2.waitKey(1) != 27):
            cv2.imshow("@calibration", calibration)
            cv2.imshow("faceLost", last_found)
            cv2.imshow("Mass Center", black_img)
    return angle

def hist_plot(frame_mouth, frame_mouth_gray, show=SHOW, log=LOG):
    plt.ion()
    # Histograms creation and manipulation
    histR = cv2.calcHist([frame_mouth],[2],None,[256],[0,256])
    histR = [int(i[0]) for i in histR]
    # histR[histR.index(max(histR))] = 0
    histG = cv2.calcHist([frame_mouth],[1],None,[256],[0,256])
    histG = [int(i[0]) for i in histG]
    # histG[histG.index(max(histG))] = 0
    histB = cv2.calcHist([frame_mouth],[0],None,[256],[0,256])
    histB = [int(i[0]) for i in histB]
    # histB[histB.index(max(histB))] = 0
    histGray = cv2.calcHist([frame_mouth_gray],[0],None,[256],[0,256])
    histGray = [int(i[0]) for i in histGray]
    histGray[histGray.index(max(histGray))] = 0

    # plot
    if show:
        plt.plot(histR, color = 'red', linewidth=1.0)
        plt.plot(histG, color = 'green', linewidth=1.0)
        plt.plot(histB, color = 'blue', linewidth=1.0)
        plt.plot(histGray, color = 'black', linewidth=1.0)
        plt.draw()
        plt.pause(0.0000001)
        plt.clf()

    # statistical analysis
    median  = (hist_median(histR), hist_median(histG), hist_median(histB), hist_median(histGray))
    var     = (np.var(histR), np.var(histG), np.var(histB), np.var(histGray))
    std_dev = (np.std(histR), np.std(histG), np.std(histB), np.std(histGray))
    peak    = (max(histR), max(histG), max(histB), max(histGray))
    statistics = (median, var, std_dev, peak)
    if log:
        # print data
        print("-"*50)
        print("Red   : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histR), hist_median(histR), np.std(histR), np.var(histR)))
        print("Green : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histG), hist_median(histG), np.std(histG), np.var(histG)))
        print("Blue  : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histB), hist_median(histB), np.std(histB), np.var(histB)))
        print("Gray  : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histGray), hist_median(histGray), np.std(histGray), np.var(histGray)))
    return statistics

def tracking(data):
    calibration_frame, patient_pos, calibration_cropped_face = data
    calibration_face = crop_img(cv2.cvtColor(calibration_frame,cv2.COLOR_BGR2GRAY),patient_pos[0],patient_pos[1])
    # auxiliar variables
    backup = [] # last calibration_data[1] seen by the algorithm
    lostCounter = 0
    # setups
    cap = cv2.VideoCapture(VIDEO) # get the video from 0, (0 = notebook webcam)

    # track loop
    while True:
        # data aquirement
        _, frame = cap.read()
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
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
                # Crop the mouth only and generate a new image
                frame_mouth = crop_img(frame,mouth_pos[0],mouth_pos[1])
                frame_mouth_gray = crop_img(gray,mouth_pos[0],mouth_pos[1])
                statistics = hist_plot(frame_mouth,frame_mouth_gray)
                # SVM mouth classifier
                # result = svm_classifier.predict([[item[i] for item in statistics for i in range(len(item))]])
                # if result[0] == 0: print("obstructed")
                # else: print("free")

                dados = np.array([[int(item[i])] for item in statistics for i in range(len(item))])
                dados = dados.transpose()
                result = model.predict(dados)
                if result == 0: print("free")
                else: print("obstructed")

                if COLECT_DATA:
                    with open(path,'ab') as out:
                        csv_out = csv.writer(out)
                        write_data = [statistics[i][j] for i in range(4) for j in range(4)]
                        csv_out.writerow(write_data)
                        out.close()
                break # don't even bother looking to the other faces

        # if patient not in frame
        if not present:
            lostCounter += 1
            print(lostCounter)

        if lostCounter == MAXCOUNT:
            last_face_found = crop_img(gray,patient_pos[0],patient_pos[1])
            cap.release()
            cv2.destroyAllWindows()
            return (calibration_face, last_face_found)

        cv2.imshow("copy", frame_copy)
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
        try:
            calib, last = tracking(calibration_data)
        except Exception as e:
            print (e)
            print ("Turning off ....")
            return
# STEP 3 - FIND ANGLE AND PUBLISH
        pub_move_arm.publish(find_angle(calib, last))
# STEP 4 - ENTER SEEK NEW FACE MODE
        calibration_data = calibration(VIDEO, detector, predictor, waitTime = 1.5, display = True)
        pub_move_arm.publish(-1)
    rospy.spin()

if __name__ == '__main__':
    loop()
    file.close()

