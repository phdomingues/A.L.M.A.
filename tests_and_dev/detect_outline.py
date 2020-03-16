# https://pysource.com/2019/03/12/face-landmarks-detection-opencv-with-python/

import cv2
import numpy as np
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/
import time
import math
import matplotlib.pyplot as plt

def create_mask(h,w,p1,p2):
    # h, w = height and with of the image
    # p1, p2 = top left and bottom right points in (x, y) format

    # generate a image for the mask
    img = np.zeros([h,w,1],dtype=np.uint8)
    # create the mask
    for i in range(p1[0],p2[0]+1):
        for j in range(p1[1],p2[1]+1):
            if (i < 0 or j < 0) and (j > height or i > width):
                print("ERROR")
            else:
                img[j,i] = 255
    return img

def crop_img(img,p1,p2):
    return img[p1[1]-1:p2[1], p1[0]-1:p2[0]]

def denoise(img):
    img2 = img.copy()
    kernel = np.ones((5, 5), np.uint8) # kernel 5x5

    # img2 = cv2.GaussianBlur(img2,(5,5),0)

    # Step 1 - Dilate
    img2 = cv2.dilate(img2, kernel, iterations = 1)
    # Step 2 - Erode
    img2 = cv2.erode(img2, kernel, iterations = 1)
    # Step 3 - Blur
    img2 = cv2.medianBlur(img2, 3)
    return img2

def mark_faces(frame, gray, faces, color, focus = None, maxError = 100):
    faces_pos = []
    patient_pos = []
    mouth_pos = []
    for face in faces:
        landmarks = predictor(gray,face)
        error = 1000

        # face = top left and bottom right points
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()

        if focus:
            error = abs((focus[0][0] - x1)) +  abs((focus[0][1] - y1)) +  abs((focus[1][0] - x2)) +  abs((focus[1][1] - y2))

        if error < maxError:
            color = (0,0,255)
            patient_pos = [((x1,y1),(x2,y2))]

        cv2.rectangle(frame,(x1,y1), (x2,y2), color, 3)

        # mark mouth
        xLeft = math.inf
        xRight = 0
        yUp = math.inf
        yDown = 0

        for point in range(0,68):
            x = landmarks.part(point).x
            y = landmarks.part(point).y

            if point > 47 and point < 69:
                if x < xLeft  : xLeft = x
                if y < yUp    : yUp = y
                if x > xRight : xRight = x
                if y > yDown  : yDown = y

            cv2.circle(frame,(x,y),3,color,-1)

        mouth_pos = ((xLeft,yUp),(xRight,yDown))
        cv2.rectangle(frame,mouth_pos[0],mouth_pos[1],color,3)
        faces_pos.append(((x1,y1),(x2,y2)))

    return (faces_pos, patient_pos, mouth_pos)

def hist_median(array):
    m = int(sum(array)/2)
    idx = 0
    while m > 0:
        m -= array[idx]
        idx += 1
    idx -= 1
    return idx

def calibration(cap, detector, waitTime = 5):
    timeCounter = 0
    start = time.time()
    while True:
        # get data
        _, frame_original = cap.read()
        frame = frame_original.copy()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        height, width, channels = frame.shape
        faces = detector(gray)
        face_pos,_,_ = mark_faces(frame,gray,faces,(0,255,0))
        key = cv2.waitKey(1)
        if key == 27:
            return
        cv2.imshow('calibration', frame)

        # ensures 1 face on the image
        print("Number of faces detected: ", len(faces))
        if len(faces) != 1:
            print (">>> Please position one and only one face on the camera...")
            timeCounter = 0
            start = time.time()
        else:
            timeCounter = int(time.time() - start)
        print(timeCounter)
        if timeCounter == waitTime:
            print("Calibration completed!")
            cropped_img = crop_img(gray,face_pos[0][0],face_pos[0][1])
            cv2.destroyAllWindows()
            return (frame, face_pos, cropped_img)

# auxiliar variables
lostCounter = 0
backup = []
useBackup = False

# setups
cap = cv2.VideoCapture(1) # get the video from 0, (0 = notebook webcam)
detector = dlib.get_frontal_face_detector() # starts the detector
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat") # load the points file

# STEP 1 - CALIBRATION
calibrationImg, patient_face, cropped_patient = calibration(cap, detector)

# plot aux for closing
plt.ion()

tracker = dlib.correlation_tracker()
print(patient_face)
tracker.start_track(calibrationImg, dlib.rectangle(patient_face[0][0][0], patient_face[0][0][1], patient_face[0][1][0], patient_face[0][1][1]))

# STEP 2 - KEEP THE FACE DETECTED
while True:
    # data aquirement
    _, frame_original = cap.read()
    frame = frame_original.copy()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    height, width, channels = frame.shape
    faces = detector(gray)
    masked_img = []

    pos = tracker.get_position()
    print(pos, patient_face)
    tracker.update(frame_original)

    cv2.rectangle(frame_original,(int(pos.left()), int(pos.top())), (int(pos.right()), int(pos.bottom())), (255,0,0), 3)


    # ensures that the patient is present
    if patient_face:

        # mask = create_mask(height, width, patient_face[0][0], patient_face[0][1])
        # masked_img = cv2.bitwise_and(gray,gray,mask = mask)
        # cv2.imshow('mask', masked_img)

        # if the face was lost, we use the backup to try to find it again
        if useBackup:
            _, patient_face, mouth_pos = mark_faces(frame,gray,faces,(0,255,0), focus = backup[0])
        # if it's not the backup beeing used, then the face have been found
        else:
            backup = patient_face
            # patient_face returns in a 1 dimension array, only for compatibility
            _, patient_face, mouth_pos = mark_faces(frame,gray,faces,(0,255,0), focus = patient_face[0])
            lostCounter = 0

        # mouth obstruction control
        if mouth_pos:

            # Crop the mouth only and generate a new image
            mouth = crop_img(frame_original,mouth_pos[0],mouth_pos[1])
            mouth_gray = crop_img(gray,mouth_pos[0],mouth_pos[1])

            ##### TEST REGION #####
            # Histograms of the 3 channels and grayscalle for study
            histR = cv2.calcHist([mouth],[2],None,[256],[0,256])
            histR = [int(i[0]) for i in histR]
            histG = cv2.calcHist([mouth],[1],None,[256],[0,256])
            histG = [int(i[0]) for i in histG]
            histB = cv2.calcHist([mouth],[0],None,[256],[0,256])
            histB = [int(i[0]) for i in histB]
            histGray = cv2.calcHist([mouth_gray],[0],None,[256],[0,256])
            histGray = [int(i[0]) for i in histGray]
            plt.plot(histR, color = 'red', linewidth=1.0)
            plt.plot(histG, color = 'green', linewidth=1.0)
            plt.plot(histB, color = 'blue', linewidth=1.0)
            plt.plot(histGray, color = 'black', linewidth=1.0)
            plt.draw()
            plt.pause(0.0000001)
            plt.clf()

            # statistical analysis
            print("-"*50)
            # np.average(histB,weights = range(256))

            # print("Red   : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histR), hist_median(histR), np.std(histR), np.var(histR)))
            # print("Green : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histG), hist_median(histG), np.std(histG), np.var(histG)))
            # print("Blue  : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histB), hist_median(histB), np.std(histB), np.var(histB)))
            # print("Gray  : max: {:d} \t median: {:.2f}  \t Standard Deviation: {:.2f} \t Variance: {:.2f}".format(max(histGray), hist_median(histGray), np.std(histGray), np.var(histGray)))

            #######################


        else:
            mouth = []

        useBackup = False

    else:
        lostCounter = lostCounter + 1
        patient_face = backup
        useBackup = True

        print("no faces detected... ({})".format(lostCounter))

        # face lost
        if lostCounter == 40:

            massCenter = []
            lostCounter = 0
            cv2.destroyAllWindows()
            print(backup)
            cropped_face = crop_img(gray,backup[0][0],backup[0][1])

            # cropped_patient = calibration face
            # eliminate background noise
            cropped_patient = denoise(cropped_patient)
            height, width, channels = frame.shape
            # find mass center
            M = cv2.moments(cropped_patient)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(cropped_patient, (cX, cY), 3, (255, 255, 255), -1)
            cX = cX/len(cropped_patient[0])
            cY = cY/len(cropped_patient)
            massCenter.append((cX,cY))
            print("@calibration: ({}, {})".format(cX,cY))

            # cropped_face = current face
            # repeats for the other one
            cropped_face = denoise(cropped_face)
            height, width, channels = frame.shape
            M = cv2.moments(cropped_face)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(cropped_face, (cX, cY), 3, (255, 255, 255), -1)
            cX = cX/len(cropped_patient[0])
            cY = cY/len(cropped_patient)
            massCenter.append((cX,cY))
            print("faceLost: ({}, {})".format(cX,cY))
            print(massCenter)

            # make the data usefull for the robo arm
            vX = - massCenter[0][0] + massCenter[1][0]
            vY = - massCenter[0][1] + massCenter[1][1]
            # + vX = go right / - vX = go left
            # + vY = go down  / - vY = go up

            alpha = math.degrees(math.atan(abs(vY)/abs(vX)))
            if vX >= 0 and vY <= 0: data = (alpha)
            elif vX < 0 and vY <= 0: data = (90 + alpha)
            elif vX < 0 and vY > 0:  data = (270 - alpha)
            elif vX >= 0 and vY > 0: data = (360 - alpha)

            print(data)

            # display images and wait for confirmation
            while(cv2.waitKey(1) != 27):
                cv2.imshow("@calibration", cropped_patient)
                cv2.imshow("faceLost", cropped_face)
                # cv2.imshow("last_frame", cropped_face)

            calibrationImg, patient_face, cropped_patient = calibration(cap, detector)



    cv2.imshow("live", frame_original)

    key = cv2.waitKey(1)
    if key == 27:
        calibrationImg, patient_face, cropped_patient = calibration(cap, detector)
        # break
