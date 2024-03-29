import cv2
import numpy as np
import dlib # https://www.learnopencv.com/install-dlib-on-ubuntu/
import time
import math

# create rectangular mask
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

# crops a section on the image
def crop_img(img,p1,p2):
    return img[p1[1]-1:p2[1], p1[0]-1:p2[0]]

# apply filter to smoth the image
def denoise(img):
    img2 = img.copy()
    img2 = cv2.GaussianBlur(img2,(5,5),0)

    # kernel = np.ones((5, 5), np.uint8) # kernel 5x5
    # # Step 1 - Dilate
    # img2 = cv2.dilate(img2, kernel, iterations = 1)
    # # Step 2 - Erode
    # img2 = cv2.erode(img2, kernel, iterations = 1)
    # # Step 3 - Blur
    # img2 = cv2.medianBlur(img2, 3)
    return img2

def hist_median(array):
    m = int(sum(array)/2)
    idx = 0
    while m > 0:
        m -= array[idx]
        idx += 1
    idx -= 1
    return idx

def mark_faces(frame, predictor, face, color, focus = None, maxError = 100, draw=True):
    # returns:  - face_pos:     returns two points if didn't respect the max error (is not patient)
    #           - patient_pos:  returns two points only if respects the max error (is patient)
    #           - mouth_pos:    returns the mouth pos for the person, patient or not

    face_pos = ()
    patient_pos = ()
    mouth_pos = ()

    # create gray image
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    error = maxError
    # find landmarks on the given face
    landmarks = predictor(gray,face)
    # top left (x1, y1) and bottom right (x2, y2) coordinates
    x1 = face.left()
    y1 = face.top()
    x2 = face.right()
    y2 = face.bottom()
    # if focus on particular face is necessary
    if focus:
        error = abs((focus[0][0] - x1)) +  abs((focus[0][1] - y1)) +  abs((focus[1][0] - x2)) +  abs((focus[1][1] - y2))
    if error < maxError:
        color = (255,0,0)
        patient_pos = ((x1,y1),(x2,y2))
        if draw: cv2.rectangle(frame,(x1,y1), (x2,y2), color, 3)
    else:
        face_pos = ((x1,y1),(x2,y2))
    xLeft = 999999
    xRight = 0
    yUp = 999999
    yDown = 0
    # for al 68 landmarks
    for point in range(0,68):
        # get landmark pos
        x = landmarks.part(point).x
        y = landmarks.part(point).y
        # if it's a mouth landmark
        if point > 47 and point < 69:
            # find most left, right, up and down points
            if x < xLeft  : xLeft = x
            if y < yUp    : yUp = y
            if x > xRight : xRight = x
            if y > yDown  : yDown = y
        # mark landmark on screen
        if draw: cv2.circle(frame,(x,y),3,color,-1)
    # mark a rectangle on the mouth
    mouth_pos = ((xLeft,yUp),(xRight,yDown))
    # cv2.rectangle(frame,mouth_pos[0],mouth_pos[1],color,3)
    return (face_pos, patient_pos, mouth_pos)

def calibration(video, detector, predictor, waitTime = 5, display = True):
    timeCounter = 0
    cap = cv2.VideoCapture(video) # get the video (0 = notebook webcam)
    start = time.time()
    while True:
        # get data
        _, frame_original = cap.read()
        frame = frame_original.copy()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        height, width, channels = frame.shape
        faces = detector(gray)

        # ensures 1 face on the image
        print("Number of faces detected: {}".format(len(faces)))
        if len(faces) != 1:
            print (">>> Please position one and only one face on the camera...")
            timeCounter = 0
            start = time.time()
        else:
            timeCounter = time.time() - start
            for face in faces:
                face_pos,_,mouth_pos = mark_faces(frame,predictor,face,(0,255,0))
        if display:
                cv2.imshow("calibration", frame)
                key = cv2.waitKey(1)
                if key == 27:
                    return

        print("time: {}".format(timeCounter))
        if timeCounter >= waitTime:
            print("Calibration completed!")
            cropped_img = crop_img(gray,face_pos[0],face_pos[1])
            cap.release()
            cv2.destroyAllWindows()
            return (frame_original, face_pos, cropped_img)
