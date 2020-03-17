import numpy as np
import cv2
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(0)

fig = plt.figure()
fig2 = plt.figure()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    (b, g, r) = cv2.split(frame)

    histR = cv2.calcHist([frame],[2],None,[256],[0,256])
    histG = cv2.calcHist([frame],[1],None,[256],[0,256])
    histB = cv2.calcHist([frame],[0],None,[256],[0,256])
    histGray = cv2.calcHist([gray],[0],None,[256],[0,256])

    plt.plot(histR)
    plt.plot(histG)
    plt.plot(histB)
    plt.plot(histG)
    plt.draw()
    plt.pause(0.0001)
    plt.clf()

    # Display the resulting frame
    cv2.imshow('frame',gray)
    cv2.imshow('blue',b)
    cv2.imshow('green',g)
    cv2.imshow('red',r)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
