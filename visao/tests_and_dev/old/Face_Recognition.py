import cv2
import numpy

face_cascade = cv2.CascadeClassifier('./xml/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('./xml/haarcascade_eye.xml')
mouth_cascade = cv2.CascadeClassifier('./xml/Mouth.xml')

# img = cv2.imread('./images/face_01.jpeg')
# img2 = img.copy()
# img3 = img.copy()

# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#cv2.imshow('image',img)
#cv2.waitKey(0)

cap = cv2.VideoCapture(0)

while True:
    # capture the frames
    ret, frame = cap.read()

    # convert image to gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.01, minNeighbors=100) # (imagem, downscale/pass{scalePass}, minNeighbors)
    mouth = mouth_cascade.detectMultiScale(gray, 1.3, 5)
    eyes = eye_cascade.detectMultiScale(gray, 1.01, 100)

    for (x, y, w, h) in mouth: # x, y sao um ponto do retangulo, w, h sao largura e altura do retangulo
        #y = int(y-0.15*h) # transferindo o ponto para o cando inferior esquerdo
        cv2.rectangle(gray, (y, x), (x + w, y + h), (255, 0, 0), 3)

    # for (x, y, w, h) in faces: # x, y sao um ponto do retangulo, w, h sao largura e altura do retangulo
    #     y = y+h # transferindo o ponto para o cando inferior esquerdo
    #     cv2.rectangle(gray, (x, y), (x+w, int(y-(h/3))), (0, 0, 255), 3)
    #     cv2.rectangle(gray, (x, y), (x + w, y - h), (255, 0, 0), 3)
    #
    # for (x, y, w, h) in eyes: # x, y sao um ponto do retangulo, w, h sao largura e altura do retangulo
    #     y = y+h # transferindo o ponto para o cando inferior esquerdo
    #     cv2.rectangle(gray, (x, y), (x+w, y-h), (255, 0, 0), 3)

    cv2.imshow('boca', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
