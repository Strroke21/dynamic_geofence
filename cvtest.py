
import cv2
import numpy as np

cap=cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)

while True:

    ret,frame = cap.read()
    frame_np = np.array(frame)
    gray_frame =cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cv2.imshow("Video",gray_frame)
    cv2.waitKey(1) 