#video using webcam and communication between pc and arduino using pyserial lib.
import numpy as np
import cv2
cap=cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FPS,72)
while(True):
    ret,frame=cap.read()
    cv2.imshow('ashw',frame)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
    
cap.release()
cv2.destroyAllWindows()
