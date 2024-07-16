
import cv2
import numpy as np

path='/home/deathstroke/Downloads/car.jpg'

img = cv2.imread(path)
img = cv2.resize(img,(640,480))
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# Define lower and upper bounds for pink color in HSV
lower_pink = np.array([140, 50, 50])
upper_pink = np.array([180, 255, 255])
# Create a mask to select pixels within the pink color range
mask = cv2.inRange(hsv, lower_pink, upper_pink)
# Bitwise AND operation to mask the original image
masked_img = cv2.bitwise_and(img, img, mask=mask)

cv2.imshow('Image',masked_img)
cv2.waitKey(0)
