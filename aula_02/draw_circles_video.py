#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__      = "Matheus Dib, Fabio de Miranda"


import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

# If you want to open a video, just change this path
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

# Returns an image containing the borders of the image
# sigma is how far from the median we are setting the thresholds
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged



while(True):
    # Capture frame-by-frame
    print("New frame")
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    #blur = gray
    # Detect the edges present in the image
    bordas = auto_canny(blur)


    circles = []


    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)

    # HoughCircles - detects circles using the Hough Method. For an explanation of
    # param1 and param2 please see an explanation here http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    circles = None
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=5,maxRadius=60)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            print(i)
            # draw the outer circle
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)

    # Draw a diagonal blue line with thickness of 5 px
    # cv2.line(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
    cv2.line(bordas_color,(0,0),(511,511),(255,0,0),5)

    # cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
    cv2.rectangle(bordas_color,(384,0),(510,128),(0,255,0),3)

    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(bordas_color,'Ninjutsu ;)',(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    cv2.imshow('Detector de circulos',bordas_color)
    print("No circles were found")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
