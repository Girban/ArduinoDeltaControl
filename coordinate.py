## program for object location

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import requests
import imutils
import cv2
import math as m
def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
# load the image, convert it to grayscale, and blur it slightly
def coord(clr,url1):
    img_resp1 = requests.get(url1)
    img_arr1 = np.array(bytearray(img_resp1.content), dtype=np.uint8)
    image=cv2.imdecode(img_arr1,-1)
    image=cv2.resize(image,(500,400),interpolation=cv2.INTER_AREA)
    h,w,_=image.shape;
    h=int(h)
    h_h=int(h/2)
    w=int(w)
    w_h=int(w/2)
    #L=np.array([160,140,100],np.uint8)
#H=np.array([179,255,255],np.uint8)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    if (clr == "r"):
        lower_red = np.array([0, 100, 50])
        upper_red = np.array([2, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # Range for upper range
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        # Generating the final mask to detect red color
        mask = mask1 + mask2
    elif (clr == "b"):
        L = np.array([110, 50, 50])
        H = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, L, H)
    elif (clr == "O"):
        L = np.array([10, 150, 50])
        H = np.array([20, 255, 255])
        mask = cv2.inRange(hsv, L, H)
    elif (clr == "g"):
        L = np.array([42, 52, 72], np.uint8)
        H = np.array([90, 255, 255], np.uint8)
        mask = cv2.inRange(hsv, L, H)
    kernal = np.ones((5,5), "uint8")
    mask=cv2.dilate(mask,kernal,iterations=4)
# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
#edged = cv2.dilate(edge, None, iterations=1)
#edged = cv2.erode(edged, None, iterations=1)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    contours=cnts;
    for i in range(len(contours)):
        max_area=cv2.contourArea(contours[i])
        for j in range(i,len(contours)):
            area=cv2.contourArea(contours[j])
            if(max_area<area):
                 max_area=area;
                 cnt=contours[i];
                 contours[i]=contours[j]
                 contours[j]=cnt
    if (len(contours) == 0):
        return 0,0
    elif (cv2.contourArea(contours[0]) > 2000):
        c = contours[0]
        orig = image.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        M = cv2.moments(c)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        dx = dist.euclidean((w_h, h_h), (cx, h_h))
        if (w_h < cx):
            dx = -dx
        dy = dist.euclidean((w_h, h_h), (w_h, cy))
        if (h_h < cy):
            dy = -dy
        dx = dx / 1.640625
        dy = dy / 1.5624375
        dtr = m.pi / 180;
        T = np.array([[1, 0, 0], [0, 1, 0], [60, 18.5, 1]])
        R = np.array([[m.cos(-177.8 * dtr), m.sin(-177.8 * dtr), 0], [-m.sin(-177.8 * dtr), m.cos(-177.8 * dtr), 0], [0, 0, 1]])
        TR = np.dot(T, R)
        [dx, dy, dz] = np.dot([dx, dy, 1], TR)
        cv2.circle(orig, (cx, cy), 2, (0, 0, 255), 4)
        cv2.imshow("Image", orig)
        cv2.circle(orig, (cx, cy), 2, (0, 0, 255), 4)
        cv2.imshow("Image", orig)
        cv2.waitKey(0)
        return dx,dy
    else:
        return 0,0