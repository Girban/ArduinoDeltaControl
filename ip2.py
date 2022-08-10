## program for colour detection using mobile camera

import numpy as np
import cv2
import requests
import imutils
def disp(i,color):
    p=str(i+1);
    if(color=='blue'):
        r=[255,0,0];
    elif(color=="red"):
        r=[0,0,255];
    elif(color=="green"):
        r=[0,255,0]
    elif (color == "orange"):
        r=[0, 10, 200]
    text="No "+p+color
    return text,r
def wcam(url1):
    cx = float('nan');
    cy = float('nan')
    ctr = [cx, cy];
    img_resp1 = requests.get(url1)
    img_arr1 = np.array(bytearray(img_resp1.content), dtype=np.uint8)
    img1 = cv2.imdecode(img_arr1, -1)
    frame = cv2.resize(img1, (500, 400), interpolation=cv2.INTER_AREA)
    return frame,ctr
def detect(c,url1):
    cx=float('nan');cy=float('nan')
    ctr=[cx,cy];
    img_resp1=requests.get(url1)
    img_arr1=np.array(bytearray(img_resp1.content),dtype=np.uint8)
    img1=cv2.imdecode(img_arr1,-1)
    frame=cv2.resize(img1,(500,400),interpolation=cv2.INTER_AREA)
    h1,w1,_=frame.shape;
    h1=int(h1)
    h_h1=int(h1/2)
    w1=int(w1)
    w_h1=int(w1/2)
    cv2.line(frame,(0,h_h1),(w1,h_h1),(255,255,255),1)
    cv2.line(frame,(w_h1,0),(w_h1,h1),(255,255,255),1)
    cv2.circle(frame,(w_h1,h_h1),2,(0,0,0),4)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if (c == "r"):
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([2, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # Range for upper range
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        # Generating the final mask to detect red color
        color = mask1 + mask2
        clr = "red"
        n = 1
    elif (c == "b"):
        L = np.array([110, 50, 50])
        H = np.array([130, 255, 255])
        color = cv2.inRange(hsv, L, H)
        clr = "blue"
        n = 1
    elif (c == "O"):
        L = np.array([10, 150, 50])
        H = np.array([20, 255, 255])
        color = cv2.inRange(hsv, L, H)
        clr = "orange"
        n = 1
    elif (c == "g"):
        L = np.array([42, 52, 72], np.uint8)
        H = np.array([90, 255, 255], np.uint8)
        color = cv2.inRange(hsv, L, H)
        clr = "green"
        n = 1
    elif (c == "All"):
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # Range for upper range
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        # Generating the final mask to detect red color
        color = mask1 + mask2
        clr = "red"
        n = 3;
    kernal = np.ones((5, 5), "uint8")
    color = cv2.dilate(color, kernal, iterations=4)
    (contours, hierarchy) = cv2.findContours(color, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        max_area = cv2.contourArea(contours[i])
        for j in range(i, len(contours)):
            area = cv2.contourArea(contours[j])
            if (max_area < area):
                max_area = area;
                cnt = contours[i];
                contours[i] = contours[j]
                contours[j] = cnt
        x0, y0, w0, h0 = cv2.boundingRect(contours[0])
        [text1, r] = disp(0, clr)
        frame = cv2.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), (r[0], r[1], r[2]), 2)
        cv2.putText(frame, text1, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (r[0], r[1], r[2]))
        M = cv2.moments(contours[0])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        ctr = [cx, cy]
    if (n == 3):
        Lb = np.array([110, 50, 50])
        Hb = np.array([130, 255, 255])
        blue = cv2.inRange(hsv, Lb, Hb)
        kernal = np.ones((5, 5), "uint8")
        blue = cv2.dilate(blue, kernal, iterations=4)
        (contours, hierarchy) = cv2.findContours(blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            max_area = cv2.contourArea(contours[i])
            for j in range(i, len(contours)):
                area = cv2.contourArea(contours[j])
                if (max_area < area):
                    max_area = area;
                    cnt = contours[i];
                    contours[i] = contours[j]
                    contours[j] = cnt
            x1, y1, w1, h1 = cv2.boundingRect(contours[0])
            frame = cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 0), 2)
            [text2, r] = disp(0, "blue")
            cv2.putText(frame, text2, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0))
    return frame, ctr