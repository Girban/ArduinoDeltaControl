## Program  for webcam

import numpy as np
import cv2
import requests
import imutils
def disp(i,color):
    p=str(i+1);
    if(color=='blue'):
        r=[255,0,0];
    else:
        r=[0,0,255];
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
    w, h, _ = frame.shape;
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, 270, 1)
    frame = cv2.warpAffine(frame, M, (h, w))
    return frame,ctr