#capture image, get center of mass

import sys
import time
import math
import cv2
import numpy as np

# Python Image Library
from PIL import Image

from naoqi import ALProxy
import vision_definitions
import time
# from matplotlib import pyplot as plt

def CenterOfMassDown(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #100 lower range of saturation for daveed's place, day. 165 night 95 sat; 10 hue bottom
    #195 lower range of saturation for travis' place
    kernel = np.ones((5,5),np.uint8)
    lowera = np.array([160,95,0])
    uppera = np.array([180,250,255])
    lowerb = np.array([0,95,0])
    upperb = np.array([10,250,255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cont, hier = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    if len(cont)>=1:
        maxim=0
        for i in range(len(cont)):
            if len(cont[i])>len(cont[maxim]):
                maxim=i
        contour=cont[maxim]
    
        maxi=np.amax(contour,0)
        mini=np.amin(contour,0)
        center=(maxi+mini)/2
        i=center[0][1]
        j=center[0][0]

    else:
        i=0
        j=0
        contour=cont

    CM=[i,j]
    # remove output of contour for use in code
    return CM, contour, cont, mask


if __name__ == '__main__':

  img = cv2.imread("test9.png")
  CM, contour,cont,mask=CenterOfMassDown(img)

  imgmask = cv2.bitwise_and(img, img, mask = mask)
  img = cv2.addWeighted(img,0.3,imgmask,0.7,0)

  cv2.drawContours(img, cont, -1, (0,190,255), 2)
  cv2.circle(img,(CM[1],CM[0]),15,(0,255,0),-1)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  cv2.imwrite("ContTest9.png", img)

