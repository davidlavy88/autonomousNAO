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

def CenterOfMass(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 100 lower range of saturation for daveed's place, day. 165 night 95 sat; 10 hue bottom
    # 195 lower range of saturation for travis' place
    lowera = np.array([160, 95, 0])
    uppera = np.array([180, 250, 255])
    lowerb = np.array([0, 95, 0])
    upperb = np.array([10, 250, 255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cont, hier = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    icenter = []
    if len(cont)>=1:
        maxim=0
        for i in range(len(cont)):
            if len(cont[i])>len(cont[maxim]):
                maxim=i
        contour=cont[maxim]
        center, radius = cv2.minEnclosingCircle(contour)
        icenter.append(int(center[0]))
        icenter.append(int(center[1]))
        radius = int(radius)
        print 'Center', center, '. Radius', radius
        if radius>=8 and radius<60:
            i=icenter[1]
            j=icenter[0]
        else:
            i=0
            j=0
    else:
        i=0
        j=0
        icenter=[1,1]
        contour=cont
        radius=1

    CM=[i,j]
    # remove output of contour for use in code
    return CM, contour, icenter, radius

if __name__ == '__main__':

  image = "test0.png"
  img = cv2.imread(image)
  CM, cont, icenter, radius =CenterOfMass(img)
  cv2.drawContours(img, cont, -1, (0,190,255), 2)
  cv2.circle(img, (icenter[0], icenter[1]), radius, (255,0,0),2)
  cv2.circle(img,(CM[1],CM[0]),2,(0,255,0),3)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  cv2.imwrite("CircTest0.png", img)
