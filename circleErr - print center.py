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
    
    if len(cont)>=1:
        minim=0
        minE=1
        for l in range(len(cont)):
            contour=cont[l]
            center, radius = cv2.minEnclosingCircle(contour)
            icenter = []
            icenter.append(int(center[0]))
            icenter.append(int(center[1]))
            radius = int(radius)
##            print 'Center', center, '. Radius', radius
            d=radius
            err=[]
            for k in range(len(contour)):
                t=[contour[k,0,0]-center[0],contour[k,0,1]-center[1]]
                t=math.sqrt(math.pow(t[0],2)+math.pow(t[1],2))
                e=abs(d-t)/d
                err=err+[e]
            ERR=np.mean(err)
            if ERR<minE and radius>=8 and radius<65:
                minim=l
                minE=ERR
                Radius=radius
                Center=icenter
##            print 'Error', ERR
        if minE>.20:
            i=0
            j=0
            contour=cont[minim]
            Radius=0
        else:
            i=Center[1]
            j=Center[0]
            contour=cont[minim]
    else:
        i=0
        j=0
        minim=1
        minE=1
        Center=[1,1]
        contour=cont
        Radius=0

    CM=[i,j]
##    print 'Ball Center', CM, '. Radius', Radius
    # remove output of contour for use in code
    return CM, contour, Radius, minE

if __name__ == '__main__':

  image = "trial12/ball_downfront1.png"
  img = cv2.imread(image)
  CM, cont, radius, err =CenterOfMass(img)
  if CM==[0,0]:
      string="Ball Not Found"
##      color=(0,0,255)
  else:
      string="Center: (" + str(CM[1]) + ", " + str(CM[0]) + ")"
      cv2.circle(img, (CM[1],CM[0]), radius, (255,0,0),2)
      cv2.circle(img,(CM[1],CM[0]),2,(0,255,0),10)
##  cv2.drawContours(img, cont, -1, (0,190,255), 2)
  cv2.putText(img, string, (10,30), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255), 2)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  cv2.imwrite("bdf1.png", img)
