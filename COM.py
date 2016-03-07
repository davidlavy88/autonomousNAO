#capture image, get center of mass

import sys
import time
import cv2
import cv2.cv as cv
import numpy as np

# Python Image Library
from PIL import Image

from naoqi import ALProxy
import vision_definitions
import time
# from matplotlib import pyplot as plt



def CenterOfMass(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = image[:, :, ::-1].copy()

    lowera = np.array([160,95,0])
    uppera = np.array([180,250,255])
    lowerb = np.array([0,95,0])
    upperb = np.array([10,250,255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)

    mask = cv2.add(mask1,mask2)

    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mj=sum(mask)
    mi=sum(np.transpose(mask))
    A=mask.shape
    ni=np.array(range(A[0]))
    nj=np.array(range(A[1]))
    M=sum(sum(mask))
    if sum(mj)==0 or sum(mi)==0:
        print "no ball"
        xcm=0
        ycm=0
    else:
        xcm=np.dot(mj,nj)/sum(mj)
        
        ycm=np.dot(mi,ni)/sum(mi)
   
    CM=[ycm,xcm]
    
    return CM,mask

if __name__ == '__main__':

  img = cv2.imread("test3.jpg")
  CM,mask=CenterOfMass(img)
  print CM
  imgmask = cv2.bitwise_and(img, img, mask = mask)
  img = cv2.addWeighted(img,0.3,imgmask,0.7,0)
  cv2.circle(img,(CM[1],CM[0]),15,(0,255,0),-1)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  cv2.imwrite("COMtest3.png", img)
