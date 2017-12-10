#!/usr/bin/python

################################################################
# posUv2.py
# final project
#
# Authors: Alexander Bernard, Timothy Duggan, Christian Welling
# NetIDs: arb392, trd44, csw73
# Date Written: 11/27/17
# Last Revision: 12/9/17
#
################################################################
#
## Description:
# This class defines a position conrol object which contains fields and methods
# relevant to detemining the distance (x,y) between the camera center and where
# the drone needs to be. This class does all of the CV computation upon the call
# of the update function. This 


# imports
import argparse
import imutils
import time
import cv2
import numpy as np
import psutil
import os

class posU:
    def __init__(self):
        # what happens upon initialization
        self.dXpix = 0
        self.dYpix = 0
        
        # [x,y] or [col, row]
        self.cX = 0        
        self.cY = 0
        
        self.cntr = None
        self.hsv_cntr = None
        
        
    
    # definitions
    def getCentroid(self,px,py):
        self.cX = np.divide(px,2) 
        self.cY = np.divide(py,2)
        
    def getCenterInfo(self,img):
        self.cntr = img[self.cY,self.cX]
        print self.cntr

        # print out center screen HSV values for calibration data
        center_color = np.uint8([[cntr]])
        self.hsv_cntr = cv2.cvtColor(center_color,cv2.COLOR_BGR2HSV)
        print self.hsv_cntr
    
    def printAverage(self,avgX,avgY):
        print("Average Center Approximation:  [ %f, %f]" % (avgX,avgY))
        
    def getPercentX(self,avgX,dimP):
        return int(100*np.divide(float(avgX-int(self.cX)),float(dimP)))
    
    def getPercentY(self,avgY,dimP):
        return int(100*np.divide(float(-(avgY-int(self.cY))),float(dimP)))
    
    def displayImages(self,img,mask,mask2,res,res_GRAY):
        #Display outputs for testing
        #cv2.imshow('img',img)
        cv2.imshow('mask',mask)
        cv2.imshow('Filtered Mask',mask2)
        #cv2.imshow('res',res)
        cv2.imshow('GRAY Image',res_GRAY) 
        
        # Display all three outputs (concatenate them together)
        #frame_disp = cv2.hconcat((img,mask))
        #frame_disp = cv2.hconcat((cv2.hconcat((img,mask))),res)
        #cv2.imshow('final frame',frame_disp)
        cv2.imshow("Images",np.hstack([img,res]))
    
    def update(self,img,px,py):
        
        self.getCentroid(px,py)
        
        # convert BGR to HSV
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
                
        # define range of blue color in HSV
        lower_color_range = np.array([175,10,10])     # works pretty well
        upper_color_range = np.array([185,255,255])   # works pretty well
        
        # Threshold the HSV image to only get the color you filtered
        mask = cv2.inRange(hsv,lower_color_range,upper_color_range)
        #mask2 = mask.copy()
        
        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(img,img,mask= mask)
        
        # Convert to BGR from HSV
        #res_BGR = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
        
        # Convert to GREY from BRG (GRAY is monochrome which should work with findContours)
        #res_GRAY = cv2.cvtColor(res_BGR,cv2.COLOR_BGR2GRAY)
        #res_GRAY2 = res_GRAY.copy()
        
        #### FILTERING ####
        #kernel = np.ones((5,5),np.uint8)
        #erosion = cv2.erode(mask2,kernel,iterations=1)
        #opening = cv2.morphologyEx(res_GRAY2,cv2.MORPH_OPEN,kernel)
        
        #### CENTROID APPROXMAITION 1 (Works Pretty Well)
        X = np.linspace(0,(px-1),px)
        Y = np.linspace(0,(py-1),py)
        xV,yV = np.meshgrid(X,Y)

        xF = cv2.bitwise_and(xV,xV,mask= mask)
        yF = cv2.bitwise_and(yV,yV,mask= mask)
        
        sum_xF = np.sum(xF)
        sum_yF = np.sum(xF)
        
        if sum_xF == 0 or sum_yF == 0:
            trackX_avgM = int(self.cX)
            trackY_avgM = int(self.cY)
        else: 
            trackX_avgM = int(np.average(xF[xF!=0]))
            trackY_avgM = int(np.average(yF[yF!=0]))
      
        #print("Moment Centroid Approximation: " + [trackX_cntr,trackY_cntr])
        #cv2.circle(img,(trackX_avgM,trackY_avgM),4,(255,255,100),4)
         
        
        ### CENTROID APPROXIMATION 2: USE CONTOURING TO APPROXIMATE CENTER OF MASS
        #contours, h = cv2.findContours(res_GRAY,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # attempt to make the contours better
        #epsilon = 0.1*cv2.arcLength(contours1[:],True)
        #contours = cv2.approxPolyDP(contours1[:],epsilon,True)
        
        #print len(contours)
        #print contours[40]
        
        # centroid approximation Option 1: average of all points 
        #trackX_avg = int(np.average(contours[:][0]))
        #trackY_avg = int(np.average(contours[:][1]))
        
        # centroid approximation Option 2: centroid approximation via moments
##        cnt = contours[:]
##        M = cv2.moments(cnt)
##        trackX_cntr = int(M['m10']/M['m00'])
##        trackY_cntr = int(M['m01']/M['m00'])
        
        ##      cv2.drawContours(img,contours,-1,[0,255,0],1)
        #cv2.circle(img,(trackX_avg,trackY_avg),4,(255,100,0),4)
     
        
        #self.displayImages(img,mask,res_GRAY2,res,res_GRAY)
        
        self.dXpix = self.getPercentX(trackX_avgM,px)
        self.dYpix = self.getPercentY(trackY_avgM,py) 
        
        # returns the output
        return [self.dXpix, self.dYpix]
    
 
