#!/usr/bin/python

################################################################
# camera_tracker_beta.py
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
# This class streams video and process frame by frame outputing tracking
# information

# import the necessary packages#!/usr/bin/python

from PosU import posU
from PiVideoStream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import time
import cv2
import os
import psutil

from threading import Thread

class Tracker:
    def __init__(self):
        
        ################ CAMERA SETUP ###################
        # initialize the camera and stream         
        self.vs = PiVideoStream().start()
        time.sleep(2.0)
        
        # start FPS counter
        #self.fps = FPS().start();                      # run this to keep track of frames
        
        # initialize variable indicating if thread should be stopped
        self.stopped = False
    
        # initialize return variables
        self.x = 0
        self.y = 0
        

    def start(self):
        Thread(target=self.update,args=()).start()
        return self
        
    def update(self):
        
        

        ############### IMAGE PROCESSING SETUP ##########
        dPos = posU()            # create instance of posU class

        ############### VARIABLES #######################
        #print 'are we in this part?'
        fWidth = None
        fHeight = None
        newPos = [0,0]

        ############### ACQUISITION OF DATA ##############
        while (not self.stopped):
            
            #to have a maximum width of 400 pixels
            frame = self.vs.read()                         # will just grab whatever the current frame is
            frame = imutils.resize(frame, width=400)
                
            r,c = frame.shape[:2]
            fWidth = c
            fHeight = r
         
            # DO IMAGE PROCESSING HERE
            newPos = dPos.update(frame,fWidth,fHeight)
            self.x = newPos[0]
            self.y = newPos[1]
            
            time.sleep(0.05)
            
            # Print for Testing
            #print str(str(self.x)+'|'+str(self.y))
            #print self.stopped
                
            # If you want to display the frame
            #cv2.imshow("Frame", frame)
        
            # update the FPS counter (THIS IS FPS FOR PROCESSING)
            #self.fps.update()
    
    def readX(self):
        return self.x
    
    def readY(self):
        return self.y
 
    def stop(self):
        self.stopped = True
        
        # CLEANUP
        # [1] Necessary
        self.vs.stop()
        
        # [2] Optional - If opened windows
        #cv2.destroyAllWindows()
        
        # [3] Optional - If ran FPS class
        # stop the timer and display FPS information
        #self.fps.stop()
        #print("[INFO] elasped time: {:.2f}".format(self.fps.elapsed()))
        #print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps())) 


