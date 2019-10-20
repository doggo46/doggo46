#!/usr/bin/python
#Import the needed libraries
import numpy as np
import cv2
from imutils.video import VideoStream
from time import sleep
import sys
import threading
import time
import RPi.GPIO as GPIO
import timeit

#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(18, GPIO.OUT)
#pin = GPIO.PWM(18, 55)
#pin.start(0)
Relay_Ch1 = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(Relay_Ch1,GPIO.OUT)
GPIO.output(Relay_Ch1,GPIO.HIGH)
pause = False

def setAngle():
    while True:
        global pause
        if (pause):
            GPIO.output(Relay_Ch1,GPIO.LOW)
            print("Channel 1:The Common Contact is access to the Normal Open Contact!")
            time.sleep(1.5)
    
            GPIO.output(Relay_Ch1,GPIO.HIGH)
            print("Channel 1:The Common Contact is access to the Normal Closed Contact!\n")
            time.sleep(0.5)
            pause = False
            
            

        
    
def red(hsv,redlower,redupper):
    redcontours = []
    image1 = cv2.inRange(hsv, redlower, redupper) #Look for the specified HSV color
    contours,_ = cv2.findContours(image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Find the contours of the color
    for contour in contours: #Loop through all of the contours
        (x,y,w,h) = cv2.boundingRect(contour) #Get the bounding box dimensions for the current contour
        if w > 25 and h > 25: #If the contour is wider and higher than 30px
            redcontours.append((x,y,w,h))
            #GPIO.output(Relay_Ch1,GPIO.LOW)
            #GPIO.output(Relay_Ch1,GPIO.HIGH)
            return redcontours #iztrii tui neshtu ako iskash poveche ot ino neshtu
    return redcontours
           
def green(hsv,greenlower,greenupper):
    greencontours = []
    image1 = cv2.inRange(hsv, greenlower, greenupper) #Look for the specified HSV color
    contours,_ = cv2.findContours(image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Find the contours of the color
    for contour in contours: #Loop through all of the contours
        (x,y,w,h) = cv2.boundingRect(contour) #Get the bounding box dimensions for the current contour
        if w > 25 and h > 25: #If the contour is wider and higher than 30px
            greencontours.append((x,y,w,h))
            #GPIO.output(Relay_Ch1,GPIO.LOW)
            #GPIO.output(Relay_Ch1,GPIO.HIGH)
            return greencontours #iztrii tui neshtu ako iskash poveche ot ino neshtu
    return greencontours
    
if __name__ == '__main__':

    #Green Color is in HSV format 
    greenlower = np.array([65,60,60]) #Lower boundry of the green color
    greenupper = np.array([80,255,255]) #Upper boundry of the green color

    #Red Color is in HSV format
    redlower = np.array([136,87,111]) #Lower boundry of the red color
    redupper = np.array([180,255,255]) #Upper boundry of the red color

    parallel = threading.Thread(target = setAngle)
    parallel.start()

    cap = VideoStream(src=0).start() #Start the camera
    sleep(2) #Wait for the camera to start up
    
    while True:
        image = cap.read() #Acquire frame
        
        resize = cv2.resize(image, (640, 480))
        #start_time = timeit.default_timer()
        hsv = cv2.cvtColor(resize, cv2.COLOR_BGR2HSV) #Change the image to HSV color space
        redcontours = red(hsv, redlower, redupper) #Call the function to compute the red contours
        greencontours = green(hsv,greenlower,greenupper) #Call the function to compute the green contours
        if (len(redcontours) > 0):
            pause = True
        elif (len(greencontours) > 0):
            pause = False
        for i in redcontours: #Iterate through the red contours
            cv2.rectangle(image, (i[0],i[1]), (i[0]+i[2],i[1]+i[3]), (0, 0, 255), 2) #Draw a red rectangle onto the original image with a thickness of 2px
            cv2.putText(image,"Red",(i[0],i[1]),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255)) #Write Red next to the rectangle
        for i in greencontours: #Iterate through the green contours
            cv2.rectangle(image, (i[0],i[1]), (i[0]+i[2],i[1]+i[3]), (0, 255, 0), 2) #Draw a green rectangle onto the original image with a thickness of 2px
            cv2.putText(image,"Green",(i[0],i[1]),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0)) #Write Green next to the rectangle
        #print(timeit.default_timer() - start_time)
        cv2.imshow("Color Tracking",image) #Open a window and display the processed image inside of it
        if cv2.waitKey(10) & 0xFF == ord('q'): #If q is pressed
            cap.stop() #Stop acquiring frames
            cv2.destroyAllWindows() #Close the window
            break

