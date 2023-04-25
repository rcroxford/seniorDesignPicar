# SPDX-FileCopyrightText: 2019 Dave Astels for Adafruit Industries
#
# SPDX-License-Identifier: MIT


"""
Consume LIDAR measurement file and create an image for display.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.
"""

import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar
import time
import numpy as np
#import motorControl as momo
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import cv2
#from picamera import PiCamera
import threading
import queue

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

#momo.Servo_Motor_Initialization()
#momo.Motor_Start(pca)
#servo7.angle = 75.25
#momo.Motor_Speed(pca,0.25)
# Set up pygame and the display
#os.putenv('SDL_FBDEV', '/dev/fb1')
#pygame.init()
#lcd = pygame.display.set_mode((320,240))
#pygame.mouse.set_visible(False)
#lcd.fill((0,0,0))
#pygame.display.update()

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME,timeout=3)
systime = time.time()
delay = time.time()
# used to scale data to fit on the screen
#max_distance = 0
turning = False
doneturn = False
turnAngle = 77
isThreading = False
steerError = 0
thread = 0
#pylint: disable=redefined-outer-name,global-statement
img_result = queue.Queue()

servo7.angle = 180
time.sleep(1)
servo7.angle = 0
time.sleep(1)
servo7.angle = turnAngle
time.sleep(1)
#cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 80)
import motorControl as momo

momo.Motor_Speed(pca,0.175)

def trackpavementangle(img_result):
    
    try:
        #cap.release()
        cap = cv2.VideoCapture('/dev/video0',cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #if not cap.isOpened():
          #  print("camera not open")
        #else:
           # print("camera opened")
        #time.sleep(3)
        while True:
            #print("reading camera")
            ret, img = cap.read()
           # print(f'ret: {ret} img: {img}')
            #cap.release()
            #print("camera read")
            #if not img:
              #  print("no image")
              #  continue
            dim = img.shape
            rows = dim[0]
            columns = dim[1]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            #(hue,sat,value)
            lower_thresh = np.array([30,100,95])
            upper_thresh = np.array([100,255,225])
            mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
            cv2.imwrite('Cameratester.png',img)
            cv2.imwrite('CameratestMask.png',mask)
            moments = cv2.moments(mask)
    
            if int(moments['m00']) == 0:
                #pavement not seen
                center = None
                img_result.put(0)
                print('not seeing pavement')
            else:
                cx = int(moments['m10']/moments['m00'])
                cy = int(moments['m01']/moments['m00'])
                error = 320 - cx
                #print(error)
                if error > 70:
                    #steer needs to be smaller angle
                    img_result.put(-5)
                elif error < -70:
                    #steern need to the larger angle
                    img_result.put(5) 
                else:
                    img_result.put(0)

    except KeyboardInterrupt:
        cap.release()
        #print("camera released")

def process_data(data):
    #global max_distance
    #lcd.fill((0,0,0))
    global systime
    global turning
    global doneturn
    global turnAngle
    global isThreading
    global steerError
    global thread

    #Steering Control###
    servo7.angle = turnAngle
    if not isThreading:
        thread  = threading.Thread(target = trackpavementangle, args = (img_result,))
        thread.start()
        isThreading = True
    #if thread.is_alive():
        #x = 1
        #print("waiting for image")
    #else:
    if not img_result.empty():
        #print("image proccessed")
        #thread.join()
        steerError = img_result.get()
        #print(f'steerError: {steerError}')
        if not turning:
            turnAngle = turnAngle + steerError
            if turnAngle < 67:
                turnAngle = 67
            elif turnAngle > 82:
                turnAngle = 82
        #isThreading = False
        #print(f'turnangle: {turnAngle}')
        #momo.Motor_Speed(pca,0.25)

    distance = data[359]
    turndist = data[115]
    #print(f'360:{distance}')
    #print(f'115:{turndist}')
    #print(data)
    if distance < 1500 and not turning and not doneturn:                  # ignore initially ungathered data points 
      if (time.time() - delay) > 3 and distance!=0:
            print("quick! turn! really fast! get out the way! this is unneccessarily long!")
            turning = True
            systime = time.time()
            turnAngle = 180 
            doneturn = True
    
    if distance < 3500 and not turning and (time.time() - delay) > 3 and distance!=0:
        print("angle reset to 77")
        turnAngle = 77

    if turning and ((time.time()-systime)>0.75): #and turndist < 2500: #distance : #and ((time.time() - systime) > 1.15):
        print("Stop Turning!")
        turning = False
        turnAngle = 77
        momo.Motor_Speed(pca,0.14)

    #if ((time.time()-systime)>4):
    #    print("extra angle")
    #    servo7.angle = 77

scan_data = [0]*360

try:
    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data)
        #print(distance)
            #print(f'angle: {angle}')
except KeyboardInterrupt:
    print('Stoping.')
    momo.Motor_Speed(pca,0)
    
lidar.stop()
lidar.disconnect()

