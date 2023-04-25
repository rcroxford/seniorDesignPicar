import cv2
import time
import numpy as np
sys_time = time.time()
angle = 0
#this timing is just what i've used in 205 to have finite time but we can adjust/remove this component 
t = 100



while((time.time()-sys_time)<t):

#   cap = cv2.VideoCapture('/dev/video0',cv2.CAP_V4L)
 #  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
  # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
   #ret, frame = cap.read()
   #cv2.imwrite('trackit.jpg',frame)
   #cap.release()
   #img = cv2.imread('trackit.jpg')

 #  def trackpavementangle():
    cap = cv2.VideoCapture('/dev/video0',cv2.CAP_V4L)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
    ret, img = cap.read()
    cap.release()
    print("img taken")
    dim = img.shape
    rows = dim[0]
    columns = dim[1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      #(hue,sat,value)
    lower_thresh = np.array([30,100,95])
    upper_thresh = np.array([100,255,225])
    mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
    cv2.imwrite('testCamera.png',mask)
    moments = cv2.moments(mask)

    if int(moments['m00']) == 0:
         #pavement not seen
       center = None
       print('not seeing pavement')
    else:
       cx = int(moments['m10']/moments['m00'])
       cy = int(moments['m01']/moments['m00'])
       error = 1280 - cx
       print(error)
       #if error > 0:
             #steer needs to be smaller angle
          # return -1
       #if error < 0:
             #steern need to the larger angle
          # return 1 
#center = (cy, cx)
        # dimc = (columns/2) -cx
         #dimr = rows -cy
         #hyp = np.sqrt((dimr**2)+(dimc**2))
         #angle = np.arcsin(dimc/hyp)
         #angle = abs((angle*180)/(np.pi)*2))
#Need to use angle to talk to steering to move that much
#   trackpavementangle()
