#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

# In this method you can focus on detecting the centre of the red circle
def detect_red(image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    if M['m00'] == 0:
    	return np.array([0,0])
    else:
    	# Calculate pixel coordinates for the centre of the blob
    	cx = int(M['m10'] / M['m00'])
    	cy = int(M['m01'] / M['m00'])
    	return np.array([cx, cy])

# Detecting the centre of the green circle
def detect_green(image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
    	return np.array([0,0])
    else:
    	# Calculate pixel coordinates for the centre of the blob
    	cx = int(M['m10'] / M['m00'])
    	cy = int(M['m01'] / M['m00'])
    	return np.array([cx, cy])


# Detecting the centre of the blue circle
def detect_blue(image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
    	return np.array([0,0])
    else:
    	# Calculate pixel coordinates for the centre of the blob
    	cx = int(M['m10'] / M['m00'])
    	cy = int(M['m01'] / M['m00'])
    	return np.array([cx, cy])

# Detecting the centre of the yellow circle
def detect_yellow(image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
    	return np.array([0,0])
    else:
    	# Calculate pixel coordinates for the centre of the blob
    	cx = int(M['m10'] / M['m00'])
    	cy = int(M['m01'] / M['m00'])
    	return np.array([cx, cy])
    	
def detect_target(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (10,100,20), (25,255,255))
    
    ret,thresh = cv2.threshold(mask,0,255,cv2.THRESH_BINARY)
    mask[thresh == 0] = 255
    mask = cv2.bitwise_not(thresh)
    
    contours, h = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    c = max(contours,key=len)
    approx = cv2.approxPolyDP(c, .03*cv2.arcLength(c, True), True)
    if(len(approx))>5:
    	(x,y),r = cv2.minEnclosingCircle(c)
    	return np.array([x,y])
    else:
    	return np.array([0,0])
      
def findYellow(circles):
    distance = 1000
    closestCircleIndex = 0
    yellowPos = np.array([399,529])
    
    i = 0
    for circle in circles[0,:]:
    	center = np.array([circle[0],circle[1]])
    	newDistance = np.linalg.norm(center-yellowPos)
    	if newDistance < distance:
    		distance = calculateDistance(center,yellowPos)
    		closestCircleIndex = i
    	i+=1
    		
    return closestCircleIndex
    
def findBlue(circles,yellowCircleIndex):
    distance = 1000
    closestCircleIndex = 0
    bluePos = np.array([399,472])
    
    i = 0
    for circle in circles[0,:]:
    	center = np.array([circle[0],circle[1]])
    	newDistance = np.linalg.norm(center-bluePos)
    	if newDistance < distance and i != yellowCircleIndex:
    		distance = calculateDistance(center,bluePos)
    		closestCircleIndex = i
    	i+=1
    		
    return closestCircleIndex
   	
def detect_black_circles(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY_INV)
    
    ret,thresh2 = cv2.threshold(thresh,0,255,cv2.THRESH_BINARY)
    thresh[thresh2 == 0] = 255
    thresh = cv2.bitwise_not(thresh2)
    
    canny = cv2.Canny(thresh,200,300)

    circles = cv2.HoughCircles(image=canny,method=cv2.HOUGH_GRADIENT, dp=0.8, minDist=30, param1=40, param2=10, minRadius=0, maxRadius=15)
    circles = np.int16(np.around(circles))

    if len(circles[0]) == 4:
        yellowIndex = findYellow(circles)
        yellowPos = np.array([circles[0,yellowIndex,0],circles[0,yellowIndex,1]])
        blueIndex = findBlue(circles,yellowIndex)
        bluePos = np.array([circles[0,blueIndex,0],circles[0,blueIndex,1]])
  	
        distance = 1000
        closestCircleIndex = 0
  	
        i = 0
        for circle in circles[0,:]:
            center = np.array([circle[0],circle[1]])
            newDistance = np.linalg.norm(center-bluePos)
            if newDistance < distance and i!=yellowIndex and i!=blueIndex:
                distance = calculateDistance(center,bluePos)
                closestCircleIndex = i
            i+=1
    		
        greenIndex = closestCircleIndex
        greenPos = np.array([circles[0,closestCircleIndex,0],circles[0,closestCircleIndex,1]])
        
        indexes = np.array([0,1,2,3])
        for index in indexes:
            if index!=blueIndex and index!=yellowIndex and index!=greenIndex:
                redIndex = index
	
        redPos = np.array([circles[0,redIndex,0],circles[0,redIndex,1]])
    	
        circlesList = np.array([yellowPos,bluePos,greenPos,redPos])
        return circlesList
    else:
        return np.array([0,0])
        
def pixel2meter2(circle1,circle2):
    dist = np.sum((circle1 - circle2)**2)
    return 2.5 / np.sqrt(dist)

# Calculate the conversion from pixel to meter
def pixel2meter(image):
    # Obtain the centre of each coloured blob
    circle1Pos = detect_blue(image)
    circle2Pos = detect_yellow(image)
    # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    return 2.5 / np.sqrt(dist)

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

