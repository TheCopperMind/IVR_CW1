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

#fix problem when m00 = 0, maybe in other file?

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
    if(len(approx))>4:
    	(x,y),r = cv2.minEnclosingCircle(c)
    	return np.array([x,y])
    else:
    	return (np.array([0,0]))
    	
def detect_black(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0,0,0), (180,255,50))
    
    ret,thresh = cv2.threshold(mask,0,255,cv2.THRESH_BINARY)
    mask[thresh == 0] = 255
    mask = cv2.bitwise_not(thresh)
    
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp = 1.0, minDist = 1.0, maxRadius = 20, param1 = 100, param2 = 10)
    
    circleCoords = []
    for circle in circles[0,:]:
    	circleCoords.append([circle[0], circle[1]])
    
    bluePos = getBlueJoint(circleCoords)
    circleCoords.remove(bluePos)
    circle2 = np.array(circleCoords[0])
    circle3 = np.array(circleCoords[1])
    circle2distance = distance(circle2,bluePos)
    circle3distance = distance(circle3,bluePos)
    
    if circle2distance < circle3distance:
    	greenPos = circle2
    	redPos = circle3
    else:
    	greenPos = circle3
    	redPos = circle2
    	
    circles = np.array([bluePos,greenPos,redPos])
    
    return circles
    
def getBlueJoint(circleCoords):
    z = 1000
    closest = []
    for circle in circleCoords:
    	if circle[1] < z:
           z = circle[1]
           closest = circle
    
    return closest
    
def distance(circle1, circle2):
    return np.linalg.norm(circle1-circle2)
    
def pixel2meter2(circle1,circle2):
   dist = np.sum((circle1 - circle2)**2)
   print(dist)
   return 3.5/(np.sqrt(dist))

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

