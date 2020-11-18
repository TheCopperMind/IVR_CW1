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
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
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
