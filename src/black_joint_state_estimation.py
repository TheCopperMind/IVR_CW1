#!/usr/bin/env python3

import image1
import image2

import roslib
import rospy
import sys
import cv2
import numpy as np
import message_filters
from math import pi
from math import atan2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class angle_estimator:
	
	def __init__(self):
		rospy.init_node('angle_estimator', anonymous=True)

		self.robot_joint2_angle_estimated_pub = rospy.Publisher("/robot/joint2_position_estimator/command", Float64, queue_size=10)
		self.robot_joint3_angle_estimated_pub = rospy.Publisher("/robot/joint3_position_estimator/command", Float64, queue_size=10)
		self.robot_joint4_angle_estimated_pub = rospy.Publisher("/robot/joint4_position_estimator/command", Float64, queue_size=10)

		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.bridge = CvBridge()
		
	def detect_circles(self, img1, img2):
		circles1 = image1.detect_black(img1)
		circles2 = image2.detect_black(img2)
		
		circles1Coords = []
		for circle in circles1[0,:]:
			circles1Coords.append(np.array([circle[0],circle[1]]))
			
		circles2Coords = []
		for circle in circles2[0,:]:
			circles2Coords.append(np.array([circle[0],circle[1]]))
	
		return circles2Coords
		
	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		circles2Coords = self.detect_circles(self.cv_image1, self.cv_image2)
		print(circles2Coords[2])
		
# call the class
def main(args):
  angles = angle_estimator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)