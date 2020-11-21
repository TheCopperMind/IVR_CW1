#!/usr/bin/env python3

import image1
import image2

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from math import pi
from math import atan2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class target_detector:
	
	def __init__(self):
		rospy.init_node('target_detector', anonymous=True)
		
		self.robot_target_estimated_pub = rospy.Publisher("/robot/target_position_estimated/command", Float64, queue_size=10)

		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.bridge = CvBridge()

	def detect3dtarget(self, img1, img2):
		#a1 = image1.pixel2meter(img1)
		#a2 = image1.pixel2meter(img2)
		#targetYZ = a1*image1.detect_target(img1)
		#targetXZ = a2*image2.detect_target(img2)
 		
		circle1 = image1.detect_target(img1)
		#xyz = np.array([targetXZ[0], targetYZ[0], (targetYZ[1]+targetXZ[1])/2])
		#return xyz
		return circle1

	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.target = Float64MultiArray()
		targetData = self.detect3dtarget(self.cv_image1, self.cv_image2)
		self.target.data = targetData

		print(targetData)

		try:
				self.robot_target_estimated_pub.publish(self.target.data[0])
			
		except CvBridgeError as e:
			print(e)

# call the class
def main(args):
  target = target_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
