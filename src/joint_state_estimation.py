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

class angle_estimator:
	
	def __init__(self):
		rospy.init_node('angle_estimator', anonymous=True)

		self.robot_joint2_estimated_pub = rospy.Publisher("/robot/joint2_position_estimator/command", Float64, queue_size=10)
		self.robot_joint3_estimated_pub = rospy.Publisher("/robot/joint3_position_estimator/command", Float64, queue_size=10)
		self.robot_joint4_estimated_pub = rospy.Publisher("/robot/joint4_position_estimator/command", Float64, queue_size=10)

		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.bridge = CvBridge()

	def detect3dyellow(self, img1, img2):
		a1 = image1.pixel2meter(img1)
		a2 = image1.pixel2meter(img2)
		yellowYZ = a1*image1.detect_yellow(img1)
		yellowXZ = a2*image2.detect_yellow(img2)

		xyz = np.array([yellowXZ[0], yellowYZ[0], (yellowYZ[1]+yellowXZ[1])/2])
		return xyz

	def detect3dred(self, img1, img2):
		a1 = image1.pixel2meter(img1)
		a2 = image1.pixel2meter(img2)
		redYZ = a1*image1.detect_red(img1)
		redXZ = a2*image2.detect_red(img2)

		xyz = np.array([redXZ[0], redYZ[0], (redYZ[1]+redXZ[1])/2])
		return xyz

	def detect3dblue(self, img1, img2):
		a1 = image1.pixel2meter(img1)
		a2 = image1.pixel2meter(img2)
		blueYZ = a1*image1.detect_blue(img1)
		blueXZ = a2*image2.detect_blue(img2)

		xyz = np.array([blueXZ[0], blueYZ[0], (blueYZ[1]+blueXZ[1])/2])
		return xyz

	def detect3dgreen(self, img1, img2):
		a1 = image1.pixel2meter(img1)
		a2 = image1.pixel2meter(img2)
		greenYZ = a1*image1.detect_green(img1)
		greenXZ = a2*image2.detect_green(img2)

		xyz = np.array([greenXZ[0], greenYZ[0], (greenYZ[1]+greenXZ[1])/2])
		return xyz

	def projection(self, link_vector, normal_vector):
		return(link_vector - (np.dot(link_vector, normal_vector)/np.linalg.norm(normal_vector)**2)*normal_vector)

	def length(self, v):
		return np.sqrt(np.dot(v, v))

	def vector_angle(self, u, v):
		return(np.arccos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))))

	def plane_angles(self, link_vector):
		proj_xz = self.projection(link_vector, np.array([0,1,0]))
		proj_yz = self.projection(link_vector, np.array([1,0,0]))
		proj_xy = self.projection(link_vector, np.array([0,0,1]))

		xz_angle = self.vector_angle(proj_xz, link_vector)
		yz_angle = self.vector_angle(proj_yz, link_vector)
		xy_angle = self.vector_angle(proj_xy, link_vector)

		x_rotation = self.vector_angle(proj_yz, [0,0,1])
		y_rotation = self.vector_angle(proj_yz, link_vector)

		return(x_rotation, y_rotation)
		
	def jointangles(self, img1, img2):
		yellow = self.detect3dyellow(img1, img2)
		blue = self.detect3dblue(img1, img2)
		green = self.detect3dgreen(img1, img2)
		red = self.detect3dred(img1, img2)

		vectYB = blue - yellow
		vectBG = green - blue
		vectGR = red - green
		
		print(vectBG)
		joint2and3 = self.plane_angles(vectBG)
		
		angles = self.plane_angles(vectBG)
		vectBGcm1 = np.array([vectBG[1],vectBG[2]])
		#joint_2 = self.vector_angle(vectBG,np.array([0,0,-1]))
		#joint_2 = angles[0]
		#if(vectBG[1]>0):
			#joint_2 = (-1)*joint_2
			
		#joint_3 = self.vector_angle(vectGR, np.array([-1,0,0]))

		#joint_4 = np.pi - self.vector_angle(vectBG, vectGR)
		#if(red[0]>0):
			#joint_4 = -joint_4

		return np.array([joint2and3[0], joint2and3[1]])

	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.joints = Float64MultiArray()
		jointsData = self.jointangles(self.cv_image1, self.cv_image2)
		self.joints.data = jointsData

		print(jointsData)

		try:
			self.robot_joint2_estimated_pub.publish(self.joints.data[0])
			self.robot_joint3_estimated_pub.publish(self.joints.data[1])
			#self.robot_joint4_estimated_pub.publish(self.joints.data[2])
			
		except CvBridgeError as e:
			print(e)

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
