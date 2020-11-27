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
		
		self.lastYellowYZ = np.array([0,0])
		self.lastBlueYZ = np.array([0,0])
		self.lastGreenYZ = np.array([0,0])
		self.lastRedYZ = np.array([0,0])
		self.lastYellowXZ = np.array([0,0])
		self.lastBlueXZ = np.array([0,0])
		self.lastGreenXZ = np.array([0,0])
		self.lastRedXZ = np.array([0,0])
		self.lasta1 = 0
		self.lasta2 = 0
		
	def detect_circles(self, img1, img2):
		circles1 = image1.detect_black_circles(img1)
		circles2 = image2.detect_black_circles(img2)
		
		if (circles1 == np.array([0,0])).any():
			a1 = self.lasta1
			a2 = self.lasta2		
		
		else:
			a1 = image1.pixel2meter2(circles1[0],circles1[1])
			a2 = image2.pixel2meter2(circles2[0],circles2[1])
			self.lasta1 = a1
			self.lasta2 = a2
		
		if (circles1 == np.array([0,0])).any():
			yellowYZ = self.lastYellowYZ
			blueYZ = self.lastBlueYZ
			greenYZ = self.lastGreenYZ
			redYZ = self.lastRedYZ
		else:
			yellowYZ = a1*circles1[0]
			blueYZ = a1*circles1[1]
			greenYZ = a1*circles1[2]
			redYZ = a1*circles1[3]
			self.lastYellowYZ = yellowYZ
			self.lastBlueYZ = blueYZ
			self.lastGreenYZ = greenYZ
			self.lastRedYZ = redYZ
			
		if (circles2 == np.array([0,0])).any():
			yellowXZ = self.lastYellowXZ
			blueXZ = self.lastBlueXZ
			greenXZ = self.lastGreenXZ
			redXZ = self.lastRedXZ
		else:
			yellowXZ = a1*circles2[0]
			blueXZ = a1*circles2[1]
			greenXZ = a1*circles2[2]
			redXZ = a1*circles2[3]
			self.lastYellowXZ = yellowXZ
			self.lastBlueXZ = blueXZ
			self.lastGreenXZ = greenXZ
			self.lastRedXZ = redXZ
		
		yellowXYZ = np.array([yellowXZ[0], yellowYZ[0],(yellowYZ[1]+yellowXZ[1])/2])
		blueXYZ = np.array([blueXZ[0], blueYZ[0],(blueYZ[1]+blueXZ[1])/2])
		greenXYZ = np.array([greenXZ[0], greenYZ[0],(greenYZ[1]+greenXZ[1])/2])
		redXYZ = np.array([redXZ[0], redYZ[0],(redYZ[1]+redXZ[1])/2])
		joints = np.array([yellowXYZ,blueXYZ,greenXYZ,redXYZ])
		
		return joints
		
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

		#proj2 = self.projection(proj_xz, np.array([1,0,0]))
		x_rotation = 0
		y_rotation =0
		#positive x rotation
		if link_vector[1]<=0:
			x_rotation = self.vector_angle(proj_yz, [0,0,-1])
			if x_rotation > np.pi/2:
				x_rotation = np.pi/2
		#negative x rotation
		else:
			x_rotation = -self.vector_angle(proj_yz, [0,0,-1])
			if x_rotation < -np.pi/2:
				x_rotation = -np.pi/2
		#if np.abs(self.lastXrot - x_rotation) > np.pi/2:
		#	x_rotation = self.lastXrot
		self.lastXrot = x_rotation
		if link_vector[0]>=0:
			y_rotation = self.vector_angle(proj_yz, link_vector)
		else:
			y_rotation = -self.vector_angle(proj_yz, link_vector)
		return(x_rotation, y_rotation)
		
	def jointangles(self, joints):
		yellow = joints[0]
		blue = joints[1]
		green = joints[2]
		red = joints[3]

		vectBG = green - blue
		vectGR = red - green
		
		joint2and3 = self.plane_angles(vectBG)
		
		joint_4 = self.vector_angle(vectBG, vectGR)
		#print(joint_4)
		if(vectGR[1]> 0):
			joint_4 = -joint_4
		

		return np.array([0, joint2and3[0], joint2and3[1], joint_4])
	
	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		joints = self.detect_circles(self.cv_image1,self.cv_image2)
		jointangles = self.jointangles(joints)
		
		try:
			self.robot_joint2_angle_estimated_pub.publish(jointangles[1])
			self.robot_joint3_angle_estimated_pub.publish(jointangles[2])
			self.robot_joint4_angle_estimated_pub.publish(jointangles[3])
			
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
