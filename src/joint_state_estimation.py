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
from sympy import symbols, Matrix, cos, sin
import sympy

class angle_estimator:
	
	def __init__(self):
		rospy.init_node('angle_estimator', anonymous=True)

		self.robot_joint2_estimated_pub = rospy.Publisher("/robot/joint2_position_estimator/command", Float64, queue_size=10)
		self.robot_joint3_estimated_pub = rospy.Publisher("/robot/joint3_position_estimator/command", Float64, queue_size=10)
		self.robot_joint4_estimated_pub = rospy.Publisher("/robot/joint4_position_estimator/command", Float64, queue_size=10)

		self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
		self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
		self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
		self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
		
		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.blue = np.array([0,0,0])
		self.green = np.array([0,0,0])
		self.red = np.array([0,0,0])
		
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
		redYZ = a1 *image1.detect_red(img1)
		redXZ = a2 *image2.detect_red(img2)

		xyz = np.array([redXZ[0], redYZ[0], (redYZ[1]+redXZ[1])/2])
		if (np.array([0,0,0]) == xyz).any():
			xyz = self.red
		self.red = xyz
		
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
		greenYZ = a1 * image1.detect_green(img1)
		greenXZ = a2 * image2.detect_green(img2)

		
		xyz = np.array([greenXZ[0], greenYZ[0], (greenYZ[1]+greenXZ[1])/2])
		if (np.array([0,0,0]) == xyz).any():
			xyz = self.green
		self.green = xyz
		return xyz


	def dh_matrix(self, d, theta, alpha, r):
		return(np.array([   [np.cos(theta)  , -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha)     , r*np.cos(theta)],
							[np.sin(theta)  , np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha)    , r*np.sin(theta)],
							[0	            , np.sin(alpha)               ,  np.cos(alpha)                  , d],
							[0              , 0                           , 0                               , 1]]))
	def dh_matrix_sym(self, d, theta, alpha, r):
		return(Matrix([     [cos(theta)  , -sin(theta)*cos(alpha), sin(theta)*sin(alpha)     , r*cos(theta)],
							[sin(theta)  , cos(theta)*cos(alpha) , -cos(theta)*sin(alpha) , r*sin(theta)],
							[0	         , sin(alpha)               ,cos(alpha)                 ,d],
							[0           , 0                        , 0                         ,1]]))
							
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
		
	def jointangles(self, img1, img2):
		yellow = self.detect3dyellow(img1, img2)
		blue = self.detect3dblue(img1, img2)
		green = self.detect3dgreen(img1, img2)
		red = self.detect3dred(img1, img2)

		vectYB = blue - yellow
		vectBG = green - blue
		vectGR = red - green
		
		joint2and3 = self.plane_angles(vectBG)
		
		joint_4 = self.vector_angle(vectBG, vectGR)
		#print(joint_4)
		if(vectGR[1]> 0):
			joint_4 = -joint_4
		

		return np.array([joint2and3[0], joint2and3[1]])

	def sym_forward_kinametics(self, ds, thetas, alphas, rs):
		theta0, theta1, theta2, theta3, r0, r1, r2, r3, alpha0, alpha1, alpha2, alpha3, d0, d1, d2, d3 = symbols('theta0 theta1 theta2 theta3 r0 r1 r2 r3 alpha0 alpha1 alpha2 alpha3 d0 d1 d2 d3')
		matrix0 = (self.dh_matrix_sym(d0, theta0, alpha0, r0))
		matrix1 = (self.dh_matrix_sym(d1, theta1, alpha1, r1))
		matrix2 = (self.dh_matrix_sym(d2, theta2, alpha2, r2))
		matrix3 = (self.dh_matrix_sym(d3, theta3, alpha3, r3))

		finalMatrix = (((matrix0 * matrix1) * matrix2) * matrix3)
		finalMatrix = (finalMatrix.subs([(theta0 ,thetas[0]), (theta1 ,thetas[1]),(theta2 ,thetas[2]),(theta3 ,thetas[3])]))
		finalMatrix = (finalMatrix.subs([(d0 ,ds[0]), (d1 ,ds[1]),(d2 ,ds[2]),(d3 ,ds[3])]))
		finalMatrix = (finalMatrix.subs([(alpha0 ,alphas[0]), (alpha1, alphas[1]),(alpha2 ,alphas[2]),(alpha3 ,alphas[3])]))
		finalMatrix = (finalMatrix.subs([(r0 ,rs[0]), (r1 ,rs[1]),(r2 ,rs[2]),(r3 ,rs[3])]))

		return(np.array(finalMatrix).astype(np.float64))

	def dh_matrix_jacobian(self, matrix, jas):
		return(matrix.jacobian(jas))
		

	def forward_kinematics(self, ds, thetas, alphas, rs):
		final_matrix = np.eye(4,4)
		matrix0 = dh_matrix(ds[0],thetas[0],  alphas[0], rs[0])
		matrix1 = dh_matrix(ds[1],thetas[1],  alphas[1], rs[1])
		matrix2 = dh_matrix(ds[2],thetas[2],  alphas[2], rs[2])
		matrix3 = dh_matrix(ds[3],thetas[3],  alphas[3], rs[3])
		return ((matrix0.dot(matrix1)).dot(matrix2)).dot(matrix3)
	
	def end_effector_estimate(self, jas):
		ds = [2.5, 0, 0,0]
		thetas= np.deg2rad([-90,90,180,0]) + jas
		rs = [0,0,3.5,3]
		alphas = np.deg2rad([-90,-90,-90,0])
		matrix = np.round(self.sym_forward_kinametics(ds,thetas, alphas,rs),2)
		ee_pos = matrix[:,3][:3]
		return ee_pos, matrix


	def control_closed(self,jas):
		# P gain
		K_p = np.array([[10,0],[0,10]])
		# D gain
		K_d = np.array([[0.1,0],[0,0.1]])
		# estimate time step
		cur_time = np.array([rospy.get_time()])
		dt = cur_time - self.time_previous_step
		self.time_previous_step = cur_time
		# robot end-effector position
		pos, matrix = self.end_effector_estimate(jas)
		# THIS NEEDS TO BE SPHERE POSITION
		pos_d= self.trajectory() 
		# estimate derivative of error
		self.error_d = ((pos_d - pos) - self.error)/dt
		# estimate error
		self.error = pos_d-pos
		q = jas
		J_inv = np.linalg.pinv(self.calculate_jacobian(matrix, jas))  # calculating the psudeo inverse of Jacobian
		dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
		q_d = q + (dt * dq_d)  # control input (angular position of joints)
		return q_d
		
	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)
		if (self.blue == np.array([0,0,0])).all():
			self.blue = self.detect3dblue(self.cv_image1, self.cv_image2)
		self.joints = Float64MultiArray()
		jointsData = self.jointangles(self.cv_image1, self.cv_image2)
		
		newOrig = self.detect3dyellow(self.cv_image1, self.cv_image2)
		
		jointsData = np.array([np.pi,1.57,1.57,-1.57])
		
		ee_pos, matrix = self.end_effector_estimate(jointsData)
		ee_pos = [ee_pos[0], ee_pos[1], -ee_pos[2]]
		print(ee_pos)
		self.joints.data = jointsData
		print(newOrig)
		print(newOrig + ee_pos)
		print(self.detect3dred(self.cv_image1, self.cv_image2))
		#print(jointsData)

		try:
			self.robot_joint1_pub.publish(self.joints.data[0])
			self.robot_joint2_pub.publish(self.joints.data[1])
			self.robot_joint3_pub.publish(self.joints.data[2])
			self.robot_joint4_pub.publish(self.joints.data[3])
			
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