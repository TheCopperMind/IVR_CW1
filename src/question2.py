import numpy as np
from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame
from sympy import symbols, Matrix, cos, sin
import sympy
#import rospy

#def calculate_jacobian(jas):
#    jacobian = np.array([[3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()), 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()),  3 *np.cos(joints.sum())], [-3 * np.sin(joints[0]) - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints.sum())]])
#    return jacobian

class question2:

	def __init__(self):
		self.time =0
		# initialize the node named image_processing
		#rospy.init_node('image_processing', anonymous=True)
		# initialize a publisher to send messages to a topic named image_topic
		#self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
		# initialize a publisher to send joints' angular position to a topic called joints_pos
		#self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
		# initialize a publisher to send robot end-effector position
		#self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
		# initialize a publisher to send desired trajectory
		#self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
		# initialize a publisher to send joints' angular position to the robot
		#self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
		#self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
		#self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
		#self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
		# initialize the bridge between openCV and ROS
		#self.bridge = CvBridge()
		# initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
		#self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)
		# record the begining time
		#self.time_trajectory = rospy.get_time()
		# initialize errors
		#self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
		#self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')   
		# initialize error and derivative of error for trajectory tracking  
		#self.error = np.array([0.0,0.0], dtype='float64')  
		#self.error_d = np.array([0.0,0.0], dtype='float64') 


	
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

	def sym_forward_kinametics(self, ds, thetas, alphas, rs):
		theta0, theta1, theta2, theta3, r0, r1, r2, r3, alpha0, alpha1, alpha2, alpha3, d0, d1, d2, d3 = symbols('theta0 theta1 theta2 theta3 r0 r1 r2 r3 alpha0 alpha1 alpha2 alpha3 d0 d1 d2 d3')
		matrix0 = (dh_matrix_sym(d0, theta0, alpha0, r0))
		matrix1 = (dh_matrix_sym(d1, theta1, alpha1, r1))
		matrix2 = (dh_matrix_sym(d2, theta2, alpha2, r2))
		matrix3 = (dh_matrix_sym(d3, theta3, alpha3, r3))

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
		thetas= np.deg2rad([-90,90,180,0])
		rs = [0,0,3.5,3]
		alphas = np.deg2rad([-90,-90,-90,0])
		matrix = np.round(question2.sym_forward_kinametics(ds,thetas, alphas,rs),2)
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

	# Recieve data, process it, and publish
	def callback(self,data):
	# Recieve the image
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	# Perform image processing task (your code goes here)
	# The image is loaded as cv_imag

	# Uncomment if you want to save the image
	#cv2.imwrite('image_copy.png', cv_image)

		cv2.imshow('window', cv_image)
		cv2.waitKey(3)

		# publish robot joints angles (lab 1 and 2)
		self.joints=Float64MultiArray()
		self.joints.data= self.detect_joint_angles(cv_image)


		# compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
		x_e = self.forward_kinematics(cv_image)
		x_e_image = self.detect_end_effector(cv_image)
		self.end_effector=Float64MultiArray()
		self.end_effector.data= x_e_image	

		# send control commands to joints (lab 3)
		q_d = self.control_closed(cv_image)
		#q_d = self.control_open(cv_image)
		self.joint1=Float64()
		self.joint1.data= q_d[0]
		self.joint2=Float64()
		self.joint2.data= q_d[1]
		self.joint3=Float64()
		self.joint3.data= q_d[2]

		# Publishing the desired trajectory on a topic named trajectory(lab 3)
		x_d = self.trajectory()    # getting the desired trajectory
		self.trajectory_desired= Float64MultiArray()
		self.trajectory_desired.data=x_d

		# Publish the results
		try: 
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			self.joints_pub.publish(self.joints)
			self.end_effector_pub.publish(self.end_effector)
			self.trajectory_pub.publish(self.trajectory_desired)
			self.robot_joint1_pub.publish(self.joint1)
			self.robot_joint2_pub.publish(self.joint2)
			self.robot_joint3_pub.publish(self.joint3)
		except CvBridgeError as e:
			print(e)

	#[2.5, 0, 0,0
	#np.deg2rad([-90,90,180,0])
	#[0,0,3.5,3]
	#np.deg2rad([-90,-90,-90,0])
	def projection(self, link_vector, normal_vector):
		print(link_vector)
		print(normal_vector.shape)
		return(link_vector - (np.dot(link_vector, normal_vector)/np.linalg.norm(normal_vector)**2)*normal_vector)
	
	#def vector_angle(self, u, v):
	#	return(np.arccos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))))

	def vector_angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

		
	def unit_vector(self, vector):
		return(vector / np.linalg.norm(vector))

	def plane_angles(self, link_vector):
		proj_xz = self.projection(link_vector, np.array([0,1,0]))
		proj_yz = self.projection(link_vector, np.array([1,0,0]))
		proj_xy = self.projection(link_vector, np.array([0,0,1]))

		#xz_angle = self.vector_angle(proj_xz, link_vector)
		#yz_angle = self.vector_angle(proj_yz, link_vector)
		#xy_angle = self.vector_angle(proj_xy, link_vector)
		x_rotation = 0
		y_rotation =0
		print(proj_yz)
		#positive x rotation
		if link_vector[1]<0:
			x_rotation = self.vector_angle(proj_yz, [0,0,-1])
		#negative x rotation
		else:
			x_rotation = -self.vector_angle(proj_yz, [0,0,-1])

		if link_vector[0]>0:
			y_rotation = self.vector_angle(proj_yz, link_vector)
		else:
			y_rotation = -self.vector_angle(proj_yz, link_vector)
		return(x_rotation, y_rotation)

def main():
	ic = question2()
	#define symbols for all links
	print(ic.plane_angles(np.array([0.06,-4.22,-2.64])))
		
	
	
	
	#print(np.round(question2.sym_forward_kinametics(ds,thetas, alphas,rs),2))
	#jacobian = grad(matrix)
	#print(ee_pos)
		
if __name__ == '__main__':
	main()
