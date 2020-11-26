#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64


# Publish data
def move():
  rospy.init_node('move_joints_2', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()])-t0
    joint2 = Float64()
    joint2.data = (np.pi/2)*np.sin(cur_time*np.pi/15)
    robot_joint2_pub.publish(joint2.data)
            
    joint3 = Float64()
    joint3.data = (np.pi/2)*np.sin(cur_time*np.pi/18)
    robot_joint3_pub.publish(joint3.data)
            
    joint4 = Float64()
    joint4.data = (np.pi/2)*np.sin(cur_time*np.pi/20)
    robot_joint4_pub.publish(joint4.data)
            
    print(joint2.data,joint3.data,joint4.data)
    rate.sleep()



# run the code if the node is called
if __name__ == '__main__':
  try:
    move()
  except rospy.ROSInterruptException:
    pass
