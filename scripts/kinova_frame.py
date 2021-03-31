#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int32
from geometry_msgs.msg import Point
import numpy as np
from numpy.linalg import inv
pos_lst_x = []
pos_lst_y = []
pos_lst_z = []
ball_position = Point()
state = 0

def callback(data):
	global pos_lst_x
	global pos_lst_y
	global pos_lst_z
	global state
	global ball_position
	pub = rospy.Publisher("ball_pos", Point, queue_size=1)
	camera_coordinate_x = data.x  
	camera_coordinate_y = data.y 
	camera_coordinate_z = data.z
	camera_coordinate = np.array([[camera_coordinate_x], [camera_coordinate_y], [camera_coordinate_z]])


	camera_coordinate_new = np.array([[data.x], [data.y], [data.z],[1]])
	transform_matrix = np.array([[-1,0,0,0.132],[0,0,-1,0.555],[0,-1,0,0.3],[0,0,0,1]])
	kinova_coordinate_new = np.matmul(transform_matrix,camera_coordinate_new)

	print(state)
	if abs(kinova_coordinate_new.item(0)) <=0.7 and abs(kinova_coordinate_new.item(1))<=0.7 and abs(kinova_coordinate_new.item(2))<=1 and state!=0:
		
		pos_lst_x.append(kinova_coordinate_new.item(0))
		pos_lst_y.append(kinova_coordinate_new.item(1))
		pos_lst_z.append(kinova_coordinate_new.item(2))
		ball_position.x = np.median(pos_lst_x)
		ball_position.y = np.median(pos_lst_y)
		ball_position.z = np.median(pos_lst_z)
		print("get pos")

	pub.publish(ball_position)
	print("ball_position:"+str(ball_position))
	print("pos_lst_x:"+str(pos_lst_x))
	print("pos_lst_y:"+str(pos_lst_y))
	print("pos_lst_z:"+str(pos_lst_z))
	print('---')

def state_callback(data):
	global state 
	state = data.data
   

def listener():
	rospy.init_node('kinova_frame', anonymous=True)
	rospy.Subscriber("camera_coordinate",Point, callback)
	rospy.Subscriber("state", Int32, state_callback)
	rospy.spin()

if __name__ == '__main__':
	listener()

