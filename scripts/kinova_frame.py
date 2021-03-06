#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
from numpy.linalg import inv

def callback(data):
	camera_coordinate_x = data.x  
	camera_coordinate_y = data.y 
	camera_coordinate_z = data.z
	
	camera_coordinate = np.array([[camera_coordinate_x], [camera_coordinate_y], [camera_coordinate_z]])

	
	rotate_matrix = np.array([[-1.16829603,0.08799549,0.15661172],[-0.81759598,-0.84087181,-0.52033374],[-0.57934749,-0.64769659,0.41838879]])

	kinova_coordinate = np.matmul(rotate_matrix,camera_coordinate)

	print(kinova_coordinate)


	camera_coordinate_new = np.array([[camera_coordinate_x], [camera_coordinate_y], [camera_coordinate_z],[1]])
	transform_matrix = np.array([[-1,0,0,0.102],[0,0,-1,0.365],[0,-1,0,0.49],[0,0,0,1]])
	kinova_coordinate_new = np.matmul(transform_matrix,camera_coordinate_new)
	print(kinova_coordinate_new)
	print('---')
    

def listener():
	rospy.init_node('kinova_frame', anonymous=True)
	rospy.Subscriber("camera_coordinate",Point, callback)
	camera_coordinate_matrix = np.matrix('0.00428918032787,-0.154750487805,0.005275;-0.138367298014,-0.0678232596144,-0.153020599016;0.90706882041,0.674770707866,0.9248678145')
	kinova_coordinate_matrix = np.matrix('0.124870881438,0.280503243208,0.12521725893;-0.359136164188,-0.167551919818,-0.356882035732;0.466642528772,0.41589970112,0.483009189367')
	camera_coordinate_matrix_inv = inv(np.matrix(camera_coordinate_matrix))
	r_matrix=np.matmul(kinova_coordinate_matrix, camera_coordinate_matrix_inv)
	
	rospy.spin()

if __name__ == '__main__':
    listener()

