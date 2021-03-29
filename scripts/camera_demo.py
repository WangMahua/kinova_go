#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""
import sys

reload(sys)
sys.setdefaultencoding('utf-8')
import roslib; roslib.load_manifest('kinova_demo')
import rospy
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Point
import sys
import numpy as np
import time

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from numpy import linalg as LA


# for gui screen
import Tkinter as tk
import math
import argparse
from kinova_msgs.srv import HomeArm, Stop, Start
from kinova_msgs.msg import KinovaPose
import threading
import Queue

currentFingerPosition = [0.0, 0.0, 0.0]
target_point = []
state = 0 
mission_data = Int32()
ball_pos_in_kinova = Point()
average_ball_pos = Point()
last_average_ball_pos = Point()
real_ball_pos = Point()
ball_pos_list = []
pos_lst = [[],[],[]]

kinova_pose = Point()
kinova_complete = 0
flag_ = 0
state_num_ = 0
state_flag = 0

q = Queue.Queue(maxsize=10)
sumx = 0
sumy = 0 
sumz = 0 

#function for kinova finger open and close
def gripper_client(prefix,finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None

# function for robot arm position control
def cartesian_pose_client(prefix,position, orientation):
	"""Send a cartesian goal to the action server."""
	action_address = '/' + prefix + 'driver/pose_action/tool_pose'
	client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
	client.wait_for_server()

	goal = kinova_msgs.msg.ArmPoseGoal()
	goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
	goal.pose.pose.position = geometry_msgs.msg.Point(
		x=position[0], y=position[1], z=position[2])
	goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
		x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

	# print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

	client.send_goal(goal)

	if client.wait_for_result(rospy.Duration(10.0)):
		return client.get_result()
	else:
		client.cancel_all_goals()
		print('        the cartesian action timed-out')
		return None

######             common             ######
def QuaternionNorm(Q_raw):
	qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
	qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
	qx_ = qx_temp/qnorm
	qy_ = qy_temp/qnorm
	qz_ = qz_temp/qnorm
	qw_ = qw_temp/qnorm
	Q_normed_ = [qx_, qy_, qz_, qw_]
	return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
	Q_normed = QuaternionNorm(Q_raw)
	qx_ = Q_normed[0]
	qy_ = Q_normed[1]
	qz_ = Q_normed[2]
	qw_ = Q_normed[3]

	tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
	ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
	tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
	EulerXYZ_ = [tx_,ty_,tz_]
	return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
	tx_, ty_, tz_ = EulerXYZ_[0:3]
	sx = math.sin(0.5 * tx_)
	cx = math.cos(0.5 * tx_)
	sy = math.sin(0.5 * ty_)
	cy = math.cos(0.5 * ty_)
	sz = math.sin(0.5 * tz_)
	cz = math.cos(0.5 * tz_)

	qx_ = sx * cy * cz + cx * sy * sz
	qy_ = -sx * cy * sz + cx * sy * cz
	qz_ = sx * sy * cz + cx * cy * sz
	qw_ = -sx * sy * sz + cx * cy * cz

	Q_ = [qx_, qy_, qz_, qw_]
	return Q_


def verboseParser(verbose, pose_mq_):
	""" Argument verbose """
	position_ = pose_mq_[:3]
	orientation_q = pose_mq_[3:]
	if verbose:
		orientation_rad = Quaternion2EulerXYZ(orientation_q)
		orientation_deg = list(map(math.degrees, orientation_rad))
		print('Cartesian position is: {}'.format(position_))
		print('Cartesian orientation in Quaternion is: ')
		print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]))
		print('Cartesian orientation in Euler-XYZ(radian) is: ')
		print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
		print('Cartesian orientation in Euler-XYZ(degree) is: ')
		print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))

class RobotArm():
	def __init__(self,kinova_robotType):
		#Robot param set
		self.robot_category = kinova_robotType[0]
		self.robot_category_version = int(kinova_robotType[1])
		self.wrist_type = kinova_robotType[2]
		self.arm_joint_number = int(kinova_robotType[3])
		self.robot_mode = kinova_robotType[4]
		self.finger_number = int(kinova_robotType[5])
		self.prefix = kinova_robotType+ "_"
		finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
		finger_maxTurn = 6800  # max thread turn for one finger
		#Rosnode init
		
		self.unit = 'mdeg'
		self.currentCartesian = [0.2, -0.27, 0.506, 1.64, 1.108, -0.04]
		self.setpoint = [0.2, -0.27, 0.506, 1.64, 1.108, -0.04]
		self.target_ = [ 0, 0, 0, 0, 0, 0]
		
		

	def start_GUI(self):
		##################################################	
		##				build button					##
		##################################################	
		tk.Button(Button_frame,  text='GO!', command=lambda:[self.go()]).grid(row = 1 ,column = 1)
		tk.Button(Button_frame,  text='Go to set point', command=self.go_to_setpoint).grid(row = 1 ,column = 2)
		tk.Button(Button_frame,	text = 'Reset point',command=self.reset_point).grid(row=1,column=3)
		## finger
		tk.Button(Finger_frame,text = 'finger close',command=lambda:[ gripper_client(self.prefix,[6800,6800,6800])]).grid(row=1,column=1)
		tk.Button(Finger_frame,text = 'finger open' ,command = lambda:[gripper_client(self.prefix,[0,0,0])]).grid(row=2,column=1)
		##kill button 
		tk.Button(Kill_Button_frame,text = "Stop",command = self.stop , height =20 , width =20).grid(row=1,column=1)
		tk.Button(Kill_Button_frame,text = "Start",command = self.start , height =20 , width =20).grid(row=1,column=2)
		


	def arm_get(self,data):
		command = data.data
		if command == 1:
			self.arm_home()
	def go(self): 
		threading.Thread(target=self.arm_go_thread).start()

	def arm_go_thread(self):
		data = []
		for i in range(6):
			data.append(self.m[i])
		pose_mq, pose_mdeg, pose_mrad = self.unitParser(data) # get q deg and rag 
		try:

			poses = [float(n) for n in pose_mq]

			result = cartesian_pose_client(self.prefix,poses[:3], poses[3:])

			print('Cartesian pose sent!')

		except rospy.ROSInterruptException:
			print "program interrupted before completion"
		verboseParser(1, poses)

	def go_to_setpoint(self):
		self.arm_home()
	def stop(self):
		self.arm_stop()
	def start(self):
		self.arm_start()
        
	def reset_point(self):
		for i in range(6):
			self.m[i].set(self.currentCartesian[i])

	def print_select_value(self,value):
		print(value)
	def unitParser(self, pose_value_):
		relative_ = 0
		position_ = pose_value_[:3]
		orientation_ = pose_value_[3:]
		if relative_:
			orientation_rad_list =  currentCartesianCommand[3:]
			orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
		else:
			orientation_rad = orientation_
		orientation_deg = list(map(math.degrees, orientation_rad))
		orientation_q = EulerXYZ2Quaternion(orientation_rad)
		pose_mq_ = position_ + orientation_q
		pose_mdeg_ = position_ + orientation_deg
		pose_mrad_ = position_ + orientation_rad

		return pose_mq_, pose_mdeg_, pose_mrad_
	def getcurrentCartesianCommand(self,prefix_):
	# wait to get current position
		topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
		rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand)
		rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
		print 'position listener obtained message for Cartesian pose. '


	def setcurrentCartesianCommand(self,feedback):
		currentCartesianCommand_str_list = str(feedback).split("\n")
		for index in range(0,len(currentCartesianCommand_str_list)):
			temp_str=currentCartesianCommand_str_list[index].split(": ")
			self.currentCartesian[index] = float(temp_str[1])

	def getpos_from_camera(self,state_num,target_): #new
		self.m =[]
		self.arm_home = rospy.ServiceProxy('/j2n6s300_driver/in/home_arm',HomeArm)
		self.arm_stop = rospy.ServiceProxy('/j2n6s300_driver/in/stop',Stop)
		self.arm_start = rospy.ServiceProxy('/j2n6s300_driver/in/start',Start)
		self.arm_s = rospy.Subscriber('arm/command',Int32,self.arm_get)
		self.arm_p = rospy.Publisher('arm/finish',Int32,queue_size=1)

		global average_ball_pos
		global flag_ 
		global kinova_complete
		global real_ball_pos
		global kinova_pose
		global state_num_
		global state_flag


		print(state_num)

		if state_num == 0:
			target_=[-0.368360340595, -0.0915138721466, 0.498772323132, 3.1319835186, 0.000802508671768, -2.05962896347]
			for i in range(6):
				self.m.append(target_[i])
				print(self.m)
			self.go()
			print('0 first point')
			time.sleep(2)
			print(kinova_pose.x)
			print(kinova_pose.y)
			print(kinova_pose.z)
			if abs(kinova_pose.x-target_[0])<0.03 and abs(kinova_pose.y-target_[1])<0.03 and abs(kinova_pose.z-target_[2])<0.03:
				state_num = 1
				state_num_ = 1 

		if state_num == 1 and state_num_ <2:  #recognize ball
			target_=[-0.00153621006757, -0.404122054577, 0.496579319239, -3.13902044296, 0.0104509945959, 2.90323400497]
			for i in range(6):
				self.m.append(target_[i])
			print(self.m)	
			self.go()
			print('1 in set')
			self.m =[]
			print(kinova_pose.x)
			print(kinova_pose.y)
			print(kinova_pose.z)
			if abs(kinova_pose.x-target_[0])<0.1 and abs(kinova_pose.y-target_[1])<0.1 and abs(kinova_pose.z-target_[2])<0.1:
				state_num_ = 2
			time.sleep(2)

		if state_num_ == 2:  #recognize ball
			if flag_ == 0 :
				real_ball_pos = average_ball_pos 
				flag_ = -1
				gripper_client('j2n6s300_',[0,0,0])
				time.sleep(1)

			print("real_ball_pos")	
			print(real_ball_pos.x)
			print(real_ball_pos.y)
			print(real_ball_pos.z)
			target_=[real_ball_pos.x, real_ball_pos.y, real_ball_pos.z+0.11, -3.13902044296, 0.0104509945959, 2.90323400497]
			for i in range(6):
				self.m.append(target_[i])
			print(self.m)
			self.go()
			print('2 in set')
			if abs(kinova_pose.x-target_[0])<0.03 and abs(kinova_pose.y-target_[1])<0.03 and abs(kinova_pose.z-target_[2])<0.03:
				state_num_ = 3
			self.m =[]
			time.sleep(2)


		if state_num_ == 3:  #recognize ball
			print('3 in set')
			target_=[real_ball_pos.x, real_ball_pos.y, real_ball_pos.z+0.05, -3.13902044296, 0.0104509945959, 2.90323400497]
			for i in range(6):
				self.m.append(target_[i])
			print(self.m)
			self.go()
			if abs(kinova_pose.x-target_[0])<0.03 and abs(kinova_pose.y-target_[1])<0.03 and abs(kinova_pose.z-target_[2])<0.03:			
				state_num_ = 4
			self.m =[]
			time.sleep(2)
			
		if state_num_ == 4:  #finger close 
			gripper_client('j2n6s300_',[6800,6800,6800])
			print('4 in set')
			time.sleep(0.5)
			state_num_ = 6
			

		if state_num == 666:  #finger open 
			gripper_client('j2n6s300_',[0,0,0])
			
			time.sleep(0.5)

		if state_num_ == 6:  #recognize ball
			print('3 in set')
			target_=[real_ball_pos.x, real_ball_pos.y, real_ball_pos.z+0.2, -3.13902044296, 0.0104509945959, 2.90323400497]
			for i in range(6):
				self.m.append(target_[i])
			print(self.m)
			self.go()
			
			#state_num = 4
			self.m =[]
			time.sleep(2)
			kinova_complete = 1
		if state_num == 5:  #finger open 
			gripper_client('j2n6s300_',[0,0,0])
			
			time.sleep(0.5)
		mission_data.data = kinova_complete
		mission_state.publish(mission_data)
		print (mission_data)
		       

def camera_callback(data):
	coordinate_input = data.data 


def state_callback(data):
	global state 
	state = data.data
	mission_state.publish(mission_data)
	print('in callback')
	kinova_robotType = 'j2n6s300'
	KINOVA = RobotArm(kinova_robotType)
	KINOVA.getpos_from_camera(state,target_point)

def ball_pos_callback(data):
	global ball_pos_in_kinova
	global q
	global average_ball_pos
	global last_average_ball_pos
	global pos_lst
	global sumx
	global sumy 
	global sumz 
	global state
	global flag 

	ball_pos_in_kinova = data 
	if abs(data.x) <=0.6 and abs(data.y)<=0.6 and abs(data.z)<=1 and state>0:
		
		pos_lst[0].append[data.x]
		pos_lst[1].append[data.y]
		pos_lst[2].append[data.z]

	
	average_ball_pos.x = np.median(pos_lst[0])
	average_ball_pos.y = np.median(pos_lst[1])
	average_ball_pos.z = np.median(pos_lst[2])

	'''
	q.put(ball_pos_in_kinova)
	
	if q.full() == True and flag_ == 0:
		for i in range(10):
			temp_pos = q.get()

			sumx += temp_pos.x
			sumy += temp_pos.y
			sumz += temp_pos.z
			if i == 9:			
				average_ball_pos.x = sumx/10
				average_ball_pos.y = sumy/10
				average_ball_pos.z = sumz/10
				sumx = 0
				sumy = 0 
				sumz = 0
				a = np.array([average_ball_pos.x,average_ball_pos.y,average_ball_pos.z])
				b = np.array([last_average_ball_pos.x,last_average_ball_pos.y,last_average_ball_pos.z])
				error = LA.norm(a) - LA.norm(b)
				print(error)
				if abs(error)<0.02 and state>1 :
					flag_ = -1
					print("flag=-1")
					print(average_ball_pos.x)
					print(average_ball_pos.y)
					print(average_ball_pos.z)
				last_average_ball_pos = average_ball_pos
	'''				
	kinova_robotType = 'j2n6s300'
	KINOVA = RobotArm(kinova_robotType)
	KINOVA.getpos_from_camera(state,target_point)	

def KinovaPose_callback(data):
	kinova_pose.x = data.X
	kinova_pose.y = data.Y
	kinova_pose.z = data.Z

if __name__ == '__main__':
	rospy.init_node('j2n6s300_pose_action_client')
	rospy.Subscriber("camera_coordinate", String, camera_callback)
	rospy.Subscriber("ball_pos", Point, ball_pos_callback)
	rospy.Subscriber("j2n6s300_driver/out/cartesian_command", KinovaPose, KinovaPose_callback)
	# rospy.Subscriber("camera_coordinate", String, camera_callback)
	rospy.Subscriber("state", Int32, state_callback)
	mission_state = rospy.Publisher("kinova_ok", Int32, queue_size=1)
	mission_data.data = 0
	mission_state.publish(mission_data)

	rospy.spin()

