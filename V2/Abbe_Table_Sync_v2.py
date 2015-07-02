#!/usr/bin/env python
# Note the following must be running in a different tab and Baxter must be enabled
# roslaunch baxter_moveit_config demo_baxter.launch

import math
import random
import rospy
import argparse
import struct
import sys
import numpy
import cv2
import tf
import copy
from abbe_ik import Abbe_IK
import threading
import Tkinter as tk
import json
import moveit_commander
import time
import urllib, json
from abbe_gripper import Abbe_Gripper

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    Grasp,
    GripperTranslation,
)


from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
	Polygon,
	Vector3,
    Vector3Stamped
)

import yaml


from trajectory_msgs.msg import(
    JointTrajectory,
    JointTrajectoryPoint
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range
from Abbe_Three_Points_To_Rot_Matrix import Abbe_Three_Points_To_Rot_Matrix

class Abbe_Table_Sync(object):
	_server = "http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com" #CHANGE THIS TO YOUR SERVER'S DOMAIN
	_abbe_three_points_matrix = False
	_ik = False
	_tkinter = False
	_move_left_arm = True
	_move_arm_offset = 0.1
	_last_move_to_file_x = False
	_last_move_to_file_y = False
	_currently_moving = False
	_grippers = False
	_ik = False
	_right_arm_rotation = 0
	_object_count = 0
	_object_already_added_to_moveit = []

	def __init__(self):
		self._ik = Abbe_IK()

		print "enabling grippers"
		self._grippers = Abbe_Gripper()

		#move it
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		print "sleeping for 2 seconds for moveit scene"
		rospy.sleep(2)

		self.draw_table_in_rviz()

		self._ik = Abbe_IK()
		self._abbe_three_points_matrix = Abbe_Three_Points_To_Rot_Matrix()

		self._require_configuration()
		#note: if the config file doesn't exist yet the init() script will never get past this point

		while not rospy.is_shutdown():
			self._check_for_new_object_on_table()
			time.sleep(5)

	def _check_for_new_object_on_table(self):
		url = self._server + "/touch_json_last_object.php"
		response = urllib.urlopen(url)
		data = json.loads(response.read())
		try:
			x = float(data["x"])
			y = float(data["y"])

			pose = [x,y]
			if pose in self._object_already_added_to_moveit:
				print "check for new object: already imported"
			else:
				self._object_already_added_to_moveit.append(pose)
				#read the RFID
				self.go_to_relative_position(float(data["x"]),float(data["y"]),True)
				self._point_rfid_reader_down_on_right_arm(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))

				#TODO check for new RFID value and import everything to moveit
				self.draw_leading_point(data["x"],data["y"],data["orientation_in_radians"])
				self._get_right_arm_out_of_the_way()

		except:
			print "check for new object: last object is false"

	def _require_configuration(self):
		# If a config file (that tells transformation of touch screeen's vs robot's poses) already exists skip the configuration step
		if not self._abbe_three_points_matrix.does_config_file_exist():
			#TkInter is just an easy way to capture key stroaks in python.
			self._point_arms_straight_down()
			print "Opening TkInter window to capture key strokes.  Make sure it stays in focus."
			self._tkinter = tk.Tk()
			#Size doesn't matter
			self._tkinter.geometry('300x200')
			#On each keypress call _onKeypress function
			self._tkinter.bind('<KeyPress>', self._onKeypress)

			#instructions on how to configure robot
			print "Moving left arm first.  Set to orientation 0,0 (relative to the robot, bottom, left corner of screen) on screen.";
			print "Movement Speed... T: fast, G: Medium, B: Slow";
			print "Move with W,A,S,D like an old school video game";
			print "Save Pose... M";
			self._tkinter.mainloop()

	def draw_table_in_rviz(self):
		p = PoseStamped()
		thickness_of_table = 0.025 #was 0.795
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.28 + (0.72/2)
		p.pose.position.y = 0
		p.pose.position.z = -0.175 - (thickness_of_table/2)
		self.scene.add_box("table",p,(0.72, 1.2, thickness_of_table))

	def _onKeypress(self,event):
		_c = event.char
		
		#changing speed?
		if _c == 't':
			self._move_arm_offset = 0.1
		if _c == 'g':
			self._move_arm_offset = 0.03
		if _c == 'b':
			self._move_arm_offset = 0.01
		if _c == 'y':
			self._move_arm_offset = 0.003
		if _c == 'm':
			self.save_point()
		if _c == 'u':
			self._draw_parameter()
		if _c == 'j':
			self._go_to_center()
		if _c == 'k':
			self._go_to_center_x_bottom_y()
		if _c == 'l':
			self._go_to_left_x_center_y()
		if _c == 'q':
			self.go_to_front_point_of_last_object_for_rfid()
		if _c == '0':
			self._point_arms_straight_down()
		if _c == '1':
			print "will open in 3 seconds"
			time.sleep(3)
			self._grippers.open(False)
		if _c == '2':
			print "will close in 5 seconds"
			time.sleep(5)
			self._grippers.close(False)
		if _c == '3':
			print "will open in 3 seconds"
			time.sleep(3)
			self._grippers.open(True)
		if _c == '4':
			print "will close in 5 seconds"
			time.sleep(5)
			self._grippers.close(True)
		if _c == '5':
			print "go to leading point"
			self.go_to_leading_point()
		if _c == '9':
			self.pickup_object()

		if(not (_c == 'w' or _c == 'a' or _c == 's' or _c == 'd')):
			 return False
		
		if(self._move_left_arm):
			pose = self._ik.get_pose('left')
		else:
			pose = self._ik.get_pose('right')
		
		y = pose.y
		x = pose.x

		if _c == 'a':
			y = y + self._move_arm_offset
		if _c == 'd':
			y = y - self._move_arm_offset
		if _c == 'w':
			x = x + self._move_arm_offset
		if _c == 's':
			x = x - self._move_arm_offset
		
		print "moving... (wait for success prior to pressing another key)"

		#prohibit multiple _ik requests simul
		if(self._currently_moving):
			return True

		self._currently_moving = True #prevent invalid key strokes

		if(self._move_left_arm):
			if not self._ik.set_left(float(x),float(y),float(pose.z)):
				print "left failed to point down at pose"
			else:
				print "success left arm at:"
				print self._ik.get_pose("left")
		else:
			if not self._ik.set_right(float(x),float(y),float(pose.z)):
				print "right failed to point down at pose"	
			else:
				print "success right arm at:"
				print self._ik.get_pose("right")
		
		self._currently_moving = False

	def _draw_parameter(self):
		print "going to 0,0"
		self.go_to_relative_position(0,0)
		print "arrived ... sleeping for 2"
		time.sleep(2)
		print "going to 1,0"
		self.go_to_relative_position(1,0)
		print "arrived ... sleeping for 2"
		time.sleep(2)
		print "going to 1,1"
		self.go_to_relative_position(1,1)
		print "arrived ... sleeping for 2"
		time.sleep(2)
		print "going to 0,1"
		self.go_to_relative_position(0,1)
		print "arrived .. done"

	def _go_to_center(self):
		self.go_to_relative_position(0.5,0.5)

	def _go_to_center_x_bottom_y(self):
		self.go_to_relative_position(0.5,0)

	def _go_to_left_x_center_y(self):
		self.go_to_relative_position(0,0.5)

	def save_point(self):
		if self._move_left_arm:
			self._move_left_arm = False
			_pos = self._ik.get_pose('left')
			print "saved"
			print "Now let's move the right arm to 1,0 (bottom,right) ... press m ... and then after that 1,1 (top,right) and press m"
			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)			
		else:
			_pos = self._ik.get_pose('right')
			print "saved"
			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)

	def go_to_relative_position(self,x_per,y_per, force_right = False, force_left = False):
		pos = self._abbe_three_points_matrix.determine_a_relative_point(x_per,y_per)
		return self._go_to_position(pos[0],pos[1],x_per,force_right,force_left)

	def go_to_front_point_of_last_object_for_rfid(self):
		url = self._server + "/touch_json_last_object.php"
		response = urllib.urlopen(url)
		data = json.loads(response.read())
		try:
			self.go_to_relative_position(float(data["x"]),float(data["y"]),True)
			self._point_rfid_reader_down_on_right_arm(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))
		except:
			print "last object is false"

	def go_to_leading_point(self):
		url = self._server + "/touch_json_last_object_rviz.php"
		response = urllib.urlopen(url)
		data = json.loads(response.read())
		try:
			self.go_to_relative_position(float(data["x"]),float(data["y"]),True)
		except:
			print "last object is false"

	def draw_leading_point(self,x,y,orientation_in_radians):

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()

		p.pose.position.z = -0.175 + (0.095 / 2)

		try:
			tmp_pos = self._abbe_three_points_matrix.determine_a_relative_point(float(x),float(y))
			p.pose.position.x = tmp_pos[0]
			p.pose.position.y = tmp_pos[1]

			quaternion = tf.transformations.quaternion_from_euler(0,0,orientation_in_radians)

			p.pose.orientation.x = quaternion[0]
			p.pose.orientation.y = quaternion[1]
			p.pose.orientation.z = quaternion[2]
			p.pose.orientation.w = quaternion[3]


			self.scene.add_box("cup" + str(self._object_count),p,(0.08, 0.14, 0.095)) #0.72 ... is the size of the object
			self._object_count = self._object_count + 1

		except:
			print "draw_leading_point: failed"

	def pickup_object(self):
		url = self._server + "/touch_json_last_object_pickup.php"
		response = urllib.urlopen(url)
		data = json.loads(response.read())
		try:
			#TODO move right arm straight up first to avoid colis
			self.go_to_relative_position(1,1,True) #get right hand out of the way
			self._grippers.open(True)
			self.go_to_relative_position(float(data["x"]),float(data["y"]),False, True) #move left arm to pickup location
			self._drop_left_arm_to_pickup_height(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))
			self._grippers.close(True)
			self.go_to_relative_position(0,0,False, True)


		except:
			print "last object is false"

	def _go_to_position(self,_tmp_x,_tmp_y,x_per, force_right = False, force_left = False):
		_prioritize_left_hand = True

		if not force_left:
			if force_right:
				_prioritize_left_hand = False

			if(x_per > 0.5):
				_prioritize_left_hand = False

		#print "going to " + str(_tmp_x) + " " + str(_tmp_y)
		if _prioritize_left_hand:
			#print "priority left due to " + str(x_per)
			if(not self._ik.set_left(_tmp_x,_tmp_y,self._default_height())):
				print "left failed trying right..."
				if force_left:
					print "right hand prohibited"
					return False
				if(not self._ik.set_right(_tmp_x,_tmp_y,self._default_height())):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False
		else:
			#print "priority right due to " + str(x_per)
			if(not self._ik.set_right(_tmp_x,_tmp_y,self._default_height())):
				print "right failed trying left..."
				if force_right:
					print "left hand prohibited"
					return False
				if(not self._ik.set_left(_tmp_x,_tmp_y,self._default_height())):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False		

		return True

	def _default_height(self):
		return 0.2

	def _point_rfid_reader_down_on_right_arm(self,radians_for_rfid = 0):

		pose = self._ik.get_pose('right')
		temp_height = 0.1
		print "Radians for rfid is: " + str(radians_for_rfid)
		#TODO change height from 0.2 to 0 and the last 0 to -0.2 or correct value

		#to avoid colis turn prior to dropping
		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),float(pose.z),0,radians_for_rfid):
			print "failed to turn prior to going down"

		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),0,0,radians_for_rfid):
			print "right failed to point rfid down at this radians... should try opposite here with correct offsets"
			#if self._ik.set_right_rfid_down(float(pose.x),float(pose.y),0,0,radians_for_rfid - math.pi):
			#	print "opposite orientation success"
			#	return True
			#print "opposite failed as well"
			return False
		print "RFID is down"
		return True

	def _drop_left_arm_to_pickup_height(self, orient_radians):
		pose = self._ik.get_pose('left')
		#TODO orientation

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians):
			print "failed to turn prior to going down: " + str(orient_radians)
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians - math.pi):
				print "failed to turn prior to going down opposite: " + str(orient_radians - math.pi)

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),-0.05,0,orient_radians):
			print "Left arm failed to drop"
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),-0.05,0,orient_radians - math.pi):
				print "Left arm failed to drop"

	def _get_right_arm_out_of_the_way(self):
		pose = self._ik.get_pose('right')
		if not self._ik.set_right(float(pose.x),float(pose.y),self._default_height()):
			print "couldnt go up first"
		if not self._ik.set_right(float(0.5),float(-0.5),self._default_height()):
			print "couldnt go out of the way first"

	def _point_arms_straight_down(self):

		#print "End effectors must point down for calibration... sleeping for 2 seconds to make sure IK is live"
		time.sleep(2)
		pose = self._ik.get_pose('left')

		if not self._ik.set_left(float(pose.x),float(pose.y),self._default_height()):
			print "left failed to point down at pose"
			exit()

		pose = self._ik.get_pose('right')

		if not self._ik.set_right(float(pose.x),float(pose.y),self._default_height()):
			print "right failed to point down at pose"
			exit()

		print "ready for calibration"

if __name__ == '__main__':
    rospy.init_node('Abbe_Table_Sync', anonymous=True)
    at = Abbe_Table_Sync()
