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
from abbe_emotion import Abbe_Emotions

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

	#Custom values to offset x & y coordinates in meters - this makes it easier to fix slightly inaccurate config files
	_screen_x_offset = -0.003
	_screen_y_offset = -0.021

	#The following 3 values control where objects should be dropped off
	_drop_off_cord_x = 0.5
	_drop_off_cord_y = 0.5
	_drop_off_cord_z = 0.1

	_z_height_of_gripper = 0.1
	_z_offset_above_table_to_scan_rfid_at = False
	_height_of_rfid_scanner = 0.075
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
	_last_object_type = False
	_last_object_pose = False
	_abbe_emotions = False
	_head = False

	def __init__(self):
		#set height for RFID scanner to read at
		self._z_offset_above_table_to_scan_rfid_at = self._z_height_of_gripper + self._height_of_rfid_scanner

		self._ik = Abbe_IK()
		self._abbe_emotions = Abbe_Emotions()

		print "enabling grippers"
		self._grippers = Abbe_Gripper()

		self._head = baxter_interface.Head()


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

		if data == False:
			return False

		if "x" in data:
			self._last_object_pose = data
			data["y"] = float(data["y"]) + float(self._screen_y_offset / self._abbe_three_points_matrix.ret_height_of_screen())
			data["x"] = float(data["x"]) + float(self._screen_x_offset / self._abbe_three_points_matrix.ret_width_of_screen())

			pose = [data["x"],data["y"]]
			if pose in self._object_already_added_to_moveit:
				print "check for new object: already imported"
			else:
				self._object_already_added_to_moveit.append(pose)
				#read the RFID

				self._head.set_pan(-0.7)

				rfid_pose = self.determine_object_rfid_pose(data["x"],data["y"],data["orientation_in_radians"])
				self.go_to_relative_position(float(rfid_pose[0]),float(rfid_pose[1]),True)

				self._point_rfid_reader_down_on_right_arm(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))

				#Detect everything there is to know about the object
				self._objectapi()
				#Draw the object in RVIZ
				self.draw_collision_object_in_rviz()

				#Move the right arm out of the way
				self.move_right_arm_up_with_same_radians_and_xy(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))
				self._get_right_arm_out_of_the_way()

				#Pickup Object
				self.pickup_object()

		else:
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
		p.pose.position.z = self.height_of_table() - (thickness_of_table/2)
		self.scene.add_box("table",p,(0.72, 1.2, thickness_of_table))

	def height_of_table(self):
		return -0.175

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

		offset_of_gripper_above_table = 0.02

		if(self._move_left_arm):
			if not self._ik.set_left(float(x),float(y),float(self.height_of_table()) + float(self._z_height_of_gripper) + float(offset_of_gripper_above_table)):
				print "left failed to point down at pose"
			else:
				print "success left arm at:"
				print self._ik.get_pose("left")
		else:
			if not self._ik.set_right(float(x),float(y),float(self.height_of_table()) + float(self._z_height_of_gripper) + float(offset_of_gripper_above_table)):
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

	def determine_center_of_object(self):
		#offsets are based on the orientation of the object so calculate the correct center of object coordinates by using rotation matrix
		x = float(self._last_object_pose["x"])
		y = float(self._last_object_pose["y"])

		#The x & y coordinates are percentages of the screen width.  The offsets are in meters so we need to covert these to percents as well (of screen).
		x_offset = 0
		if(float(self._last_object_type["transformation"]["x_offset"]) != 0):
			x_offset = float(self._last_object_type["transformation"]["x_offset"]) / float(self._abbe_three_points_matrix.ret_width_of_screen())

		y_offset = 0
		if(float(self._last_object_type["transformation"]["y_offset"]) != 0):
			y_offset = float(self._last_object_type["transformation"]["y_offset"]) / float(self._abbe_three_points_matrix.ret_height_of_screen())

		radians = float(self._last_object_pose["orientation_in_radians"])

		#only the offset is rotated using x/y as the origin
		pre_rotated_x = x_offset
		pre_rotated_y = y_offset

		rotMatrix = numpy.array([[numpy.cos(radians), -numpy.sin(radians)],
					      [numpy.sin(radians), numpy.cos(radians)]])

		pre_rotated_matrix = [[pre_rotated_x],[pre_rotated_y]]
		post_rotated_matrix = numpy.dot(rotMatrix,pre_rotated_matrix)


		return [float(post_rotated_matrix[0][0]) + x,float(post_rotated_matrix[1][0]) + y]


	def draw_collision_object_in_rviz(self):
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()

		p.pose.position.z = self.height_of_table() + (float(self._last_object_type["size"]["h"]) / 2)

		#determine point relative to robot's pose
		coordinates_of_last_object_with_offset = self.determine_center_of_object()
		tmp_pos = self._abbe_three_points_matrix.determine_a_relative_point(float(coordinates_of_last_object_with_offset[0]),float(coordinates_of_last_object_with_offset[1]))

		p.pose.position.x = tmp_pos[0]
		p.pose.position.y = tmp_pos[1]

		quaternion = tf.transformations.quaternion_from_euler(0,0,float(self._last_object_pose["orientation_in_radians"]))

		p.pose.orientation.x = quaternion[0]
		p.pose.orientation.y = quaternion[1]
		p.pose.orientation.z = quaternion[2]
		p.pose.orientation.w = quaternion[3]

		self.scene.add_box("object" + str(self._object_count),p,(float(self._last_object_type["size"]["l"]), float(self._last_object_type["size"]["w"]), float(self._last_object_type["size"]["h"]))) #0.72 ... is the size of the object
		self._object_count = self._object_count + 1

	def determine_object_pickup_pose(self):
		#offsets are based on the orientation of the object so calculate the correct center of object coordinates by using rotation matrix
		x = float(self._last_object_pose["x"])
		y = float(self._last_object_pose["y"])

		#The x & y coordinates are percentages of the screen width.  The offsets are in meters so we need to covert these to percents as well (of screen).
		x_offset = 0
		if(float(self._last_object_type["grasp"]["x_offset"]) != 0):
			x_offset = float(self._last_object_type["grasp"]["x_offset"]) / float(self._abbe_three_points_matrix.ret_width_of_screen())

		y_offset = 0
		if(float(self._last_object_type["grasp"]["y_offset"]) != 0):
			y_offset = float(self._last_object_type["grasp"]["y_offset"]) / float(self._abbe_three_points_matrix.ret_height_of_screen())

		radians = float(self._last_object_pose["orientation_in_radians"])

		#only the offset is rotated using x/y as the origin
		pre_rotated_x = x_offset
		pre_rotated_y = y_offset

		rotMatrix = numpy.array([[numpy.cos(radians), -numpy.sin(radians)],
					      [numpy.sin(radians), numpy.cos(radians)]])

		pre_rotated_matrix = [[pre_rotated_x],[pre_rotated_y]]
		post_rotated_matrix = numpy.dot(rotMatrix,pre_rotated_matrix)

		print "self._last_object_pose"
		print self._last_object_pose
		print "post_rotated_matrix"
		print post_rotated_matrix


		ret = [float(post_rotated_matrix[0][0]) + x,float(post_rotated_matrix[1][0])+y,self.height_of_table() + self._last_object_type["grasp"]["z_offset"],self._last_object_type["grasp"]["yaw"]]
		print "Change in X: "
		print str(float(ret[0]) - float(self._last_object_pose["x"]))
		print "Change in Y:"
		print str(float(ret[1]) - float(self._last_object_pose["y"]))
		return ret

	def determine_object_rfid_pose(self,x,y,orientation_in_radians):
		#offsets are based on the orientation of the object so calculate the correct center of object coordinates by using rotation matrix
		x = float(x)
		y = float(y)

		#in meters below but they'll be converted to %s
		x_offset = 0.15 #0.08
		y_offset = 0.0

		#The x & y coordinates are percentages of the screen width.  The offsets are in meters so we need to covert these to percents as well (of screen).
		if(x_offset != 0):
			print "screen width: " + str(self._abbe_three_points_matrix.ret_width_of_screen())
			x_offset = float(x_offset) / float(self._abbe_three_points_matrix.ret_width_of_screen())
			print "x offset: " + str(x_offset)

		if(y_offset != 0):
			y_offset = float(y_offset) / float(self._abbe_three_points_matrix.ret_height_of_screen())

		radians = float(orientation_in_radians)

		#only the offset is rotated using x/y as the origin
		pre_rotated_x = x_offset
		pre_rotated_y = y_offset

		rotMatrix = numpy.array([[numpy.cos(radians), -numpy.sin(radians)],
					      [numpy.sin(radians), numpy.cos(radians)]])

		pre_rotated_matrix = [[pre_rotated_x],[pre_rotated_y]]
		post_rotated_matrix = numpy.dot(rotMatrix,pre_rotated_matrix)


		return [float(post_rotated_matrix[0][0]) + x,float(post_rotated_matrix[1][0]) + y]


	def pickup_object(self):
		self._head.set_pan(0.7) #point to left arm

		pickup_pose = self.determine_object_pickup_pose()

		#TODO comment the following test
		'''
		self.go_to_relative_position(float(self._last_object_pose['x']),float(self._last_object_pose['y']),False, True)
		rospy.sleep(5)
		self.go_to_relative_position(float(pickup_pose[0]),float(pickup_pose[1]),False, True) #move left arm to pickup location
		rospy.sleep(5)
		return True
		'''

		self._grippers.open(True) #open left gripper
		self.go_to_relative_position(float(pickup_pose[0]),float(pickup_pose[1]),False, True) #move left arm to pickup location
		self._drop_left_arm_to_pickup_height(self._abbe_three_points_matrix.calc_relative_radians_angle(self._last_object_pose["orientation_in_radians"]) + pickup_pose[3] ,pickup_pose[2])
		self._grippers.close(True)
		self.scene.remove_world_object("object" + str(self._object_count - 1)) #remove object from rviz
		self._ik.set_left(float(self._drop_off_cord_x),float(self._drop_off_cord_y),float(self._drop_off_cord_z)) #just above drop off point
		self._grippers.open(True)


	def _objectapi(self):
		url = self._server + "/objectapi/?objectapi_id=" + str(self._last_rfid_value())
		response = urllib.urlopen(url)
		data = json.loads(response.read())
		#open left gripper
		self._last_object_type = data


	def _last_rfid_value(self):
		#TODO change this
		return "C66A1190-87D7-4C98-A7EC-C509FEE39C8B"

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

	def move_right_arm_up_with_same_radians_and_xy(self,radians_for_rfid = 0):
		pose = self._ik.get_pose('right')
		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self._default_height(),0,radians_for_rfid):
			print "right failed to go up (move_right_arm_up_with_same_radians_and_xy)"

	def _point_rfid_reader_down_on_right_arm(self,radians_for_rfid = 0):

		pose = self._ik.get_pose('right')

		#to avoid colis turn prior to dropping
		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),float(pose.z),0,radians_for_rfid):
			print "failed to turn prior to going down"

		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self.height_of_table() + self._z_offset_above_table_to_scan_rfid_at,0,radians_for_rfid):
			print "right failed to point rfid down at this radians... should try opposite here with correct offsets: " + str(radians_for_rfid)
			if radians_for_rfid >= math.pi:
				#TODO offset for distance between rfid scanner when rotated
				if self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self.height_of_table() + self._z_offset_above_table_to_scan_rfid_at,0,radians_for_rfid - math.pi):
					print "opposite orientation success"
					return True
				print "opposite failed as well"
			return False
		#print "RFID is down"
		return True

	def _drop_left_arm_to_pickup_height(self, orient_radians, z_value_for_pickup):
		pose = self._ik.get_pose('left')
		#TODO orientation

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians):
			print "failed to turn prior to going down: " + str(orient_radians)
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians - math.pi):
				print "failed to turn prior to going down opposite: " + str(orient_radians - math.pi)

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),z_value_for_pickup,0,orient_radians):
			print "Left arm failed to drop"
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),z_value_for_pickup,0,orient_radians - math.pi):
				print "Left arm failed to drop"

	def _get_right_arm_out_of_the_way(self):
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
