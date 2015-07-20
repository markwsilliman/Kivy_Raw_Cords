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
	#CHANGE THIS TO YOUR SERVER'S DOMAIN
	_server = "http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com"

	#All of the following units are meters

	#Custom values to offset x & y coordinates in meters - this makes it easier to fix slightly inaccurate config files
	_screen_x_offset = -0.003 #if your X coordinates are slightly off (RELATIVE TO SCREEN; NOT ROBOT!) tweak this
	_screen_y_offset = -0.021 #if your Y coordinates are slightly off (RELATIVE TO SCREEN; NOT ROBOT!) tweak this
	_height_of_rfid_scanner = 0.075 #What is the height of your RFID reader (only measure the distance below the gripper)
	_z_height_of_gripper = 0.1 #Height of gripper.  Change this if you aren't using the stock grippers
	_height_of_table = -0.175 #Note: This is the relative height of the table compared to the bottom of Baxter (excluding the stand!).  Mine is negative because the table is lower than than the base of baxter (regardless of the stand & wheels which are lower than the table but don't influence the math.).

	#The following 3 values control where objects should be dropped off by the left arm
	_drop_off_cord_x = 0.5
	_drop_off_cord_y = 0.5
	_drop_off_cord_z = 0.1

	#Review function self.draw_table_in_rviz for more custom values related to the dimensions / pose of your table

	#You shouldn't need to modify any of the following defaults
	_objects_on_table = []
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
		#Setup inverse kinematics
		self._ik = Abbe_IK()
		#Emotions are exclusively so kids think the robot is alive...
		self._abbe_emotions = Abbe_Emotions()

		#enable the grippers
		self._grippers = Abbe_Gripper()
		#enable the head (only for kids enterainment)
		self._head = baxter_interface.Head()

		#enable move it
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		#if you don't sleep for a couple seconds you'll get errors that the scene or robot doesn't exist
		print "sleeping for 2 seconds for moveit scene"
		rospy.sleep(2)

		#add the table to rviz
		self.draw_table_in_rviz()
		#abbe_three_points_matrix controls the transformations of poses on the screen to poses relative to the robot.  Keep in mind that the screen is almost never perfectly parallel to the robot's pose.  Rotation matrix is used to compensate.  The transposition is calculated once by move the grippers to 3 corners of the screen.  Then a config file (threepoints.config) is created.  This config file is valid as long as you don't move the table or robot.
		self._abbe_three_points_matrix = Abbe_Three_Points_To_Rot_Matrix()
		#check if threepoints.config exists.  If no, prompt the user to calibrate the screen's pose.
		self._require_configuration()
		#note: if the config file doesn't exist yet the init() script will never get past this point

		while not rospy.is_shutdown():
			#check if a new object has been added to the table
			self._check_for_new_object_on_table()
			time.sleep(5)

	def get_json_from_server(self,script_url):
		#download information from the API
		url = self._server + "/" + script_url
		response = urllib.urlopen(url)
		#convert json to python datatype
		return json.loads(response.read())

	def turn_head(self,right):
		#which direction should we turn the head?
		if right:
			self._head.set_pan(-0.7)
		else:
			self._head.set_pan(0.7)

	def _check_for_new_object_on_table(self):
		#ask the server if a new object exists
		data = self.get_json_from_server("touch_json_last_object.php")

		#False = no valid 3 point pose found
		if data == False:
			return False

		self._last_object_pose = data
		#_screen_x_offset & _screen_y_offset are manually set above and allow you to tweak an imperfect calibration
		data["y"] = float(data["y"]) + float(self._screen_y_offset / self._abbe_three_points_matrix.ret_height_of_screen())
		data["x"] = float(data["x"]) + float(self._screen_x_offset / self._abbe_three_points_matrix.ret_width_of_screen())

		pose = [data["x"],data["y"]]
		#Was this pose already imported?  If yes, ignore it.
		if pose in self._object_already_added_to_moveit:
			print "check for new object: already imported"
		else:
			#prevent duplicates by adding pose to already completed list
			self._object_already_added_to_moveit.append(pose)

			#Step 1: read the RFID

			#turn head towards right arm
			self.turn_head(True)

			#determine the relative pose of the RFID.
			rfid_pose = self.determine_object_rfid_pose(data["x"],data["y"],data["orientation_in_radians"])
			#go to pose (but way above table)
			self.go_to_relative_position(float(rfid_pose[0]),float(rfid_pose[1]),True)
			#turn to the right orientation and drop to RFID reading height
			self._point_rfid_reader_down_on_right_arm(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))

			#Detect everything there is to know about the object from the API
			self._objectapi()

			print "Adding a new " + str(self._last_object_type["nickname"]) + " to RVIZ"

			#Draw the object in RVIZ
			self.draw_collision_object_in_rviz()

			#Move the right arm out of the way (2 steps)
			#First move it up without rotation to avoid collision with object
			self.move_right_arm_up_with_same_radians_and_xy(self._abbe_three_points_matrix.calc_relative_radians_angle(data["orientation_in_radians"]))
			#Now that we're above the table get the arm out of the way
			self._get_right_arm_out_of_the_way()

			#Pickup Object
			#TODO implement pickup object
			#if self._object_count > 0:
			#	self.pickup_object(self._object_count - 1)

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
		#draw the table in rviz.  You'll likely need to tweak the following values for your table.

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.28 + (0.72/2) #0.28 is offset from center of robot to the front of the table.  0.72/2 is the distance to the center of the table
		p.pose.position.y = 0
		p.pose.position.z = self.height_of_table()  #0.791 is the Z value of the table (not relative to the robot like self.height_of_table returns)

		quaternion = tf.transformations.quaternion_from_euler(0,0,float(math.pi / 2)) #turn table 90 degrees

		p.pose.orientation.x = quaternion[0]
		p.pose.orientation.y = quaternion[1]
		p.pose.orientation.z = quaternion[2]
		p.pose.orientation.w = quaternion[3]

		self.scene.add_mesh("table",p,"mesh/table.stl")

		#TODO remove all of the following!
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.28 + (0.72/2)
		p.pose.position.y = 0
		p.pose.position.z = self.height_of_table() + (0.107 / 2.0) #half the height of the pot!
		#self.scene.add_mesh("pot",p,"mesh/pot.stl") #TODO copy the pot logic for objects

	def height_of_table(self):
		return self._height_of_table

	def _onKeypress(self,event):
		_c = event.char
		
		#when calibrating it's convinient to be able to change the distance each arm moves at any given time (t being the greatest and y being the smallest)
		if _c == 't':
			self._move_arm_offset = 0.1
		if _c == 'g':
			self._move_arm_offset = 0.03
		if _c == 'b':
			self._move_arm_offset = 0.01
		if _c == 'y':
			self._move_arm_offset = 0.003
		#m saves a point
		if _c == 'm':
			self.save_point()

		#movement of either are is done with WASD like a video game
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
				print "success left arm"
		else:
			if not self._ik.set_right(float(x),float(y),float(self.height_of_table()) + float(self._z_height_of_gripper) + float(offset_of_gripper_above_table)):
				print "right failed to point down at pose"	
			else:
				print "success right arm"
		
		self._currently_moving = False

	def save_point(self):
		#save the current pose when calibrating
		if self._move_left_arm:
			self._move_left_arm = False
			_pos = self._ik.get_pose('left')
			print "0,0 (bottom, left) saved"
			print "Move the right arm to 1,0 (bottom,right) and press m to save that point."
			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)			
		else:
			_pos = self._ik.get_pose('right')
			print "saved"
			if self._abbe_three_points_matrix.count_cords() == 2:
				print "Finally move the right arm to 1,1 (top,right) and press m to complete the calibration."

			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)

	def go_to_relative_position(self,x_per,y_per, force_right = False, force_left = False):
		#This function is required because the screen only knows the coordinates of the object relative to the screen.  Not relative to the robot.  This converts from the screen percentages to a robot pose.
		pos = self._abbe_three_points_matrix.determine_a_relative_point(x_per,y_per)
		return self._go_to_position(pos[0],pos[1],x_per,force_right,force_left)

	def determine_center_of_object(self):
		#This function is required for adding objects to RVIZ.  In practice it isn't convinient to add the 3 points directly in the middle of each object.  x_offset & y_offset allow you to add it any location and then offset (via the API) the location for the collision box & mesh.
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

		#add this to an array of objects
		self._objects_on_table.append([self._object_count,self._last_object_type,self._last_object_pose,0]) #final ,0 means that the object hasn't been removed from the table

		self.scene.add_box("object" + str(self._object_count),p,(float(self._last_object_type["size"]["l"]), float(self._last_object_type["size"]["w"]), float(self._last_object_type["size"]["h"]))) #0.72 ... is the size of the object
		self._object_count = self._object_count + 1

	def determine_object_pickup_pose(self,object_id):
		#offsets are based on the orientation of the object so calculate the correct center of object coordinates by using rotation matrix
		x = float(self._objects_on_table[object_id][2]["x"])
		y = float(self._objects_on_table[object_id][2]["x"])

		#The x & y coordinates are percentages of the screen width.  The offsets are in meters so we need to covert these to percents as well (of screen).
		x_offset = 0
		if(float(self._objects_on_table[object_id][1]["grasp"]["x_offset"]) != 0):
			x_offset = float(self._objects_on_table[object_id][1]["grasp"]["x_offset"]) / float(self._abbe_three_points_matrix.ret_width_of_screen())

		y_offset = 0
		if(float(self._objects_on_table[object_id][1]["grasp"]["y_offset"]) != 0):
			y_offset = float(self._objects_on_table[object_id][1]["grasp"]["y_offset"]) / float(self._abbe_three_points_matrix.ret_height_of_screen())

		radians = float(self._objects_on_table[object_id][2]["orientation_in_radians"])

		#only the offset is rotated using x/y as the origin
		pre_rotated_x = x_offset
		pre_rotated_y = y_offset

		rotMatrix = numpy.array([[numpy.cos(radians), -numpy.sin(radians)],
					      [numpy.sin(radians), numpy.cos(radians)]])

		pre_rotated_matrix = [[pre_rotated_x],[pre_rotated_y]]
		post_rotated_matrix = numpy.dot(rotMatrix,pre_rotated_matrix)



		ret = [float(post_rotated_matrix[0][0]) + x,float(post_rotated_matrix[1][0])+y,self.height_of_table() + self._objects_on_table[object_id][1]["grasp"]["z_offset"],self._objects_on_table[object_id][1]["grasp"]["yaw"]]

		return ret

	def determine_object_rfid_pose(self,x,y,orientation_in_radians):
		#offsets are based on the orientation of the object so calculate the correct center of object coordinates by using rotation matrix
		x = float(x)
		y = float(y)

		#When an object is placed on the table we only know it's location and orientation.  Not it's identity.  Therefore all RFIDs must be on the same relative position of each object (relative to 3 points)
		#in meters below but they'll be converted to %s
		x_offset = 0.15
		y_offset = 0.0

		#The x & y coordinates are percentages of the screen width.  The offsets are in meters so we need to covert these to percents as well (of screen).
		if(x_offset != 0):
			x_offset = float(x_offset) / float(self._abbe_three_points_matrix.ret_width_of_screen())

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


	def pickup_object(self,object_id):
		self.turn_head(False) #point to left arm

		pickup_pose = self.determine_object_pickup_pose(object_id)

		self._grippers.open(True) #open left gripper
		self.go_to_relative_position(float(pickup_pose[0]),float(pickup_pose[1]),False, True) #move left arm to pickup location
		self._drop_left_arm_to_pickup_height(self._abbe_three_points_matrix.calc_relative_radians_angle(self._objects_on_table[object_id][2]["orientation_in_radians"]) + pickup_pose[3] ,pickup_pose[2])
		self._grippers.close(True)
		self.scene.remove_world_object("object" + str(self._objects_on_table[object_id][0])) #remove object from rviz
		self._objects_on_table[object_id][3] = 1 #mark object as deleted
		self._ik.set_left(float(self._drop_off_cord_x),float(self._drop_off_cord_y),float(self._drop_off_cord_z)) #just above drop off point
		self._grippers.open(True)


	def _objectapi(self):
		self._last_object_type = self.get_json_from_server("/objectapi/?objectapi_id=" + str(self._last_rfid_value()))


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

		if _prioritize_left_hand:
			if(not self._ik.set_left(_tmp_x,_tmp_y,self._default_height())):
				print "left failed trying right..."
				if force_left:
					print "right hand prohibited"
					return False
				if(not self._ik.set_right(_tmp_x,_tmp_y,self._default_height())):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False
		else:
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
		#height used to get the hands out of the way etc.
		return float(self.height_of_table()) + float(0.375)

	def move_right_arm_up_with_same_radians_and_xy(self,radians_for_rfid = 0):
		#used to get RFID out of the way without rotating or moving (X/Y) the gripper
		pose = self._ik.get_pose('right')
		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self._default_height(),0,radians_for_rfid):
			print "right failed to go up (move_right_arm_up_with_same_radians_and_xy)"

	def _point_rfid_reader_down_on_right_arm(self,radians_for_rfid = 0):
		#used to drop the RFID reader near the table height to read the RFID's value
		pose = self._ik.get_pose('right')

		#to avoid colis turn prior to dropping
		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),float(pose.z),0,radians_for_rfid):
			print "failed to turn prior to going down"

		if not self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self.height_of_table() + self._z_height_of_gripper + self._height_of_rfid_scanner,0,radians_for_rfid):
			print "right failed to point rfid down at this radians... should try opposite here with correct offsets: " + str(radians_for_rfid)
			if radians_for_rfid >= math.pi:
				if self._ik.set_right_rfid_down(float(pose.x),float(pose.y),self.height_of_table() + self._z_height_of_gripper + self._height_of_rfid_scanner,0,radians_for_rfid - math.pi):
					print "opposite orientation success"
					return True
				print "opposite failed as well"
			return False
		#print "RFID is down"
		return True

	def _drop_left_arm_to_pickup_height(self, orient_radians, z_value_for_pickup):
		#drops the left arm to the pickup height required to pickup the object (defined in the API)
		pose = self._ik.get_pose('left')

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians):
			print "failed to turn prior to going down: " + str(orient_radians)
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),float(pose.z),0,orient_radians - math.pi):
				print "failed to turn prior to going down opposite: " + str(orient_radians - math.pi)

		if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),z_value_for_pickup,0,orient_radians):
			print "Left arm failed to drop"
			if not self._ik.set_left_down_for_pickup(float(pose.x),float(pose.y),z_value_for_pickup,0,orient_radians - math.pi):
				print "Left arm failed to drop"

	def _get_right_arm_out_of_the_way(self):
		#primarily so the left arm won't have problems with the left arm
		if not self._ik.set_right(float(0.5),float(-0.5),self._default_height()):
			print "couldnt go out of the way first"

	def _point_arms_straight_down(self):
		#End effectors must point down for calibration (so laser pointers are accurate)
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
