#!/usr/bin/env python

import math
import random
import rospy
import argparse
import struct
import sys
from abbe_ik import Abbe_IK
import threading
import Tkinter as tk
import json
import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range
from Abbe_Three_Points_To_Rot_Matrix import Abbe_Three_Points_To_Rot_Matrix

class Abbe_Table_Sync(object):
	_abbe_three_points_matrix = False
	_ik = False
	_tkinter = False
	_move_left_arm = True
	_move_arm_offset = 0.1
	_last_move_to_file_x = False
	_last_move_to_file_y = False
	_currently_moving = False

	def __init__(self):
		self._point_arms_straight_down()
		print "Opening TkInter window to capture key strokes.  Make sure it stays in focus."
		self._tkinter = tk.Tk()
		#Size doesn't matter
		self._tkinter.geometry('300x200')
		#On each keypress call _onKeypress function
		self._tkinter.bind('<KeyPress>', self._onKeypress)
		print "Moving left arm first.  Set to orientation 0,0 on screen.";
		print "Movement Speed... T: fast, G: Medium, B: Slow";
		print "Move with W,A,S,D like an old school video game";
		print "Save Pose... M";

		self._ik = Abbe_IK()
		self._abbe_three_points_matrix = Abbe_Three_Points_To_Rot_Matrix()
		self._navigator_io = baxter_interface.Navigator('left')
		self._navigator_io_r = baxter_interface.Navigator('right')

		#mainloop must be at bottom of init
		self._tkinter.mainloop()

		while not rospy.is_shutdown():
			self.go_to_position_in_file()
			time.sleep(1)

	def _onKeypress(self,event):
		_c = event.char
		
		#changing speed?
		if _c == 't':
			self._move_arm_offset = 0.1
		if _c == 'g':
			self._move_arm_offset = 0.03
		if _c == 'b':
			self._move_arm_offset = 0.01
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

	def go_to_relative_position(self,x_per,y_per):
		pos = self._abbe_three_points_matrix.determine_a_relative_point(x_per,y_per)
		return self._go_to_position(pos[0],pos[1],x_per)

	def go_to_position_in_file(self):
		with open('kivy_touch_down_log.json') as data_file: 
			data = json.load(data_file)
			#has the position changed?
			if(self._last_move_to_file_x != data[0] or self._last_move_to_file_y != data[1]):
				self._last_move_to_file_x = data[0]
				self._last_move_to_file_y = data[1]
				self.go_to_relative_position(float(data[0]),float(data[1]))

	def _go_to_position(self,_tmp_x,_tmp_y,x_per):
		_prioritize_left_hand = True
		#print "x_per is: " + str(x_per)

		if(x_per > 0.5):
			_prioritize_left_hand = False

		#print "going to " + str(_tmp_x) + " " + str(_tmp_y)
		if _prioritize_left_hand:
			#print "priority left due to " + str(x_per)
			if(not self._ik.set_left(_tmp_x,_tmp_y,self._default_height())):
				print "left failed trying right..."
				if(not self._ik.set_right(_tmp_x,_tmp_y,self._default_height())):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False
		else:
			#print "priority right due to " + str(x_per)
			if(not self._ik.set_right(_tmp_x,_tmp_y,self._default_height())):
				print "right failed trying left..."
				if(not self._ik.set_left(_tmp_x,_tmp_y,self._default_height())):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False		

		return True

	def _default_height(self):
		return 0.2

	def _point_arms_straight_down(self):
		n = Abbe_IK()
		print "End effectors must point down for calibration... sleeping for 2 seconds to make sure IK is live"
		time.sleep(2)
		pose = n.get_pose('left')

		if not n.set_left(float(pose.x),float(pose.y),self._default_height()):
			print "left failed to point down at pose"
			exit()

		pose = n.get_pose('right')

		if not n.set_right(float(pose.x),float(pose.y),self._default_height()):
			print "right failed to point down at pose"
			exit()

		print "ready for calibration"

if __name__ == '__main__':
    rospy.init_node('Abbe_Table_Sync', anonymous=True)
    at = Abbe_Table_Sync()
    rospy.spin()
