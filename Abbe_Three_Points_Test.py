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

class Abbe_Three_Points_Test(object):
	_abbe_three_points_matrix = False
	_ik = False
	_tkinter = False
	_move_left_arm = True
	_move_arm_offset = 0.1

	def __init__(self):
		self._tkinter = tk.Tk()
		self._tkinter.geometry('300x200')
		self._tkinter.bind('<KeyPress>', self._onKeypress)		

		self._ik = Abbe_IK()
		self._abbe_three_points_matrix = Abbe_Three_Points_To_Rot_Matrix()
		self._navigator_io = baxter_interface.Navigator('left')
		self._navigator_io_r = baxter_interface.Navigator('right')
		self._navigator_io.button0_changed.connect(self._savepoint_button_was_pressed_left)
		self._navigator_io_r.button0_changed.connect(self._savepoint_button_was_pressed_right)
		self._navigator_io.button1_changed.connect(self._button1_was_pressed)

		#mainloop must be at bottom of init
		self._tkinter.mainloop()

	def _onKeypress(self,event):
		_c = event.char
		
		#changing speed?
		if _c == 't':
			self._move_arm_offset = 0.1
		if _c == 'g':
			self._move_arm_offset = 0.03
		if _c == 'b':
			self._move_arm_offset = 0.01

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
		

		if(self._move_left_arm):
			if not self._ik.set_left(float(x),float(y),float(pose.z)):
				print "left failed to point down at pose"
			else:
				print "success"
		else:
			if not self._ik.set_right(float(x),float(y),float(pose.z)):
				print "right failed to point down at pose"	
			else:
				print "success"
		

	def _button1_was_pressed(self, value):
		if value:
			_tmp = 0
			while(_tmp <= 1):
				self.go_to_relative_position(0,_tmp)
				_tmp = _tmp + 0.05
			_tmp = 0
			while(_tmp <= 1):
				self.go_to_relative_position(_tmp,1)
				_tmp = _tmp + 0.05
			_tmp = 1
			while(_tmp >= 0):
				self.go_to_relative_position(1,_tmp)
				_tmp = _tmp - 0.05
			_tmp = 1
			while(_tmp >= 0):
				self.go_to_relative_position(_tmp,0)
				_tmp = _tmp - 0.05
			

	def _savepoint_button_was_pressed_left(self, value):
		if value:
			self._move_left_arm = False
			_pos = self._ik.get_pose('left')
			print "saved"
			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)

	def _savepoint_button_was_pressed_right(self, value):
		if value:
			_pos = self._ik.get_pose('right')
			print "saved"
			self._abbe_three_points_matrix.add_cord(_pos.x,_pos.y)

	def go_to_relative_position(self,x_per,y_per):
		pos = self._abbe_three_points_matrix.determine_a_relative_point(x_per,y_per)
		return self._go_to_position(pos[0],pos[1],x_per)

	def _go_to_position(self,_tmp_x,_tmp_y,x_per):
		_prioritize_left_hand = True
		print "x_per is: " + str(x_per)

		if(x_per > 0.5):
			_prioritize_left_hand = False
		print "going to " + str(_tmp_x) + " " + str(_tmp_y)
		if _prioritize_left_hand:
			print "priority left due to " + str(x_per)
			if(not self._ik.set_left(_tmp_x,_tmp_y,0.0)):
				print "left failed trying right..."
				if(not self._ik.set_right(_tmp_x,_tmp_y,0.0)):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False
		else:
			print "priority right due to " + str(x_per)
			if(not self._ik.set_right(_tmp_x,_tmp_y,0.0)):
				print "right failed trying left..."
				if(not self._ik.set_left(_tmp_x,_tmp_y,0.0)):
					print "neither arm code reach position: x:" + str(_tmp_x) + " y: " + str(_tmp_y)
					return False		

		return True

if __name__ == '__main__':
    rospy.init_node('Abbe_Three_Points_Test', anonymous=True)
    at = Abbe_Three_Points_Test()
    rospy.spin()
