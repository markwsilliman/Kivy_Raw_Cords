#!/usr/bin/env python
#
# Converts the coordinates of the screen to coordinates of the robot's end effectors
#
#                      - 2
#                      -
#                      -
#----------------------- 1
#0

import math
import numpy
import pickle
import os.path
import sys

class Abbe_Three_Points_To_Rot_Matrix(object):
	_cords = []
	_pos = 0
	_slope_in_radians = False
	_rotMatrix = False
	_length_of_x_slope = False
	_length_of_y_slope = False
	_print_test_output = False
	_origin_x = False
	_origin_y = False

	def __init__(self):
		# To start off the robot must know where three of the corners of the screen are.  This takes some time and must be done with laser pointers.  Therefore it saves a config file with the configuration after you do it once.  After this config file is created it'll load those settings so you don't need to repeat the process.  As long as you haven't moved the robot or the screen this will work.
		self._import_config_file_if_exists()

	def add_cord(self,x,y):
		if len(self._cords) == 0:
			#intentionally backwards because it's relative to screen.  Not robot
			self._origin_x = float(y)
			self._origin_y = float(x)
	
		if len(self._cords) > 3:
			if(self._print_test_output):
				print "3 cords already exist"
			return False

		self._cords.insert(self._pos,[float(x),float(y)])
		self._pos = self._pos + 1

		if(self._print_test_output):
			print "cord added x: " + str(x) + " y: " + str(y)

		if len(self._cords) == 3:
			#after 3rd point is added calc rot matrix
			self._save_three_cords_to_config_file()
			self.calc_cord_positions()

	def _config_file_path(self):
		return os.path.dirname(os.path.realpath(__file__)) + "/threepoints.config"

	def does_config_file_exist(self):
		if(os.path.isfile(self._config_file_path())):
			return True
		return False

	def _import_config_file_if_exists(self):
		#does the config file exist
		if(os.path.isfile(self._config_file_path())):
			with open(self._config_file_path(), 'rb') as input:
				self._cords = pickle.load(input)
				print "Config file imported"
				self._origin_x = self._cords[0][1]
				self._origin_y = self._cords[0][0]
				self.calc_cord_positions()

	def _save_three_cords_to_config_file(self):
		with open(self._config_file_path(),"w+") as output:
			pickle.dump(self._cords, output, pickle.HIGHEST_PROTOCOL)
		print "Config file saved.  Please exit this program and restart it (next time it'll load the .config file and disable TkInter)"
		sys.exit()

	def qc_cords(self):
		#determine angle
		if len(self._cords) != 3:
			if(self._print_test_output):
				print "you don't have 3 cords yet"
			return False

		return True

	def calc_relative_radians_angle(self,rad):
		#TODO haven't tested if the slope_in_radians should be added or subtracted
		return (float(self._slope_in_radians) + float(rad))

	def calc_cord_positions(self):
		if not self.qc_cords():
			return False

		#y is perp to robot.  x is away so reverse these for relative
		#y gets greater moving to left. x gets greater moving away
		_tmp_change_in_x = self._cords[0][1] - self._cords[1][1] #farther left is positive
		_tmp_change_in_y = self._cords[1][0] - self._cords[0][0]

		if _tmp_change_in_x <= 0.005:
			if(self._print_test_output):
				print "X didn't change sufficiantly: " + str(_tmp_change_in_x)
			return False

		if(self._print_test_output):
			print "change in x: " + str(_tmp_change_in_x)
			print "change in y: " + str(_tmp_change_in_y)

		#determine angle of slope and rotational matrix
		_tmp_slope = _tmp_change_in_y / _tmp_change_in_x
		self._slope_in_radians = math.atan(_tmp_slope)
		if(self._print_test_output):
			print "degrees: " + str(math.degrees(self._slope_in_radians))

		self._rotMatrix = numpy.array([[numpy.cos(self._slope_in_radians), -numpy.sin(self._slope_in_radians)],
					      [numpy.sin(self._slope_in_radians), numpy.cos(self._slope_in_radians)]])
		
		if(self._print_test_output):
			print self._rotMatrix

		#determine length of x-slope
		self._length_of_x_slope = math.sqrt(math.pow(_tmp_change_in_x,2) + math.pow(_tmp_change_in_y,2))

		if(self._print_test_output):
			print "length of x slope: " + str(self._length_of_x_slope)

		self._calc_cord_positions_for_y_slope()

	
	def _calc_cord_positions_for_y_slope(self):
		if not self.qc_cords():
			return False

		self._length_of_y_slope = math.sqrt(math.pow(self._cords[1][0] - self._cords[2][0],2) + math.pow(self._cords[1][1] - self._cords[2][1],2))
		if(self._print_test_output):
			print "length of y slope: " + str(self._length_of_y_slope)
	
	def determine_a_relative_point(self,x_per,y_per):
		if(x_per < 0 or x_per > 1 or y_per < 0 or y_per > 1):
			if(self._print_test_output):
				print "passed relative percents invalid"
			return False

		if not self.qc_cords():
			return False

		#relative to screen!
		pre_rotated_x = float(x_per) * float(self._length_of_x_slope)
		#1 - y_per is because right is positive
		#TODO x & y are reversed?
		pre_rotated_y = float(y_per) * float(self._length_of_y_slope)

		if(self._print_test_output):
			print "Pre Rotated X"
			print pre_rotated_x
			print "Pre Rotated Y"
			print pre_rotated_y

		pre_rotated_matrix = [[pre_rotated_x],[pre_rotated_y]]
		post_rotated_matrix = numpy.dot(self._rotMatrix,pre_rotated_matrix)
		if(self._print_test_output):
			print "Post Rotated X"
			print str(round(post_rotated_matrix[0][0],3))		
			print "Post Rotated Y"
			print str(round(post_rotated_matrix[1][0],3))			

		##_origin_x
		post_rotated_matrix_adjust_with_origin_values = [post_rotated_matrix[1][0]+self._origin_y, self._origin_x - post_rotated_matrix[0][0]] # self._origin_x - is because origin x is max value (of robot's y)
		return post_rotated_matrix_adjust_with_origin_values
