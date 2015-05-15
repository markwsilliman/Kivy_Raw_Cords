#!/usr/bin/env python

#                      - 2
#                      -
#                      -
#----------------------- 1
#0

import math
import numpy

class Abbe_Three_Points_To_Rot_Matrix(object):
	_cords = []
	_pos = 0
	_slope_in_radians = False
	_rotMatrix = False
	_length_of_x_slope = False
	_length_of_y_slope = False
	_print_test_output = True
	_origin_x = False
	_origin_y = False

	def __init__(self):
		pass

	def add_cord(self,x,y):
		if len(self._cords) == 0:
			self._origin_x = float(x)
			self._origin_y = float(y)
	
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
			self.calc_cord_positions()

	def qc_cords(self):
		#determine angle
		if len(self._cords) != 3:
			if(self._print_test_output):
				print "you don't have 3 cords yet"
			return False
		
		if self._cords[0][0] >= self._cords[1][0]:
			if(self._print_test_output):
				print "x values of point 0 & 1 are inverse order"
			return False
		
		if self._cords[2][1] <= self._cords[1][1]:
			if(self._print_test_output):
				print "y values of point 1 & 2 are inverse order"
			return False

		return True
			

	def calc_cord_positions(self):
		if not self.qc_cords():
			return False

		_tmp_change_in_x = self._cords[1][0] - self._cords[0][0]
		_tmp_change_in_y = self._cords[1][1] - self._cords[0][1]

		if _tmp_change_in_x <= 0.05:
			if(self._print_test_output):
				print "X didn't change sufficiantly"
			return False

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

		pre_rotated_x = float(x_per) * float(self._length_of_x_slope)
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
		post_rotated_matrix_adjust_with_origin_values = [post_rotated_matrix[0][0] + self._origin_x,post_rotated_matrix[1][0]+self._origin_y]
		return post_rotated_matrix_adjust_with_origin_values
