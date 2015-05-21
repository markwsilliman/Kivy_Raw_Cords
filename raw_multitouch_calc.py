#!/usr/bin/env python

import json
import string
import math

class Raw_Multitouch_Calc(object):
	_pos1 = False
	_pos2 = False
	_pos3 = False

	def test_last_three_points(self):
		with open('kivy_touch_down_multitouch_log.json', 'r') as outfile:
			_tmp_list = list(outfile)
			self._pos1 = self.raw_multitouch_calc_to_point(str(_tmp_list[-1]))
			self._pos2 = self.raw_multitouch_calc_to_point(str(_tmp_list[-2]))
			self._pos3 = self.raw_multitouch_calc_to_point(str(_tmp_list[-3]))
			print(self._pos1)
			print(self._pos2)
			print(self._pos3)
			self._calc_position_and_orientation()
	
	def _calc_position_and_orientation(self):
		dist1 = self._dist(self._pos1,self._pos2)
		dist2 = self._dist(self._pos2,self._pos3)
		dist3 = self._dist(self._pos3,self._pos1)

		print dist1
		print dist2
		print dist3
		
	def _dist(self,pos,posb):
		return math.sqrt(math.pow(self._diff(pos[0],posb[0]),2) + math.pow(self._diff(pos[1],posb[1]),2))

	def _diff(self,a,b):
		return float(abs(float(a) - float(b)))
	
	def raw_multitouch_calc_to_point(self,temp_points):
		temp_points = temp_points.rstrip()
		temp_points = temp_points.split(",")
		return temp_points

if __name__ == '__main__':
    at = Raw_Multitouch_Calc()
    at.test_last_three_points()
