#!/usr/bin/env python

#
# Controls what images should be shown on Abbe's face
#

import os
import sys
import argparse
import rospy
import cv2
import cv_bridge
from abbe_errors import Abbe_Error
import threading
import time
import random

from sensor_msgs.msg import (
	Image,
)

class Abbe_Emotions(object):

	def __init__(self):
		self._eye_direction = "center" #default eye direction
		self._emotion = "awake" #default emotion

		self._error = Abbe_Error() #for throwing excpetions
		self.schedule_blinking() #schedule blinking via threads

		self.emotion("awake") #default emotion by calling funtion so all defaults are set


	def schedule_blinking(self):
		if(not rospy.is_shutdown()):
			self._blink = threading.Timer(1.0, self.blink)
			self._blink.start()

	def blink(self):
		if random.randint(1,10) <= 2: #20% chance of blinking every second
			self.send_image(self.image_static_path("blink.png"))
			time.sleep(0.15)
			self.send_image(self._current_img_path)

		#reschedule
		self.schedule_blinking()

	def emotion_is_valid(self,emotion_val):
		valid_emotions = ["awake","confused","happy","asleep"]
		if(emotion_val in valid_emotions):
			return True
		return False

	def eyedirection_is_valid(self,eyedirection_val):
		valid_eyedirections = ["center","left","right"]
		if(eyedirection_val in valid_eyedirections):
			return True
		return False

	def eye_direction(self,eye_direction):
		if(not self.eyedirection_is_valid(eye_direction)):
			self._error.error("unknown eye direction: " + str(eye_direction))

		self._eye_direction = eye_direction
		self.update_image_to_reflect_emotion_and_eye_direction()

	def update_image_to_reflect_emotion_and_eye_direction(self):
		self._local_img_file = "asleep"
		if(self._emotion == "awake" or self._emotion == "happy" or self._emotion == "confused"):
			self._local_img_file = str(self._emotion) + "_" + str(self._eye_direction)

		self._current_img_path = self.image_static_path(self._local_img_file + ".png")
		self.send_image(self._current_img_path)


	def emotion(self,emotion_val):
		if(not self.emotion_is_valid(emotion_val)):
			self._error.error("unknown emotion: " + str(emotion_val))

		self._emotion = emotion_val
		self.update_image_to_reflect_emotion_and_eye_direction()

	def send_image(self,path):
		img = cv2.imread(path)
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
		pub.publish(msg)
		rospy.sleep(1)

	def image_static_path(self,filename):
		return os.path.dirname(os.path.realpath(__file__)) + "/emotions/" + filename