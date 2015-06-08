import kivy
import urllib.request
kivy.require('1.1.1')

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.core.window import Window
import json
from kivy.graphics import *
from kivy.config import Config
from random import randint

class RawCord(Widget):
	
	def on_touch_down(self, touch):
		with self.canvas:
			Color(float(randint(3,10)/10), float(randint(3,10)/10), float(randint(3,10)/10))
			Rectangle(pos=(touch.x - 150,touch.y - 25 - 150), size=(300,300))
			x_per = float(touch.x) / float(windowsize_width())
			y_per = float(touch.y) / float(windowsize_height())
			with urllib.request.urlopen('http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com/touch_api.php?touch_api_x=' + str(x_per) + '&touch_api_y=' + str(y_per) + '&touch_api_type=1') as response:
				html = response.read()
				
	def on_touch_up(self, touch):
		with self.canvas:
			Color(1, 1, 1)
			Rectangle(pos=(touch.x - 150,touch.y - 25 - 150), size=(300,300))
			x_per = float(touch.x) / float(windowsize_width())
			y_per = float(touch.y) / float(windowsize_height())
			with urllib.request.urlopen('http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com/touch_api.php?touch_api_x=' + str(x_per) + '&touch_api_y=' + str(y_per) + '&touch_api_type=2') as response:
				html = response.read()
					
	def on_touch_move(self, touch):
		return False #to noisy for now
		with self.canvas:
			Color(0, 1., 0)
			Rectangle(pos=(touch.x,touch.y - 25), size=(2,2))
			x_per = float(touch.x) / float(windowsize_width())
			y_per = float(touch.y) / float(windowsize_height())
			with urllib.request.urlopen('http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com/touch_api.php?touch_api_x=' + str(x_per) + '&touch_api_y=' + str(y_per) + '&touch_api_type=3') as response:
				html = response.read()
	
class RawCordApp(App):
	def build(self):
		g = RawCord()
		Window.size = (windowsize_width(), windowsize_height())
		Window.clearcolor = (1, 1, 1, 1)
		return g
		
def windowsize_width():
	return 1600
			
def windowsize_height():
	return 900
	
if __name__ == '__main__':
	Window.fullscreen = True
	RawCordApp().run()