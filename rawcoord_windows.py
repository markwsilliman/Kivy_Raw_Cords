import kivy
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
Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')

class RawCord(Widget):
	
	def on_touch_down(self, touch):
		with self.canvas:
			Rectangle(pos=(touch.x,touch.y - 25), size=(2,2))
	
class RawCordApp(App):
	def build(self):
		g = RawCord()
		Window.size = (1920, 1080)
		return g
		
if __name__ == '__main__':
	Window.fullscreen = True
	RawCordApp().run()