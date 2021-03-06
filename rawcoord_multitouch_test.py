#To exit: esc then ctrl-c seems to work

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

class RawCord(Widget):

    def on_touch_down(self, touch):
	with self.canvas:
		Rectangle(pos=(touch.x,touch.y), size=(10,10))
	#remove next line
	return True

	with open('kivy_touch_down_multitouch_log.json', 'a') as outfile:
	    outfile.write(str(float(touch.x) / float(Window.size[0])) + "," + str(float(touch.y) / float(Window.size[1])) + "\n")

    def on_touch_up(self, touch):
	pass
	#print "UP"
	#print "X: " + str(float(touch.x) / float(Window.size[0]))
	#print "Y: " + str(float(touch.y) / float(Window.size[1]))


class RawCordApp(App):
    def build(self):
        g = RawCord()
        return g


if __name__ == '__main__':
    #Window.fullscreen = True
    RawCordApp().run()
