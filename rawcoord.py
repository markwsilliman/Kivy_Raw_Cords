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

class RawCord(Widget):
    def on_touch_down(self, touch):
	data = [str(float(touch.x) / float(Window.size[0])),str(float(touch.y) / float(Window.size[1]))]
	with open('kivy_touch_down_log.json', 'w') as outfile:
	    json.dump(data, outfile)

    def on_touch_up(self, touch):
	return False
	#print "UP"
	#print "X: " + str(float(touch.x) / float(Window.size[0]))
	#print "Y: " + str(float(touch.y) / float(Window.size[1]))


class RawCordApp(App):
    def build(self):
        g = RawCord()
        return g


if __name__ == '__main__':
    Window.fullscreen = True
    RawCordApp().run()
