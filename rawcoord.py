#Note: must run as SU e.g. sudo python rawcoord.py
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

class RawCord(Widget):
    def on_touch_down(self, touch):
	print "DOWN"
	print "X: " + str(float(touch.x) / float(Window.size[0]))
	print "Y: " + str(float(touch.y) / float(Window.size[1]))

    def on_touch_up(self, touch):
	print "UP"
	print "X: " + str(float(touch.x) / float(Window.size[0]))
	print "Y: " + str(float(touch.y) / float(Window.size[1]))


class RawCordApp(App):
    def build(self):
        g = RawCord()
        return g


if __name__ == '__main__':
    Window.fullscreen = True
    RawCordApp().run()
