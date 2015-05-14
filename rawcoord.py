import kivy
kivy.require('1.1.1')

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock

class RawCord(Widget):
    def on_touch_down(self, touch):
	print "DOWN"
	print "X: " + str(touch.x)
	print "Y: " + str(touch.y)

    def on_touch_up(self, touch):
	print "UP"
	print "X: " + str(touch.x)
	print "Y: " + str(touch.y)


class RawCordApp(App):
    def build(self):
        g = RawCord()
        return g


if __name__ == '__main__':
    RawCordApp().run()
