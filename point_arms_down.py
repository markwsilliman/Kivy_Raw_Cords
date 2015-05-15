#!/usr/bin/env python

#
# Detect Table Height
#

import math
import random
import rospy
import argparse
import struct
import sys
from abbe_ik import Abbe_IK
import threading
import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range

if __name__ == '__main__':
    rospy.init_node('Abbe_Relative_Position_Tests', anonymous=True)
    n = Abbe_IK()
    time.sleep(2)
    pose = n.get_pose('left')
    
    if not n.set_left(float(pose.x),float(pose.y),float(pose.z)):
	print "left failed to point down at pose"

    pose = n.get_pose('right')
    
    if not n.set_right(float(pose.x),float(pose.y),float(pose.z)):
	print "right failed to point down at pose"
    
    print "complete"
