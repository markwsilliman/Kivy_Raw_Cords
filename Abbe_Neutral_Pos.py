#!/usr/bin/env python

import math
import random
import rospy
import argparse
import struct
import sys
from abbe_ik import Abbe_IK
import threading

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
from Abbe_Three_Points_To_Rot_Matrix import Abbe_Three_Points_To_Rot_Matrix

if __name__ == '__main__':
    rospy.init_node('Abbe_IK', anonymous=True)
    Abbe_IK = Abbe_IK()
    Abbe_IK.neutral()	
