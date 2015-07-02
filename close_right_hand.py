#!/usr/bin/env python

import math
import random
import rospy
import argparse
import struct
import sys
import numpy
import cv2
import tf
import copy
from abbe_ik import Abbe_IK
import threading
import Tkinter as tk
import json
import moveit_commander
import time
import urllib, json
from abbe_gripper import Abbe_Gripper

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    Grasp,
    GripperTranslation,
)


from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
	Polygon,
	Vector3,
    Vector3Stamped
)

import yaml


from trajectory_msgs.msg import(
    JointTrajectory,
    JointTrajectoryPoint
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range
from Abbe_Three_Points_To_Rot_Matrix import Abbe_Three_Points_To_Rot_Matrix

if __name__ == '__main__':
    rospy.init_node('Abbe_Table_Sync', anonymous=True)
    _grippers = Abbe_Gripper()
    print "sleeping for 10"
    rospy.sleep(10)
    _grippers.close(False)