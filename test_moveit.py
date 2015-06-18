# Note these must be running!
# rosrun baxter_interface joint_trajectory_action_server.p
# roslaunch baxter_moveit_config demo_baxter.launch
import argparse
import sys
import copy
import rospy
import numpy
import cv2
import tf
import moveit_commander
import math

from std_msgs.msg import Header

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    Grasp,
    GripperTranslation,
)

from trajectory_msgs.msg import(
    JointTrajectory,
    JointTrajectoryPoint
)

from geometry_msgs.msg import (
    Point,
    Polygon,
    Pose,
    PoseStamped,
    Quaternion,
    Vector3,
    Vector3Stamped
)

import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml

def main():
    rospy.init_node("move_test")
    rightHand = moveit_commander.MoveGroupCommander("right_arm")

    rightHand.set_goal_position_tolerance(0.02)
    rightHand.set_goal_orientation_tolerance(0.03)
    rightHand.allow_replanning(True)

    quaternion = tf.transformations.quaternion_from_euler(0, math.pi, 0.0)

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "base"
    poseStamped.pose.position.x =  0.35
    poseStamped.pose.position.y =  -0.46
    poseStamped.pose.position.z =  0.168
    poseStamped.pose.orientation.x = quaternion[0]
    poseStamped.pose.orientation.y = quaternion[1]
    poseStamped.pose.orientation.z = quaternion[2]
    poseStamped.pose.orientation.w = quaternion[3]

    rightHand.set_pose_target(poseStamped)
    rightHand.go()

    print (rightHand.get_current_pose())
    print (rightHand.get_current_pose(rightHand.get_end_effector_link()))

if __name__ == '__main__':
    main()