# Note these must be running!
# rosrun baxter_interface joint_trajectory_action_server.py
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
from abbe_gripper import Abbe_Gripper
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml

class testmoveit2:
    def __init__(self):
        rospy.init_node("move_test")

        self._grippers = Abbe_Gripper()

        self.robot = moveit_commander.RobotCommander()
        self.rightHand = moveit_commander.MoveGroupCommander("right_arm")
        self.scene = moveit_commander.PlanningSceneInterface()

        print "sleeping for 2 seconds"
        rospy.sleep(2)
        print "done sleeping"

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.88
        p.pose.position.y = 0
        p.pose.position.z = -0.175 - (0.795/2)
        self.scene.add_box("table", p, (0.72, 1.2, 0.795))

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.88
        p.pose.position.y = 0
        p.pose.position.z = -0.175 + (0.095 / 2)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0.5) #roll, pitch, yaw
        p.pose.orientation.x = quaternion[0]
        p.pose.orientation.y = quaternion[1]
        p.pose.orientation.z = quaternion[2]
        p.pose.orientation.w = quaternion[3]

        self.scene.add_box("cup", p, (0.08, 0.14, 0.095)) #z should be0.095

        '''
        p = PoseStamped();
        p.header.frame_id = "right_gripper"
        p.pose.position = Point(*[0,0,0.025])
        p.pose.orientation = Quaternion(*[0,0,0,1])
        self.scene.attach_box("right_gripper","my_attached_gripper_right",p,(0.07,0.1,0.12))

        p = PoseStamped();
        p.header.frame_id = "left_gripper"
        p.pose.position = Point(*[0,0,0.025])
        p.pose.orientation = Quaternion(*[0,0,0,1])
        self.scene.attach_box("left_gripper","my_attached_gripper_left",p,(0.07,0.1,0.12))
        '''
        print "Map Done"


    def testmove(self):
        print "Let's Move"
        self.rightHand.set_goal_position_tolerance(0.09) #0.02
        self.rightHand.set_goal_orientation_tolerance(0.09) #0.03
        self.rightHand.allow_replanning(True)

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

        self.rightHand.set_pose_target(poseStamped)
        self.rightHand.go()

        print ( self.rightHand.get_current_pose())
        print ( self.rightHand.get_current_pose( self.rightHand.get_end_effector_link()))

if __name__ == '__main__':
    t = testmoveit2()

    t.testmove()
    rospy.spin()