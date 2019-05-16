#! /usr/bin/env python
import rospy
import roslib
import tf
import os, sys
import moveit_commander
import moveit_msgs

from geometry_msgs.msg import Point
from tf import TransformListener

class Prepose:
    ARM_TORSO_JOINTS = [0.35, 0.25, 0, -2, 1, 0, 0, 0]

    def __init__(self):
        self.arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm_torso_joints_values = self.arm_torso_group.get_current_joint_values()

    def prepose(self):

        for i in range(0, len(self.ARM_TORSO_JOINTS)):
            self.arm_torso_joints_values[i] = self.ARM_TORSO_JOINTS[i]

        self.arm_torso_group.go(joints=self.arm_torso_joints_values)


def main(args):
    rospy.init_node('prepose_node')
    prepose = Prepose()
    prepose.prepose()

main(sys.argv)