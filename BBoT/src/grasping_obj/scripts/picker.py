#! /usr/bin/env python
import rospy
import roslib
import tf
import os, sys
import moveit_commander
import moveit_msgs

from geometry_msgs.msg import Point
from tf import TransformListener

class Picker:

    MAX_GRIPPER_JOINT = 0.044
    CENTER_GRIPPER = 0.018
    ARM_3_DOWN = -1.37
    ARM_3_UP = -1.8

    def __init__(self):
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1.0)
        self.rate.sleep()
        #self.target_object = rospy.wait_for_message("/gazebo_pose_obj", Point)
        self.target_object = Point(0.5,3.5,0.8155)

        self.arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm_torso_joints_values = self.arm_torso_group.get_current_joint_values()

        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.gripper_joints_values = self.gripper_group.get_current_joint_values()
        
        self.robot = moveit_commander.RobotCommander()

    def open_gripper(self):

        self.gripper_joints_values[0] = self.MAX_GRIPPER_JOINT
        self.gripper_joints_values[1] = self.MAX_GRIPPER_JOINT  

        self.gripper_group.go(self.gripper_joints_values) 

    def close_gripper(self):

        self.gripper_joints_values[0] = 0.035 
        self.gripper_joints_values[1] = 0.035 

        self.gripper_group.go(self.gripper_joints_values)  


    def picking(self):

        self.listener.waitForTransform('/map','/gripper_grasping_frame', rospy.Time(), rospy.Duration(1.0))

        running = True
        theta_x = 0.05
        theta_y = 0.05

        joint_1 = self.arm_torso_joints_values[1]
        joint_4 = self.arm_torso_joints_values[4]
        object_x = abs(self.target_object.x)
        object_y = abs(self.target_object.y)

        rospy.loginfo("--------------------------------")
        rospy.loginfo("Comenzando posicionamiento del gripper...")

        while running:
            try:
                (gripper_pose, gripper_rot) = self.listener.lookupTransform('/map','/gripper_grasping_frame',rospy.Time())

                gripper = (abs(gripper_pose[0])-self.CENTER_GRIPPER, abs(gripper_pose[1]))

                dif_x = abs(object_x-gripper[0])
                dif_y = abs(object_y-gripper[1])

                if dif_x<=theta_x and dif_y<=theta_y:
                    rospy.loginfo("------------FINALIZADO------------")
                    running = False
                else:
                    
                    if object_x > gripper[0]:
                        joint_1 -= 0.02 if joint_1 > 0.02 else 0
                        joint_4 -= 0.04 if joint_4 > 0.04 else 0
                    elif object_x < gripper[0]:
                        joint_1 += 0.02
                        joint_4 += 0.04 if joint_4 <= 2.2 else 2.2      
                    
                    if object_y > gripper[1]:
                        joint_1 += 0.02
                        joint_4 -= 0.04 if joint_1 > 0.04 else 0
                    elif object_y < gripper[1]:
                        joint_1 -= 0.02 if joint_1 > 0.02 else 0
                        joint_4 += 0.04 if joint_4 <= 2.2 else 2.2   

                    self.arm_torso_joints_values[1] = joint_1
                    self.arm_torso_joints_values[4] = joint_4
                    self.arm_torso_group.go(joints=self.arm_torso_joints_values)
                    
                self.rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)

        rospy.loginfo("Comenzando agarre del objeto...")
        self.open_gripper()
        self.arm_torso_joints_values[3] = self.ARM_3_DOWN
        self.arm_torso_group.go(joints=self.arm_torso_joints_values)
        self.close_gripper()
        self.arm_torso_joints_values[3] = self.ARM_3_UP
        self.arm_torso_group.go(joints=self.arm_torso_joints_values)


def main(args):
    rospy.init_node('grasping_obj_node')
    picker = Picker()
    picker.picking()

main(sys.argv)