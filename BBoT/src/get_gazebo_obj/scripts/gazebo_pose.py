#! /usr/bin/env python
import rospy
import cv2
import os, sys

from yolo_msg.msg import MSGYolo
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

TARGET_OBJECT = None
AVAILABLE_OBJECTS = {"cocacola", "beer", "pringles"}

class DetectedGazeboObj:

    def __init__(self):
        self.sub_detection = rospy.Subscriber('/detection/obj', MSGYolo, self.get_detected_obj_callback)
        self.sub_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_model_states_callback)
        self.pub = rospy.Publisher('/gazebo_pose_obj', Point, queue_size=1)

        self.model_states_names = None
        self.model_states_poses = None
        self.detected_obj = None

        self.target_point = Point()

    def get_detected_obj_callback(self, data):
        self.detected_obj = data.names

    def get_model_states_callback(self, data):  
        self.model_states_names = data.names
        self.model_states_poses = data.pose

    def search_target_pose(self):

        if any(TARGET_OBJECT in s for s in AVAILABLE_OBJECTS):

            if any(TARGET_OBJECT in s for s in self.detected_obj):
                index_name = self.model_states_names.index(TARGET_OBJECT)
                self.target_point = self.model_states_poses[index_name].position

                self.pub.publish(self.target_point)
            else:
                rospy.loginfo("TIAGo: Hey humano! No tengo este objeto en mi campo de visión.")    
        else:
            rospy.logdebug("Este tipo de objeto no se puede detectar.")
            rospy.logdebug("Prueba con: cocacola, beer, pringles")

def main(args):
    rospy.init_node('get_gazebo_obj_node')
    TARGET_OBJECT = sys.argv[0]
    rospy.spin()

main(sys.argv)