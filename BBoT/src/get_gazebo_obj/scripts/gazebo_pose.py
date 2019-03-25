#! /usr/bin/env python
import rospy
import cv2
import os, sys

from yolo_msg.msg import MSGYolo
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

TARGET_OBJECT = ""
AVAILABLE_OBJECTS = {"coke_can", "beer", "pringles"}

class DetectedGazeboObj:

    def __init__(self):
        self.sub_detection = rospy.Subscriber('/detection/obj', MSGYolo, self.get_detected_obj_callback)
        self.sub_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_model_states_callback)
        self.pub = rospy.Publisher('/gazebo_pose_obj', Point, queue_size=1)

        self.model_states_names = {}
        self.model_states_poses = {}
        self.detected_obj = ""

        self.target_point = Point()
        self.rate = rospy.Rate(15)

    def get_detected_obj_callback(self, data):
        self.detected_obj = data.names
        rospy.loginfo(self.detected_obj)

    def get_model_states_callback(self, data):  
        self.model_states_names = data.name
        self.model_states_poses = data.pose

    def search_target_pose(self):
        global TARGET_OBJECT
        global AVAILABLE_OBJECTS

        while not rospy.is_shutdown():
            if any(TARGET_OBJECT in s for s in AVAILABLE_OBJECTS):

                if any(TARGET_OBJECT in s for s in self.detected_obj):
                    index_name = self.model_states_names.index(TARGET_OBJECT)
                    self.target_point = self.model_states_poses[index_name].position

                    rospy.loginfo("Publicando en /gazebo_pose_obj la posicion del objeto...")
                    self.pub.publish(self.target_point)
                else:
                    rospy.loginfo("TIAGo: Hey humano! No tengo este objeto en mi campo de vision.")    
            else:
                rospy.logdebug("Este tipo de objeto no se puede detectar.")
                rospy.logdebug("Prueba con: cocacola, beer, pringles")

            self.rate.sleep()    

def main(args):
    global TARGET_OBJECT

    rospy.init_node('get_gazebo_obj_node')
    TARGET_OBJECT = sys.argv[1]
    rospy.loginfo("Voy a buscar el objeto %s", TARGET_OBJECT)
    gp = DetectedGazeboObj()
    gp.search_target_pose()
    rospy.spin()

main(sys.argv)