#! /usr/bin/env python
from __future__ import print_function

import rospy
import os, sys
import actionlib

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_point_client():

    print("Action client...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.5
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    print(goal.target_pose.pose)

    print("Sending goal...")
    client.send_goal(goal)

    print("Waiting for result...")
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        result = client.get_result()

        if result:
            rospy.loginfo("Goal execution done!")

def get_pose(data):
      
    

def listener_to_detection():
    sub = rospy.Subscriber("gazebo_pose/obj", PoseStamped, get_pose)

def main(args):
    try:
        rospy.init_node('move_to_point_node')
        listener_to_detection()
        move_to_point_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

main(sys.argv)