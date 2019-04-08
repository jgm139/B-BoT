#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import tf
import os, sys
import actionlib

from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener

def move_to_point_client(object_position, tiago_orientation):

    rospy.loginfo("Action client...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = object_position.x
    if goal.target_pose.pose.position.y>0:
        goal.target_pose.pose.position.y = object_position.y-0.35
    else:    
        goal.target_pose.pose.position.y = object_position.y+0.35
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = tiago_orientation[0]
    goal.target_pose.pose.orientation.y = tiago_orientation[1]
    goal.target_pose.pose.orientation.z = tiago_orientation[2]
    goal.target_pose.pose.orientation.w = tiago_orientation[3]

    rospy.loginfo(goal.target_pose.pose)

    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        result = client.get_result()

        if result:
            rospy.loginfo("Goal execution done!")


def main(args):
    rospy.init_node('move_to_point_node')

    listener = tf.TransformListener()
    rospy.Rate(4).sleep()
    try:
        listener.waitForTransform('/map','/base_footprint', rospy.Time(), rospy.Duration(1.0))

        (trans, rot) = listener.lookupTransform('/map','/base_footprint',rospy.Time())

        pos = rospy.wait_for_message("/gazebo_pose_obj", Point)

        move_to_point_client(pos,rot)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion", file=sys.stderr)

main(sys.argv)