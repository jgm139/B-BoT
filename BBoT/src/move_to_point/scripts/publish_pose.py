#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import tf
import os, sys
import actionlib

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener

def move_to_point_client(tiago_orientation):

    print("Action client...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal()

    #param = rospy.get_param("gazebo/obj")
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    #goal.target_pose.pose.position.x = param['position']['x']-0.35
    #goal.target_pose.pose.position.y = param['position']['y']-0.35
    goal.target_pose.pose.position.x = -2
    goal.target_pose.pose.position.y = -2
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = tiago_orientation[0]
    goal.target_pose.pose.orientation.y = tiago_orientation[1]
    goal.target_pose.pose.orientation.z = tiago_orientation[2]
    goal.target_pose.pose.orientation.w = tiago_orientation[3]

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


def main(args):
    rospy.init_node('move_to_point_node')

    listener = tf.TransformListener()
    try:
        #while not listener.frameExists("/base_footprint") and listener.frameExists("/map"):
        listener.waitForTransform('/map','/base_footprint', rospy.Time(), rospy.Duration(4.0))

        (trans, rot) = listener.lookupTransform('/map','/base_footprint',rospy.Time())
        
        #print trans, rot

        move_to_point_client(rot)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

main(sys.argv)