#! /usr/bin/env python

import rospy
import actionlib
import tf.transformations
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def move_base_client():
    client = actionlib.SimpleActionClient('move_base', geometry_msgs.msg.PoseStamped)

    client.wait_for_server()

    goal = geometry_msgs.msg.PoseStamped()

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = 0
    goal.pose.position.y = 0
    goal.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 60)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

def main(args):
    
    rospy.init_node('move_base_client_node')
    result = move_base_client()
    print result

main(sys.argv)    
