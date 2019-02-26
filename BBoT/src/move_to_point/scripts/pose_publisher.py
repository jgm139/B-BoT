#! /usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseStamped

def publisher(data):
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    
    pub.publish()

def listener:
    rospy.init_node('move_to_point_node')

    sub = rospy.Subscriber("gazebo_pose/obj", PoseStamped, publisher)


def main(args):
    listener()

main(sys.argv)    