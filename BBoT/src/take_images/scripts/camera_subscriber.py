#! /usr/bin/env python
 
import rospy
import cv2
import os, sys, pygame
from pygame.locals import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class TakeImageNode:
    def __init__(self):
        rospy.loginfo("Initialization of TakeImageNode")
        self.bridge = CvBridge()
        rospy.loginfo("Setting up subscriber: /xtion/rgb/image_raw")
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.take_images_callback)
        rospy.loginfo("Setting up publisher: /mobile_base_controller/cmd_vel")
        self.move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(5)
        self.lastImageTaken = None
        self.numImage = 0

        rospy.loginfo("Images 608x608")
        self.width = 608
        self.height = 608
        self.photo = False
        self.loop = False

    def take_images_callback(self, data):
   
        try:
            if self.photo:

                if not self.loop:
                    self.photo = False

                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                cv2.waitKey(3)
                
                self.lastImageTaken = cv2.resize(cv_image, (self.width, self.height))
                fileName = "/home/juliagm/Documentos/B-BoT/dataset/Images/001/" + str(self.numImage) + ".jpg"
                rospy.loginfo("Saving the last photography " + fileName)
                cv2.imwrite(fileName, self.lastImageTaken)

                self.numImage+=1
                self.rate.sleep()
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

        try:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == QUIT: 
                        pygame.quit()
                        sys.exit()
                        rospy.signal_shutdown('Bye, bye human')

                    if event.type == KEYDOWN and event.key == 119: # W - straight
                        move_cmd.linear.x += 0.65
                    elif event.type == KEYDOWN and event.key == 97: # A - left
                        move_cmd.angular.z += 0.35
                        move_cmd.linear.x = 0.0
                    elif event.type == KEYDOWN and event.key == 100: # D - right
                        move_cmd.angular.z += -0.35
                    elif event.type == KEYDOWN and event.key == 112: # P - take photo
                        self.photo = not self.photo
                    elif event.type == KEYDOWN and event.key == 122: # Z - stop
                        move_cmd.linear.x = 0.0
                        move_cmd.angular.z = 0.0
                    elif event.type == KEYDOWN and event.key == 108: # L - loop
                        self.loop = True    
                        self.photo = True
                    elif event.type == KEYDOWN and event.key == 115: # S - stop loop
                        self.loop = False    
                        self.photo = False

                pygame.event.pump()
                self.move.publish(move_cmd)

        except rospy.ROSInterruptException:
            rospy.loginfo("Program interrupted before completion", file=sys.stderr)  
    

def main(args):
    rospy.init_node('take_images_node', disable_signals=True)

    pygame.init()
    pygame.display.set_mode((300,250))
    pygame.display.set_caption('Click here and move TIAGo!')

    tI = TakeImageNode()
    tI.run()
    cv2.destroyAllWindows()
    
    rospy.spin()


main(sys.argv)