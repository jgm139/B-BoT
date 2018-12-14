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
        print("Initialization of TakeImageNode")
        self.bridge = CvBridge()
        print("Setting up subscriber: /xtion/rgb/image_raw")
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.take_images_callback)
        print("Setting up publisher: /mobile_base_controller/cmd_vel")
        self.move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(5)
        self.lastImageTaken = None
        self.numImage = 91
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
                # cv2.imshow("Image in rgb",cv_image)
                cv2.waitKey(3)
                self.lastImageTaken = cv2.resize(cv_image, (self.width, self.height))
                # image_gray = cv2.cvtColor(self.lastImageTaken, cv2.COLOR_BGR2GRAY)
            
                fileName = "/home/juliagm/Documentos/B-BoT/dataset/Images/005/" + str(self.numImage) + ".jpg"
                print("Saving the last photography " + fileName)
                cv2.imwrite(fileName, self.lastImageTaken)

                self.numImage+=1
                self.rate.sleep()
        except CvBridgeError as e:
            print(e)

    def run(self):
        print("run module")
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

        try:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == QUIT: sys.exit()

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

        except KeyboardInterrupt:
            print("Keyboard error")  
    

def main(args):
    pygame.init()
    pygame.display.set_mode((200,200))
    rospy.init_node('take_images_node')
    tI = TakeImageNode()
    tI.run()
    cv2.destroyAllWindows()
    rospy.spin()


main(sys.argv)