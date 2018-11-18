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
        self.lastImageTaken = None
        self.numImage = 0
        self.width = 320
        self.height = 240
        self.photo = False

    def take_images_callback(self, data):
   
        try:
            if self.photo:
                print("Saving the last photography")
                self.photo = False
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.lastImageTaken = cv2.resize(cv_image, (self.width, self.height))
                image_gray = cv2.cvtColor(self.lastImageTaken, cv2.COLOR_BGR2GRAY)

                fileName = "/home/juliagm/Documentos/B-BoT/dataset/Image" + str(self.numImage) + ".png"

                cv2.imwrite(fileName, image_gray)

                self.numImage+=1
        except CvBridgeError as e:
            print(e)

    def run(self):
        print("run module")
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

        try:
            while not rospy.is_shutdown():
                keys = pygame.key.get_pressed()
                
                if keys[K_UP]:
                    print("Move up")
                    move_cmd.linear.x = 0.5
                elif keys[K_LEFT]:
                    print("Move left")
                    move_cmd.angular.z = 0.2
                elif keys[K_RIGHT]:
                    print("Move right")
                    move_cmd.angular.z = -0.2
                elif keys[K_w]:
                    print("Take photo")
                    self.photo = not self.photo
                elif keys[K_PERIOD]:
                    break

                keys = None
                self.move.publish(move_cmd)

        except KeyboardInterrupt:
            print("Keyboard error")  
    

def main(args):
    pygame.init()
    pygame.display.set_mode((400,400))
    tI = TakeImageNode()
    rospy.init_node('take_images_node')
    tI.run()
    rospy.spin()


main(sys.argv)