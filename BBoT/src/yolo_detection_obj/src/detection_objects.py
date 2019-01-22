#! /usr/bin/env python
 
import rospy
import cv2
import os, sys, pygame

from sensor_msgs.msg import Image
from yolo_msg.msg import MSGYolo
from cv_bridge import CvBridge, CvBridgeError

sys.path.append(sys.path.append(os.path.join(os.getcwd(),'include/darknet/')))

import darknet as dn

class TinyYolo:

    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.take_images_callback)
        self.move = rospy.Publisher('detection_obj', MSGYolo, queue_size=1)

        self.path_cfg = "/home/juliagm/Documentos/B-BoT/include/darknet/yolov3-tiny.cfg"
        self.path_weights = "/home/juliagm/Documentos/B-BoT/include/darknet/yolov3-tiny_27200.weights"
        self.path_data = "/home/juliagm/Documentos/B-BoT/include/darknet/obj.data"

        self.currentImage = None
        self.filenameImage = "/home/juliagm/Documentos/B-BoT/BBoT/src/yolo_detection_obj/prediction.jpg"
        self.tiny_yolo_net = None
        self.tiny_yolo_meta = None
    
        self.tiny_yolo()
        self.rate = rospy.Rate(5)

    def tiny_yolo(self):
        self.tiny_yolo_net = dn.load_net(self.path_cfg, self.path_weights, 0)
        self.tiny_yolo_meta = dn.load_meta(self.path_data)

    def take_images_callback(self, data):
        try:
            self.currentImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite(self.filenameImage, self.currentImage)
        except CvBridgeError as e:
            print(e)

    def run(self):
        while not rospy.is_shutdown():
            if(self.currentImage is not None):
                detection = dn.detect(self.tiny_yolo_net, self.tiny_yolo_meta, self.filenameImage)
                print detection

                num_objects = len(detection)



def main(args):
    rospy.init_node('yolo_detection_obj')
    dn.set_gpu(0)
    yolo_node = TinyYolo()
    yolo_node.run()

main(sys.argv)    