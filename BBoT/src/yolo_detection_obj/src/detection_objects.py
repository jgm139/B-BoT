#! /usr/bin/env python
 
import rospy
import cv2
import os, sys

from sensor_msgs.msg import Image
from yolo_msg.msg import MSGYolo
from cv_bridge import CvBridge, CvBridgeError

REPOSITORY = "/home/juliagm/Documentos/B-BoT/"
INCLUDE_DARKNET = "include/darknet/"

# Array que devuelve la posicion de la funcion detect
# (name, precision, array_objeto_detectado)
NAME_YOLO = 0
PRECISION_YOLO = 1
ARRAY_OBJECT_DETECT = 2

# POSICION DEL OBJETO (x, y, w, h)
Xi_YOLO = 0
Yi_YOLO = 1
Wi_YOLO = 2
Hi_YOLO = 3

sys.path.append(REPOSITORY+INCLUDE_DARKNET)

import darknet as dn

class TinyYolo:

    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.take_images_callback)
        self.pub = rospy.Publisher('detection/obj', MSGYolo, queue_size=1)

        self.path_cfg = REPOSITORY + INCLUDE_DARKNET + "yolov3-tiny.cfg"
        self.path_weights = REPOSITORY + INCLUDE_DARKNET + "yolov3-tiny_27200.weights"
        self.path_data = REPOSITORY + INCLUDE_DARKNET + "data/obj.data"

        self.currentImage = None
        self.filenameImage = REPOSITORY + "BBoT/src/assets/prediction.jpg"
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
                
                onames = xn = yn = widths = heights = []

                for objDetect in range(len(detection)):
                    onames.append(detection[objDetect][NAME_YOLO])
                    
                    xn.append(detection[objDetect][ARRAY_OBJECT_DETECT][Xi_YOLO])
                    yn.append(detection[objDetect][ARRAY_OBJECT_DETECT][Yi_YOLO])
                    widths.append(detection[objDetect][ARRAY_OBJECT_DETECT][Wi_YOLO])
                    heights.append(detection[objDetect][ARRAY_OBJECT_DETECT][Hi_YOLO])

                    data = MSGYolo()
                    data.names = onames
                    data.xn = xn
                    data.yn = yn
                    data.widths = widths
                    data.heights = heights
                    self.pub.publish(data)
                    self.rate.sleep()

                for i in range(0, len(onames)):
                    name = onames[i]
                    print type(xn[i])
                    p1 = (int(xn[i]-(widths[i]/2)), int(yn[i]-(heights[i]/2)))
                    p2 = (int(xn[i]+(widths[i]/2)), int(yn[i]+(heights[i]/2)))

                    cv2.rectangle(self.currentImage, p1, p2, (0,0,0))

                cv2.imshow('tiny yolo detect', self.currentImage)
                cv2.waitKey(1)


def main(args):
    rospy.init_node('yolo_detection_node')
    dn.set_gpu(0)
    yolo_node = TinyYolo()
    yolo_node.run()

main(sys.argv)    