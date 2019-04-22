#! /usr/bin/env python

import rospy
import cv2
import os, sys

from sensor_msgs.msg import Image
from yolo_msg.msg import MSGYolo
from cv_bridge import CvBridge, CvBridgeError

REPOSITORY = "/home/juliagm/Documentos/B-BoT/"
INCLUDE_DARKNET = "include/darknet/"

sys.path.append(REPOSITORY+INCLUDE_DARKNET)

import darknet as dn


class TinyYolo:
    # Array que devuelve la posicion de la funcion detect
    # (name, precision, array_objeto_detectado)
    NAME_OBJECT = 0
    PRECISION_OBJECT = 1
    ARRAY_OBJECT_DETECT = 2

    # POSICION DEL OBJETO (x, y, w, h)
    Xi_OBJECT = 0
    Yi_OBJECT = 1
    Wi_OBJECT = 2
    Hi_OBJECT = 3

    # Generate different colors for different classes 
    COLORS = {'coke_can': (255,0,0), 'beer': (0,255,0), 'pringles': (0,0,255)}

    def __init__(self):
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.take_images_callback)
        self.pub = rospy.Publisher('/detection/obj', MSGYolo, queue_size=1)

        self.path_cfg = REPOSITORY + INCLUDE_DARKNET + "yolov3-tiny.cfg"
        self.path_weights = REPOSITORY + INCLUDE_DARKNET + "yolov3-tiny_27200.weights"
        self.path_data = REPOSITORY + INCLUDE_DARKNET + "data/obj.data"
        self.filenameImage = REPOSITORY + "BBoT/src/assets/prediction.jpg"

        self.currentImage = None
        self.newImage = False

        self.tiny_yolo_net = None
        self.tiny_yolo_meta = None
    
        self.tiny_yolo_net = dn.load_net(self.path_cfg, self.path_weights, 0)
        self.tiny_yolo_meta = dn.load_meta(self.path_data)

        self.rate = rospy.Rate(5)        

    def take_images_callback(self, data):
        try:
            self.currentImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite(self.filenameImage, self.currentImage)
            self.newImage = True
        except CvBridgeError as e:
            rospy.logerr(e)
  
    def run(self):

        onames = []
        predictions = []
        xn = []
        yn = []
        widths = []
        heights = []

        while not rospy.is_shutdown():
            if self.newImage:
                detection = dn.detect(self.tiny_yolo_net, self.tiny_yolo_meta, self.filenameImage)

                print "============================================="
                for objDetect in detection:
                    print "DETECTED OBJECT: ", objDetect[self.NAME_OBJECT]
                    print "PREDICTION: ", objDetect[self.PRECISION_OBJECT]*100, "%"
                    onames.append(objDetect[self.NAME_OBJECT])
                    predictions.append(objDetect[self.PRECISION_OBJECT])

                    print "x: ", objDetect[self.ARRAY_OBJECT_DETECT][self.Xi_OBJECT], " / y: ", objDetect[self.ARRAY_OBJECT_DETECT][self.Yi_OBJECT]
                    print "w: ", objDetect[self.ARRAY_OBJECT_DETECT][self.Wi_OBJECT], " / h: ", objDetect[self.ARRAY_OBJECT_DETECT][self.Hi_OBJECT]

                    xn.append(objDetect[self.ARRAY_OBJECT_DETECT][self.Xi_OBJECT])
                    yn.append(objDetect[self.ARRAY_OBJECT_DETECT][self.Yi_OBJECT])
                    widths.append(objDetect[self.ARRAY_OBJECT_DETECT][self.Wi_OBJECT])
                    heights.append(objDetect[self.ARRAY_OBJECT_DETECT][self.Hi_OBJECT])
                    print "============================================="

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
                    
                    p1 = (int(xn[i]-(widths[i]/2)), int(yn[i]-(heights[i]/2)))
                    p2 = (int(xn[i]+(widths[i]/2)), int(yn[i]+(heights[i]/2)))
                    pname = (int(xn[i]+(widths[i]/2)), int(yn[i]-(heights[i]/2)-5))

                    cv2.rectangle(self.currentImage, p1, p2, self.COLORS[name], 2)
                    cv2.putText(self.currentImage, name, pname, cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[name])

                del onames [:]
                del predictions [:]
                del xn [:]
                del yn [:]
                del widths [:]
                del heights [:]

                cv2.imshow('Prediction', self.currentImage)
                cv2.waitKey(1)
                self.rate.sleep()

            self.newImage = False    


def main(args):
    rospy.init_node('yolo_detection_node')
    dn.set_gpu(0)
    yolo_node = TinyYolo()
    yolo_node.run()

main(sys.argv)    