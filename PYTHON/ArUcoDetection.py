#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import cv2 as cv
import numpy as np
import message_filters





def callback(image,depth):
    bridgeRGB = CvBridge()
    RGB = bridgeRGB.imgmsg_to_cv2(image, "bgr8")

    bridgeDepth = CvBridge()
    depthImage = bridgeDepth.imgmsg_to_cv2(depth, "16UC1" )
    
    imgGray = cv.cvtColor(RGB, cv.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(10)
    arucoParam = aruco.DetectorParameters_create()

    bboxs, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters = arucoParam)
    print(ids)
    cv.imshow("bounding_box", RGB)
    cv.waitKey(1)


def listener():
    
   
    rospy.init_node('arUcoListener', anonymous=True)

    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10,0.1,allow_headerless=True)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

