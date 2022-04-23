#!/usr/bin/env python
from ctypes import sizeof
from unittest import result
from sensor_msgs.msg import Image
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PointStamped
import message_filters
import open3d as o3d

class PointCloud:
    
    def __init__(self):
        

    def point_cloud(self, depth):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth > 0) & (depth < 255)
        z = np.where(valid, depth / 256.0, np.nan)
        x = np.where(valid, z * (c - self.cx) / self.fx, 0)
        y = np.where(valid, z * (r - self.cy) / self.fy, 0)
        return np.dstack((x, y, z))


def callback(image,depth):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')

    bridgeDepth = CvBridge()
    cv_depthImage = bridgeDepth.imgmsg_to_cv2(depth, "16UC1")

    pc = point_cloud()






def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('colourListener', anonymous=True)

    # image_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image)
    # depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image)
    
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10,0.1,allow_headerless=True)
    
    ts.registerCallback(callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
