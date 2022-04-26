#!/usr/bin/env python
from ctypes import sizeof
from tkinter.tix import Tree
import numpy.matlib
from unittest import result
from sensor_msgs.msg import Image
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PointStamped
import message_filters


class PointCloud:

    def __init__(self):
        self.fdx = 422.3378
        self.fdy = 422.5609
        self.u0 = 424
        self.v0 = 240
        self.h = 480
        self.w = 848

    def getPointCloud(self, depth):
        rows = np.arange(start = 0, stop = self.w)
        cols = np.arange(start = 0, stop = self.h)
    
        # c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        # valid = (depth > 0) & (depth < 255)

        u = np.matlib.repmat(rows,self.h,1)
        v = np.matlib.repmat(cols,self.w,1)
        v = np.rot90(v, k = 1)
        v = np.flipud(v)
        
        valid =  depth>1
        z = np.where(valid, depth, np.nan)
        x = np.where(valid, z * (u - self.u0) / self.fdx, 0)
        y = np.where(valid, z * (v - self.v0) / self.fdy, 0)
        
        return np.dstack((x, y, z))



class CheckCollision:
    def __init__(self):
        self.check = False
    
    def isCollision(self,EEtr_current,EEtr_next,pClouds):
        count = 0
        for pc_x in  pClouds:
            for pc in pc_x:
                if pc[2] < 500:
                    tr_pc = np.zeros((4,4))
                    tr_pc[0][3] = pc[0]
                    tr_pc[1][3] = pc[1]
                    tr_pc[2][3] = pc[2]
                    tr_current = EEtr_current*tr_pc
                    
                    tr_next = EEtr_next
                    
                    if (abs(tr_current[0][3] - tr_next[0][3]) < 50) and (abs(tr_current[1][3] - tr_next[1][3]) < 50) and (abs(tr_current[2][3] - tr_next[2][3]) < 50):
                        self.check = True
        return self.check
        



# def callback(image,depth):
#     bridge = CvBridge()
#     cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')

#     bridgeDepth = CvBridge()
#     cv_depthImage = bridgeDepth.imgmsg_to_cv2(depth, "16UC1")

#     pc = point_cloud()





def callback(depth):

    bridgeDepth = CvBridge()
    cv_depthImage = bridgeDepth.imgmsg_to_cv2(depth, "16UC1")
    # if cv_depthImage[240][424]<300:
    #     print("true")
    # else:
    #     print("false")
    pc = PointCloud()
    pClouds = pc.getPointCloud(cv_depthImage)
    
    # cv.putText(img = cv_depthImage, text = 'Hello', org=(479,239),fontFace=cv.FONT_HERSHEY_TRIPLEX, fontScale=3, color=(0, 255, 0),thickness=3)
    # cv.imshow("test",cv_depthImage)
    # cv.waitKey(1)
    EEtr_current = np.zeros((4,4))
    EEtr_next = np.zeros((4,4))
    EEtr_current[2][3] = 1
    EEtr_next[2][3] = 300
    check_collision = CheckCollision()
    print(check_collision.isCollision(EEtr_current,EEtr_next,pClouds))
    
    
    # cv.imshow("bounding_box", pt)
    # cv.waitKey(1)
    # print(cv_depthImage[240][424])




def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('colourListener', anonymous=True)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, callback)
    # image_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image)
    # depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image)
    
    # image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    # depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
    
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10,0.1,allow_headerless=True)
    
    # ts.registerCallback(callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
