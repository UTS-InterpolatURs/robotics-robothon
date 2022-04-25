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


class DetectandDraw:

    channel1Min_ = 0
    channel1Max_ = 0
    channel2Min_ = 0
    channel2Max_ = 0
    channel3Min_ = 0
    channel3Max_ = 0
    mask_ = 0
    I_ = 0
    colour_ = 0

    def __init__(self, RGB):
        self.I_ = cv.cvtColor(RGB, cv.COLOR_BGR2HSV)
        lower_channel = np.array([self.channel1Min_, self.channel2Min_, self.channel3Min_])
        upper_channel = np.array([self.channel1Max_, self.channel2Max_, self.channel3Max_])
        self.mask_ = cv.inRange(self.I_, lower_channel, upper_channel)
    def setBlue(self):
        self.colour_=4
        self.setColour()
    def setRed(self):
        self.colour_ = 2
        self.setColourRed()
    def setGreen(self):
        self.colour_ = 3
        self.setColour()

    def setColour(self):
        if self.colour_ == 3:
            self.channel1Min_ = 50
            self.channel1Max_ = 80
            self.channel2Min_ = 51
            self.channel2Max_ = 255
            self.channel3Min_ = 0
            self.channel3Max_ = 255
            lower_channel = np.array([self.channel1Min_, self.channel2Min_, self.channel3Min_])
            upper_channel = np.array([self.channel1Max_, self.channel2Max_, self.channel3Max_])
            self.mask_ = cv.inRange(self.I_, lower_channel, upper_channel)
        if self.colour_ == 4:
            self.channel1Min_ = 100
            self.channel1Max_ = 140
            self.channel2Min_ = 150
            self.channel2Max_ = 255
            self.channel3Min_ = 0
            self.channel3Max_ = 255
            lower_channel = np.array([self.channel1Min_, self.channel2Min_, self.channel3Min_])
            upper_channel = np.array([self.channel1Max_, self.channel2Max_, self.channel3Max_])
            self.mask_ = cv.inRange(self.I_, lower_channel, upper_channel)



    def setColourRed(self):
        if self.colour_ == 2:
            self.channel1Min_ = 0
            self.channel1Max_ = 5
            self.channel2Min_ = 100
            self.channel2Max_ = 255
            self.channel3Min_ = 100
            self.channel3Max_ = 255
            channel1Min2_ = 175
            channel1Max2_ = 180
            channel2Min2_ = 100
            channel2Max2_ = 255
            channel3Min2_ = 100
            channel3Max2_ = 255
            lower_channel = np.array([self.channel1Min_, self.channel2Min_, self.channel3Min_])
            upper_channel = np.array([self.channel1Max_, self.channel2Max_, self.channel3Max_])
            lower_channel2 = np.array([channel1Min2_, channel2Min2_, channel3Min2_])
            upper_channel2 = np.array([channel1Max2_, channel2Max2_, channel3Max2_])
            mask1 = cv.inRange(self.I_, lower_channel, upper_channel)
            mask2 = cv.inRange(self.I_, lower_channel2, upper_channel2)
            self.mask_ = mask1|mask2

    def getMask(self):
        return self.mask_

    def getContours(self):

        contours = cv.findContours(self.mask_, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        return contours
        





def callback(image, depth):
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')
    Blue = DetectandDraw(cv_image)
    Blue.setRed()
    # mask = Blue.getMask()
    # res = cv.bitwise_and(cv_image,cv_image,mask = mask)
    # cv.imshow("Image Window", res)
    
    # cv.imshow("RGB",cv_image)
    result = cv_image.copy()
    try:
        contours = Blue.getContours()
        
        temp_area = 0
        selected_contour=0
        for contour in contours:
            if cv.contourArea(contour) > temp_area:
                temp_area = cv.contourArea(contour)
                selected_contour = contour
        x,y,w,h = cv.boundingRect(selected_contour)
        M = cv.moments(selected_contour)
        x_centre = int(M["m10"]/M["m00"])
        y_centre = int(M["m01"]/M["m00"])
        cv.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
    except:
        x_centre = 0
        y_centre = 0
        w = 0
        h = 0
    cv.imshow("bounding_box", result)
    cv.waitKey(1)


    # for cntr in contours:
    #         if cv.contourArea(cntr) < 
    #         x,y,w,h = cv.boundingRect(cntr)
    #         cv.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
    #         print(x,y)

    fdx = 420.3378
    fdy = 422.5609

    u0 = 424
    v0 = 240

    h = 480
    w = 848
    converted_x = 424 + (x_centre - 640)
    converted_y = 240 + (y_centre - 360)

    # fdx = 554.2547
    # fdy = 554.2547

    # u0 = 320.5
    # v0 = 240.5

    # h = 480
    # w = 640



    
    bridgeDepth = CvBridge()
    depthImage = bridgeDepth.imgmsg_to_cv2(depth, "16UC1")
    try:
        
        print(x_centre,y_centre)
    except:
        print('no size')

    

    try:
        # x_w = (depthImage[y_centre][x_centre]*(x_centre-u0))/fdx
        # y_w = -(depthImage[y_centre][x_centre]*(y_centre-v0))/fdy
        # z_w = depthImage[y_centre][x_centre]
        x_w = (depthImage[converted_y][converted_x]*(converted_x-u0))/fdx
        y_w = -(depthImage[converted_y][converted_x]*(converted_y-v0))/fdy
        z_w = depthImage[converted_y][converted_x]
    except:
        x_w = 0
        y_w = 0
        z_w = 0

    

    pub = rospy.Publisher('/colourChatter', PointStamped, queue_size=10)
    point = PointStamped()
    rate = rospy.Rate(10) 

    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/odom"
    point.point.x = x_w
    point.point.y = y_w
    point.point.z = z_w
    pub.publish(point)
    rate.sleep()
            
    
    
    

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
