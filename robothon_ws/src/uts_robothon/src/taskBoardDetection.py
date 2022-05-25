#!/usr/bin/env python



from cmath import pi
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32
import message_filters
from std_msgs.msg import Float32MultiArray
import math
#####################################################################################################

index = 50
Vc_array = np.zeros([index,6])
vcCount = 0
angle_flag = False
stop_count = 0
stop_flag = False
angle_flag_announced = False

#####################################################################################################

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
        self.colour_= 4
        self.setColour()
    def setRed(self):
        self.colour_ = 2
        self.setColourRed()
    def setGreen(self):
        self.colour_ = 3
        self.setColour()

    def setColour(self):
        if self.colour_ == 3:
            self.channel1Min_ = 100
            self.channel1Max_ = 150
            self.channel2Min_ = 140
            self.channel2Max_ = 255
            self.channel3Min_ = 0
            self.channel3Max_ = 90
            lower_channel = np.array([self.channel1Min_, self.channel2Min_, self.channel3Min_])
            upper_channel = np.array([self.channel1Max_, self.channel2Max_, self.channel3Max_])
            self.mask_ = cv.inRange(self.I_, lower_channel, upper_channel)
        if self.colour_ == 4:
            self.channel1Min_ = 100
            self.channel1Max_ = 150
            self.channel2Min_ = 190
            self.channel2Max_ = 255
            self.channel3Min_ = 110
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

#####################################################################################################

class Depth_h:
    fdx = 0
    fdy = 0
    u0 = 0
    v0 = 0
    h = 0
    w = 0
    def __init__(self, fdx,fdy,u0,v0,h,w):
        self.setExtrinsic(fdx,fdy,u0,v0,h,w)

    def setExtrinsic(self,fdx,fdy,u0,v0,h,w):
        self.fdx = fdx
        self.fdy = fdy
        self.u0 = u0
        self.v0 = v0 
        self.h = h
        self.w = w

    def getGeometry(self,x,y,z):

        valid =  z > 1
        x_ = np.where(valid, z * (x - self.u0) / self.fdx, 0)
        y_ = np.where(valid, z * (y - self.v0) / self.fdy, 0)
        return np.array([x_/1000,y_/1000,z/1000])
        
#####################################################################################################

def filterData(data_array):
    vx = 0
    vy = 0
    vz = 0
    wx = 0
    wy = 0
    wz = 0
    for i in range(0,len(data_array[:][1])):
        vx = vx + data_array[i][0]
    for i in range(0,len(data_array[:][1])):
        vy = vy + data_array[i][1]
    for i in range(0,len(data_array[:][1])):
        vz = vz + data_array[i][2]
    for i in range(0,len(data_array[:][1])):
        wx = wx + data_array[i][3]
    for i in range(0,len(data_array[:][1])):
        wy = wy + data_array[i][4]
    for i in range(0,len(data_array[:][1])):
        wz = wz + data_array[i][5]
    myArray = [vx,vy,vz,wx,wy,wz]
    newArray = [i/len(data_array[:][1]) for i in myArray]
    return newArray

#####################################################################################################

def rotationChecker(xr,yr,xb,yb):
    try:
        desired_angle  = 0.39
        computed_angle = math.atan2((yb-yr),(xr - xb))-desired_angle
    except:
        computed_angle = 0
        print('error in rotation checker')
    return computed_angle

#####################################################################################################

def callback(image, depth,box_centre):
    global vcCount
    global Vc_array
    global index
    global angle_flag
    global stop_count
    global stop_flag
    global angle_flag_announced
    #####################################################################################################

    bridgeRGB = CvBridge()
    RGB = bridgeRGB.imgmsg_to_cv2(image, "bgr8")

    #####################################################################################################

    Obj = DetectandDraw(RGB)
    Obj.setRed()
    Blue_Obj = DetectandDraw(RGB)
    Blue_Obj.setBlue()
    Green_Obj = DetectandDraw(RGB)
    Green_Obj.setRed()

    copy_RGB= RGB.copy()

    #####################################################################################################
    try:
        contours = Obj.getContours()
        
        temp_area = 0
        selected_contour=0
        tmp = 0
        for contour in contours:
            if cv.contourArea(contour) > temp_area:
                temp_area = cv.contourArea(contour)
                selected_contour = contour
        tmp = cv.approxPolyDP(selected_contour,3,True)
        temp_rect = cv.minAreaRect(tmp)
        box = cv.boxPoints(temp_rect)
        box = np.int0(box)
        cv.drawContours(copy_RGB,[box],0,(0,0,255),2)
        # print('space')
        M = cv.moments(selected_contour)
        x_centre = int(M["m10"]/M["m00"])
        y_centre = int(M["m01"]/M["m00"])
        
    except:
        x_centre = 0
        y_centre = 0
        x = 0
        y = 0
        w = 0
        h = 0
        print('error in red contour')
    #####################################################################################################

    try:
        Bcontours = Blue_Obj.getContours()
        Btemp_area = 0
        Bselected_contour=0
        Btmp = 0
        for contour in Bcontours:
            if cv.contourArea(contour) > Btemp_area:
                Btemp_area = cv.contourArea(contour)
                Bselected_contour = contour
        Btmp = cv.approxPolyDP(Bselected_contour,3,True)
        Btemp_rect = cv.minAreaRect(Btmp)
        Bbox = cv.boxPoints(Btemp_rect)
        Bbox = np.int0(Bbox)
        # cv.drawContours(copy_RGB,[Bbox],0,(0,0,255),2)
        Bx,By,Bw,Bh = cv.boundingRect(Bselected_contour)

        BM = cv.moments(Bselected_contour)
        Bx_centre = int(BM["m10"]/BM["m00"])
        By_centre = int(BM["m01"]/BM["m00"])
        cv.rectangle(copy_RGB, (Bx, By), (Bx+Bw, By+Bh), (0, 0, 255), 2)
    except:
        Bx_centre = 0
        By_centre = 0
        Bx = 0
        By = 0
        Bw = 0
        Bh = 0
        print('error in blue contour')

    #####################################################################################################
    try:
        Gcontours = Green_Obj.getContours()
        
        Gtemp_area = 0
        Gselected_contour=0
        Gtmp = 0
        for contour in Gcontours:
            if cv.contourArea(contour) > Gtemp_area and cv.contourArea(contour) != temp_area:
                Gtemp_area = cv.contourArea(contour)
                Gselected_contour = contour
        Gtmp = cv.approxPolyDP(Gselected_contour,3,True)
        Gtemp_rect = cv.minAreaRect(Gtmp)
        Gbox = cv.boxPoints(Gtemp_rect)
        Gbox = np.int0(Gbox)
        # cv.drawContours(copy_RGB,[Gbox],0,(0,0,255),2)
        Gx,Gy,Gw,Gh = cv.boundingRect(Gselected_contour)
        GM = cv.moments(Gselected_contour)
        Gx_centre = int(GM["m10"]/GM["m00"])
        Gy_centre = int(GM["m01"]/GM["m00"])
        cv.rectangle(copy_RGB, (Gx, Gy), (Gx+Gw, Gy+Gh), (0, 0, 255), 2)
    except:
        Gx_centre = 0
        Gy_centre = 0
        Gx = 0
        Gy = 0
        Gw = 0
        Gh = 0
        print('error in green contour')


    ########################################################################################################################################
    obs_corner_1 = np.array([Gx_centre,Gy_centre]) 
    obs_corner_2 = np.array([x_centre,y_centre]) 
    obs_corner_3 = np.array([Bx_centre,By_centre])
    triangle = np.array([[obs_corner_1, obs_corner_2, obs_corner_3]], np.int32)
    cv.polylines(copy_RGB, [triangle], True, (0,255,0), thickness=2)

    target_corner_1 = np.array([950,528]) 
    target_corner_2 = np.array([346,528])                          
    target_corner_3 = np.array([346,155])
    triangle_1 = np.array([[target_corner_1, target_corner_2, target_corner_3]], np.int32)
    cv.polylines(copy_RGB, [triangle_1], True, (255,0,0), thickness=2)

    ###########################################################################################################################################

    computed_angle = rotationChecker(x_centre,y_centre,Bx_centre,By_centre)
    fdx = 915.1260
    fdy = 913.8952

    u0 = 647.3002
    v0 = 349.3798
    #####################################################################################################
    bridgeDepth = CvBridge()
    depthImage = bridgeDepth.imgmsg_to_cv2(depth, "32FC1" )
    #####################################################################################################
    tb = Depth_h(fdx,fdy,u0,v0,720,1280)
    world_coor = tb.getGeometry(box_centre.x,box_centre.y,depthImage[y_centre][x_centre])
    desired_height = 0.336
    computed_height = world_coor[2]-desired_height
    if world_coor[2] < 0.4:
        world_coor[2] = 0
    
    # target_corner_1 = np.array([950,528]) 
    # target_corner_2 = np.array([346,528])                          
    # target_corner_3 = np.array([346,155])
    # target_corner_4 = np.array([950,155])
    # cv.rectangle(copy_RGB, (target_corner_3[0], target_corner_3[1]), (target_corner_1[0], target_corner_1[1]), (0, 0, 255), 2)

    cv.imshow("bounding_box", copy_RGB)
    cv.waitKey(2)

    Vc = np.zeros(6)
    
#############################################################################################################################
    # if abs(computed_angle) < 0.08:
    #     angle_flag = True
    #     if angle_flag_announced == False:
    #         print("angle flag raised------------------------------------------------------------------")
    #         angle_flag_announced = True
    #     try:
    #         Vc = np.zeros(6)
    #         Vc[0] = world_coor[0]
    #         Vc[1] = world_coor[1]
    #         Vc[2] = computed_height
    #     except:
    #         Vc = np.zeros(6)
    #         print('error in Vc')
    # elif angle_flag == False:   
    #     Vc = np.zeros(6)
    #     Vc[5] = -computed_angle
###############################################################################################################################
    angle_flag = True
    try:
        Vc = np.zeros(6)
        Vc[0] = world_coor[0]
        Vc[1] = world_coor[1]
        Vc[2] = computed_height
        Vc[5] = -computed_angle
    except:
        Vc = np.zeros(6)
        print('error in Vc')


###############################################################################################################################
    for i in range(0,len(Vc)):
            if abs(Vc[i])<0.007:
                Vc[i] = float(0)

    xy = [float(x_centre),float(y_centre)]
    Vc_array[:][vcCount] = Vc
    vcCount = vcCount + 1
    if vcCount == index:
        vcCount = 0
        newVc = filterData(Vc_array)
        if abs(newVc[0]) < 0.007 and abs(newVc[1]) <0.007 and abs(newVc[2]) < 0.007 and angle_flag == True:
            stop_count = stop_count + 1
        else:
            stop_count = stop_count - 1

        if stop_count < 0:
            stop_count = 0

        if stop_count == 2:
            stop_flag = True
            print("stop talking flag raised ---------------------------------------------------")
        if stop_flag == False:
            talker(newVc,xy)


###########################################################################################################################################

            
def talker(Vc,xy):
    global vcCount
    global Vc_array
    global index
    pub = rospy.Publisher('ColourDetectionChatter', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    array_ = ([Vc[0],Vc[1],Vc[2],Vc[3],Vc[4],Vc[5],xy[0],xy[1]])
    
    pub_data = Float32MultiArray(data = array_)
    rospy.loginfo(pub_data)
    pub.publish(pub_data)  
    Vc_array = np.zeros([index,6])
    rate.sleep()   
    
    

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('colourDetection', anonymous=True)

    # image_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image)
    # depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image)
    
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    box_corner_sub = message_filters.Subscriber('/center', Point32)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub,box_corner_sub], 10,0.1,allow_headerless=True)
    
    ts.registerCallback(callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
