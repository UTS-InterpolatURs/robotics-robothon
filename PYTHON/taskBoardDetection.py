#!/usr/bin/env python



from cmath import pi



from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PointStamped
import message_filters
from std_msgs.msg import Float32MultiArray
import math

index = 15
Vc_array = np.zeros([index,6])
vcCount = 0

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
        

def FuncLx(x,y,Z):
    Lx = np.zeros((2,6))
    if Z != 0:
        Lx[0][0] = -1/Z
        Lx[0][1] = 0
        Lx[0][2] = x/Z
        Lx[0][3] = x*y
        Lx[0][4] = -(1+x*x)
        Lx[0][5] = y

        Lx[1][0] = 0
        Lx[1][1] = -1/Z
        Lx[1][2] = y/Z
        Lx[1][3] = 1+y*y
        Lx[1][4] = -x*y
        Lx[1][5] = -x
    return Lx


def VServoing(target, obs, Z, lambda_):
    try:
        Lx =  np.zeros((len(target)*2,6))
        index = len(target)
        count = 0
        for i in range (0,index):
            Lx1 = FuncLx(obs[i][0],obs[i][1],Z[i])
            Lx[i+count] = Lx1[0][:]
            Lx[i+1+count] = Lx1[1][:]
            count = count + 1
        e2 = obs - target
        e = e2.flatten()
        transpose_Lx = Lx.transpose()
        Lx2 = np.dot(np.linalg.inv(np.dot(transpose_Lx,Lx)),transpose_Lx)
        Vc = -lambda_*np.dot(Lx2,e)
    except:
        Vc = np.zeros(6)
        print('error in Vservoing')
    return Vc

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

def rotationChecker(xr,yr,xb,yb):
    try:
        desired_angle  = 0.3653
        computed_angle = math.atan2((yb-yr),(xr - xb))-desired_angle
    except:
        computed_angle = 0
        print('error in rotation checker')
    return computed_angle

def edgeDetection(image):
    gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    gray_uint8 = np.uint8(gray)
    dst = cv.Canny(gray_uint8,100,200)
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    count = 0
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
            count = count +1
    print(count)
    # cdstP = cv.cvtColor(cdstP,cv.COLOR_BGR2GRAY)
    
    # contours = cv.findContours(dst, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # contours = contours[0] if len(contours) == 2 else contours[1]
    # for c in contours:
    #     cv.drawContours(image,[c], 0, (0,255,0), 3)
    # temp_area = 0
    # selected_contour=0
    # for contour in contours:
    #     if cv.contourArea(contour) > temp_area:
    #         temp_area = cv.contourArea(contour)
    #         selected_contour = contour
    # x,y,w,h = cv.boundingRect(selected_contour)
    # print(x,y,w,h)
    # M = cv.moments(selected_contour)
    # x_centre = int(M["m10"]/M["m00"])
    # y_centre = int(M["m01"]/M["m00"])
    # cv.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)

    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdst)
    cv.waitKey(2)





def callback(image, depth):
    global vcCount
    global Vc_array
    global index
    bridgeRGB = CvBridge()
    RGB = bridgeRGB.imgmsg_to_cv2(image, "bgr8")
    # edgeDetection(RGB)

    # gray = cv.cvtColor(RGB,cv.COLOR_BGR2GRAY)
    # gray_float32 = np.float32(gray)
    # gray_uint8 = np.uint8(gray)
    # dst = cv.cornerHarris(gray_float32,2,3,0.2)
    # dst = cv.dilate(dst,None)
    # ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
    # dst = np.uint8(dst)
    # dst = cv.Canny(gray_uint8,100,200)
    
    # cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    # cdstP = np.copy(cdst)
    
    # lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    
    # if lines is not None:
    #     for i in range(0, len(lines)):
    #         rho = lines[i][0][0]
    #         theta = lines[i][0][1]
    #         a = math.cos(theta)
    #         b = math.sin(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    #         pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    #         cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    
    
    # linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    # if linesP is not None:
    #     for i in range(0, len(linesP)):
    #         l = linesP[i][0]
    #         cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)

    # contours = cv.findContours(cdstP, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # contours = contours[0] if len(contours) == 2 else contours[1]



    # cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    # cv.waitKey(2)


    Obj = DetectandDraw(RGB)
    Obj.setRed()
    Blue_Obj = DetectandDraw(RGB)
    Blue_Obj.setBlue()

    Green_Obj = DetectandDraw(RGB)
    Green_Obj.setRed()


    copy_RGB= RGB.copy()

    # test_RGB = RGB.copy()
    # mask = Blue_Obj.getMask()
    # res = cv.bitwise_and(copy_RGB,copy_RGB,mask = mask)
    # cv.imshow("Image Window", res)
    # cv.imshow("RGB",copy_RGB)
    # cv.waitKey(2)
    
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
        x,y,w,h = cv.boundingRect(selected_contour)
        M = cv.moments(selected_contour)
        x_centre = int(M["m10"]/M["m00"])
        y_centre = int(M["m01"]/M["m00"])
        # print(x_centre,y_centre)
        # cv.rectangle(copy_RGB, (x, y), (x+w, y+h), (0, 0, 255), 2)
    except:
        x_centre = 0
        y_centre = 0
        x = 0
        y = 0
        w = 0
        h = 0
        print('error in red contour')

    # print(x_centre,y_centre)
    # cv.imshow("bounding_box", edges)
    # cv.waitKey(2)
    # print(x,y,w,h)

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
    # print(Bx_centre,By_centre)

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
        # print('space')
        Gx,Gy,Gw,Gh = cv.boundingRect(Gselected_contour)
        GM = cv.moments(Gselected_contour)
        Gx_centre = int(GM["m10"]/GM["m00"])
        Gy_centre = int(GM["m01"]/GM["m00"])
        # print(x_centre,y_centre)
        cv.rectangle(copy_RGB, (Gx, Gy), (Gx+Gw, Gy+Gh), (0, 0, 255), 2)
    except:
        Gx_centre = 0
        Gy_centre = 0
        Gx = 0
        Gy = 0
        Gw = 0
        Gh = 0
        print('error in green contour')


###########################################################################################################################################
    #test

    computed_angle = rotationChecker(x_centre,y_centre,Bx_centre,By_centre)
    fdx = 918.7401
    fdy = 918.3084

    u0 = 647.2181
    v0 = 345.8296

    
    

    bridgeDepth = CvBridge()
    depthImage = bridgeDepth.imgmsg_to_cv2(depth, "32FC1" )


    print(depthImage[719][1279])
    tb = Depth_h(fdx,fdy,u0,v0,720,1280)
    world_coor = tb.getGeometry(x_centre,y_centre,depthImage[y_centre][x_centre])
    desired_height = 0.4
    computed_height = world_coor[2]-desired_height
    if world_coor[2] < 0.4:
        world_coor[2] = 0
    # print(world_coor[2])




    target_corner_1 = np.array([498,464]) 
    target_corner_2 = np.array([912,278])                          
    target_corner_3 = np.array([463,464])
    # target_corner_1 = np.array([Gx_centre,Gy_centre]) 
    # target_corner_2 = np.array([x_centre,y_centre]) 
    # target_corner_3 = np.array([Bx_centre,By_centre])

    # print(target_corner_1,target_corner_2,target_corner_3)
    triangle_1 = np.array([[target_corner_1, target_corner_2, target_corner_3]], np.int32)
    cv.polylines(copy_RGB, [triangle_1], True, (255,0,0), thickness=2)

    obs_corner_1 = np.array([Gx_centre,Gy_centre]) 
    obs_corner_2 = np.array([x_centre,y_centre]) 
    obs_corner_3 = np.array([Bx_centre,By_centre])

    triangle = np.array([[obs_corner_1, obs_corner_2, obs_corner_3]], np.int32)
    cv.polylines(copy_RGB, [triangle], True, (0,255,0), thickness=2)

    cv.imshow("bounding_box", copy_RGB)
    cv.waitKey(2)

    x_ = np.array([(target_corner_1[0]-u0)/fdx,(target_corner_2[0]-u0)/fdx,(target_corner_3[0]-u0)/fdx]) #change here
    y_ = np.array([(target_corner_1[1]-v0)/fdy,(target_corner_2[1]-v0)/fdy,(target_corner_3[1]-v0)/fdy])

    x_obs = np.array([(obs_corner_1[0]-u0)/fdx,(obs_corner_2[0]-u0)/fdx,(obs_corner_3[0]-u0)/fdx]) #change here
    y_obs = np.array([(obs_corner_1[1]-v0)/fdy,(obs_corner_2[1]-v0)/fdy,(obs_corner_3[1]-v0)/fdy]) 

    lambda_ = 0.6
    if abs(computed_angle) < 0.1:
        try:
            # Z = np.array([depthImage[obs_corner_1[1]][obs_corner_1[0]],depthImage[obs_corner_2[1]][obs_corner_2[0]],depthImage[obs_corner_3[1]][obs_corner_3[0]]]) #change here
            # myZ = Z
            # newZ = [i/1000 for i in myZ]
            # # print(newZ)
            # Target = np.array([[x_[0],y_[0]],[x_[1],y_[1]],[x_[2],y_[2]]])
            # Obs = np.array([[x_obs[0],y_obs[0]],[x_obs[1],y_obs[1]],[x_obs[2],y_obs[2]]])
            # Vc = VServoing(Target,Obs,newZ,lambda_)
            # Vc[3] = 0
            # Vc[4] = 0
            Vc = np.zeros(6)
            Vc[0] = world_coor[0]
            Vc[1] = world_coor[1]
            Vc[2] = computed_height
            # print(Vc[2])



        except:
            Vc = np.zeros(6)
            print('error in Vc')
    else:   
        Vc = np.zeros(6)
        Vc[5] = -computed_angle



    for i in range(0,len(Vc)):
            if abs(Vc[i])<0.005:
                Vc[i] = float(0)
    xy = [float(x_centre),float(y_centre)]
    Vc_array[:][vcCount] = Vc
    vcCount = vcCount + 1
    if vcCount == index:
        vcCount = 0
        newVc = filterData(Vc_array)
        talker(newVc,xy)


###########################################################################################################################################




###########################################################################################################################################
    #test
    # computed_angle = rotationChecker(x_centre,y_centre,Bx_centre,By_centre)
    # fdx = 918.7401
    # fdy = 918.3084

    # u0 = 647.2181
    # v0 = 345.8296

    # bridgeDepth = CvBridge()
    # depthImage = bridgeDepth.imgmsg_to_cv2(depth, "32FC1" )
    # target_corner_1 = np.array([416,249]) 
    # target_corner_2 = np.array([885,293])                          
    # target_corner_3 = np.array([428,473])
    # # print(target_corner_1,target_corner_2,target_corner_3)
    # triangle = np.array([[target_corner_1, target_corner_2, target_corner_3]], np.int32)
    # cv.polylines(copy_RGB, [triangle], True, (255,0,0), thickness=3)

    # obs_corner_1 = np.array([Gx_centre,Gy_centre]) 
    # obs_corner_2 = np.array([x_centre,y_centre]) 
    # obs_corner_3 = np.array([Bx_centre,By_centre])

    # triangle = np.array([[obs_corner_1, obs_corner_2, obs_corner_3]], np.int32)
    # cv.polylines(copy_RGB, [triangle], True, (0,255,0), thickness=3)

    # cv.imshow("bounding_box", copy_RGB)
    # cv.waitKey(2)

    # x_ = np.array([(target_corner_1[0]-u0)/fdx,(target_corner_2[0]-u0)/fdx,(target_corner_3[0]-u0)/fdx]) #change here
    # y_ = np.array([(target_corner_1[1]-v0)/fdy,(target_corner_2[1]-v0)/fdy,(target_corner_3[1]-v0)/fdy])

    # x_obs = np.array([(obs_corner_1[0]-u0)/fdx,(obs_corner_2[0]-u0)/fdx,(obs_corner_3[0]-u0)/fdx]) #change here
    # y_obs = np.array([(obs_corner_1[1]-v0)/fdy,(obs_corner_2[1]-v0)/fdy,(obs_corner_3[1]-v0)/fdy]) 

    # lambda_ = 0.6
    # if abs(computed_angle) < 0.1:
    #     try:
    #         Z = np.array([depthImage[obs_corner_1[1]][obs_corner_1[0]],depthImage[obs_corner_2[1]][obs_corner_2[0]],depthImage[obs_corner_3[1]][obs_corner_3[0]]]) #change here
    #         myZ = Z
    #         newZ = [i/1000 for i in myZ]
    #         # print(newZ)
    #         Target = np.array([[x_[0],y_[0]],[x_[1],y_[1]],[x_[2],y_[2]]])
    #         Obs = np.array([[x_obs[0],y_obs[0]],[x_obs[1],y_obs[1]],[x_obs[2],y_obs[2]]])
    #         Vc = VServoing(Target,Obs,newZ,lambda_)
    #     except:
    #         Vc = np.zeros(6)
    #         print('error in Vc')
    # else:   
    #     Vc = np.zeros(6)
    #     Vc[5] = computed_angle

    # xy = [float(x_centre),float(y_centre)]
    # Vc_array[:][vcCount] = Vc
    # vcCount = vcCount + 1
    # if vcCount == index:
    #     vcCount = 0
    #     newVc = filterData(Vc_array)
    #     talker(newVc,xy)


###########################################################################################################################################

    # computed_angle = rotationChecker(x_centre,y_centre,Bx_centre,By_centre)
    # # print(computed_angle)


    # fdx = 918.7401
    # fdy = 918.3084

    # u0 = 647.2181
    # v0 = 345.8296

    # # h = 480
    # # w = 848
    
    # # fdx = 554.2547
    # # fdy = 554.2547

    # # u0 = 320.5
    # # v0 = 240.5

    # # h = 480
    # # w = 640
    # # print(box)

    # bridgeDepth = CvBridge()
    # depthImage = bridgeDepth.imgmsg_to_cv2(depth, "32FC1" )
    # target_corner_1 = np.array([860+2,356-2]) 
    # target_corner_2 = np.array([860+2,233+2])                          
    # target_corner_3 = np.array([927-2,223+2])
    # target_corner_4 = np.array([927-2,356-2])

    # cv.rectangle(copy_RGB, (target_corner_2[0], target_corner_2[1]), (target_corner_4[0], target_corner_4[1]), (0, 0, 255), 2)
    # cv.imshow("bounding_box", copy_RGB)
    # cv.waitKey(2)

    # # obs_corner_1 = np.array([x,y])
    # # obs_corner_2 = np.array([x+w,y])                        
    # # obs_corner_3 = np.array([x,y+h])
    # # obs_corner_4 = np.array([x+w,y+h])
    # obs_corner_1 = box[0]
    # obs_corner_2 = box[1]                    
    # obs_corner_3 = box[2]
    # obs_corner_4 = box[3]

    # x_ = np.array([(target_corner_1[0]-u0)/fdx,(target_corner_2[0]-u0)/fdx,(target_corner_3[0]-u0)/fdx,(target_corner_4[0]-u0)/fdx]) #change here
    # y_ = np.array([(target_corner_1[1]-v0)/fdy,(target_corner_2[1]-v0)/fdy,(target_corner_3[1]-v0)/fdy,(target_corner_4[1]-v0)/fdy])

    # x_obs = np.array([(obs_corner_1[0]-u0)/fdx,(obs_corner_2[0]-u0)/fdx,(obs_corner_3[0]-u0)/fdx,(obs_corner_4[0]-u0)/fdx]) #change here
    # y_obs = np.array([(obs_corner_1[1]-v0)/fdy,(obs_corner_2[1]-v0)/fdy,(obs_corner_3[1]-v0)/fdy,(obs_corner_4[1]-v0)/fdy]) 

    # lambda_ = 0.01
    
    # if abs(computed_angle) < 0.1:
    #     try:
    #         Z = np.array([depthImage[obs_corner_1[1]-2][obs_corner_1[0]+2],depthImage[obs_corner_2[1]+2][obs_corner_2[0]+2],depthImage[obs_corner_3[1]+2][obs_corner_3[0]-2],depthImage[obs_corner_4[1]-2][obs_corner_4[0]-2]]) #change here
    #         myZ = Z
    #         newZ = [i/1000 for i in myZ]
    #         print(newZ)
    #         Target = np.array([[x_[0],y_[0]],[x_[1],y_[1]],[x_[2],y_[2]],[x_[3],y_[3]]])
    #         Obs = np.array([[x_obs[0],y_obs[0]],[x_obs[1],y_obs[1]],[x_obs[2],y_obs[2]],[x_obs[3],y_obs[3]]])
    #         Vc = VServoing(Target,Obs,newZ,lambda_)
    #     except:
    #         Vc = np.zeros(6)
    #         print('error in Vc')
    # else:   
    #     Vc = np.zeros(6)
    #     Vc[5] = computed_angle
    # # for i in range(0,len(Vc)):
    # #         if abs(Vc[i])<0.005:
    # #             Vc[i] = float(0)

    
    # # for i in range(0,len(Vc_array[:][1])):
    # # newVc = filterData(Vc_array)
    # xy = [float(x_centre),float(y_centre)]
    # Vc_array[:][vcCount] = Vc
    # vcCount = vcCount + 1
    # if vcCount == index:
    #     vcCount = 0
    #     newVc = filterData(Vc_array)
    #     talker(newVc,xy)

    
    # # try:
    # #     Z = np.array([depthImage[obs_corner_1[1]][obs_corner_1[0]],depthImage[obs_corner_2[1]][obs_corner_2[0]],depthImage[obs_corner_3[1]][obs_corner_3[0]],depthImage[obs_corner_4[1]][obs_corner_4[0]]]) #change here
    # #     Target = np.array([[x_[0],y_[0]],[x_[1],y_[1]],[x_[2],y_[2]],[x_[3],y_[3]]])
    # #     Obs = np.array([[x_obs[0],y_obs[0]],[x_obs[1],y_obs[1]],[x_obs[2],y_obs[2]],[x_obs[3],y_obs[3]]])
    # #     Vc = VServoing(Target,Obs,Z,lambda_)
        
            
    # # except:
    # #     Vc = np.zeros(6)
    # # for i in range(0,len(Vc)):
    # #     if abs(Vc[i])<0.000001:
    # #         Vc[i] = float(0)

    # # print(Vc)
    # # xy = [float(x_centre),float(y_centre)]
    
    # # talker(Vc,xy)
    
    # # pub = rospy.Publisher('/colourChatter', PointStamped, queue_size=10)
    # # point = PointStamped()
    # # rate = rospy.Rate(10) 

    # # point.header.stamp = rospy.Time.now()
    # # point.header.frame_id = "/odom"
    # # point.point.x = x_centre
    # # point.point.y = y_centre
    # # point.point.z = 0
    # # pub.publish(point)
    # # rate.sleep()
            
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
    box_corner_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10,0.1,allow_headerless=True)
    
    ts.registerCallback(callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
