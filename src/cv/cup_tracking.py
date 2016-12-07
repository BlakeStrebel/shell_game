#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from shell_game.msg import Treasure
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance

class cup:
    def __init__(self):
        self.containsTreasure = False
        self.originalPoint = 0
        self.currentPoint = 0

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.find_cups)
        self.treasure_location_sub = rospy.Subscriber("/treasure_location",Treasure,self.find_treasure)
        self.treasure_cup_pub = rospy.Publisher("treasure_cup_location",Point,queue_size=10)
        self.image_tracking_pub = rospy.Publisher("/robot/xdisplay",Image,queue_size=10)
        self.treasure_cup_location = Point()
        self.cups = []
        self.cupCenters = [[0,0],[0,0],[0,0]]
        self.wasPreviouslyTrue = False
        self.flag = False
        self.minRadius = 10
        for i in range(0,3):
            self.cups.append(cup())
            print self.cups[i].containsTreasure

        # self.cups[0].containsTreasure = True

    def find_cups(self,data):
        try:
            imgOriginal = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print("==[CAMERA MANAGER]==",e)

        blurred = cv2.GaussianBlur(imgOriginal,(11,11),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # lower = np.array([0,130,80])    # red hsv range
        # upper = np.array([20,190,255])
        lower = np.array([0,100,100])    # red hsv range
        upper = np.array([3,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=7)
        mask = cv2.dilate(mask, None, iterations=7)
        output = cv2.bitwise_and(imgOriginal, imgOriginal, mask = mask)
        outputGrayscale = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        if major_ver == '3':
            contours = cv2.findContours(outputGrayscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]
        elif major_ver == '2':
            contours = cv2.findContours(outputGrayscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]

        radius = [0,0,0]
        x = [[0,0],[0,0],[0,0]]
        y = [[0,0],[0,0],[0,0]]
        if len(contours) > 2:
            contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
            for i in range(0,3):
                ((x[i],y[i]),radius[i]) = cv2.minEnclosingCircle(contours[i])
            # this will ensure that it is the 3 separate cups that are being identified
            # may have to change the radius threshold when testing on baxter
            if radius[0] > self.minRadius and radius[1] > self.minRadius and radius[2] > self.minRadius:
                for i in range(0,3):
                    M = cv2.moments(contours[i])
                    self.cupCenters[i] = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
                if self.flag is False:
                    print "RESET"
                    for i in range(0,3):
                        self.cups[i].originalPoint = self.cupCenters[i]
                        self.cups[i].currentPoint = self.cupCenters[i]
                    self.flag = True
                else:
                    for i in range(0,3):
                        self.cups[i].currentPoint = self.cupCenters[closestPoint(self.cups[i].currentPoint,self.cupCenters)]
                        if i==0:
                            cv2.circle(imgOriginal,(int(self.cups[i].currentPoint[0]),int(self.cups[i].currentPoint[1])),int(radius[i]),(255,0,0),2)
                        if i==1:
                            cv2.circle(imgOriginal,(int(self.cups[i].currentPoint[0]),int(self.cups[i].currentPoint[1])),int(radius[i]),(0,255,0),2)
                        if i==2:
                            cv2.circle(imgOriginal,(int(self.cups[i].currentPoint[0]),int(self.cups[i].currentPoint[1])),int(radius[i]),(0,0,255),2)
                        cv2.circle(imgOriginal,self.cups[i].currentPoint,5,(0,0,255),-1)
                        print "Cup 1 Current Location:",self.cups[0].currentPoint
                        print "Cup 2 Current Location:",self.cups[1].currentPoint
                        print "Cup 3 Current Location:",self.cups[2].currentPoint
        else:
            self.flag = False

        for i in range(0,3):
            if self.cups[i].containsTreasure:
                print "Cup #",i+1,"CONTAINS TREASURE"
                self.treasure_cup_location.x = self.cups[i].currentPoint[0] # might not need 640 minus
                self.treasure_cup_location.y = self.cups[i].currentPoint[1]
                self.treasure_cup_pub.publish(self.treasure_cup_location)
        cv2.imshow("MyImage", output)
        cv2.imshow("Image", imgOriginal)
        cv2.waitKey(3)

    def find_treasure(self,data):
        if data.flag is False and self.wasPreviouslyTrue is True:
            for i in range(0,3):
                self.cups[i].containsTreasure = False
            treasureCupIndex = closestPoint((data.x,data.y),(self.cups[0].currentPoint,self.cups[1].currentPoint,self.cups[2].currentPoint))
            self.cups[treasureCupIndex].containsTreasure = True
            self.wasPreviouslyTrue = False
            print "CUP #",treasureCupIndex+1,"contains treasure!!!"
        if data.flag is True and self.wasPreviouslyTrue is False:
            self.wasPreviouslyTrue = True

def closestPoint(currentPoint,cupCenters):
    minimumDist = distance.euclidean(currentPoint,cupCenters[0])
    mindex = 0
    for i in range(1,3):
        dist = distance.euclidean(currentPoint,cupCenters[i])
        if dist < minimumDist:
            minimumDist = dist
            mindex = i
    print "MINDEX:",mindex
    return mindex

def main(args):
    rospy.init_node('image_converter')
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    print "OpenCV Major Version:",major_ver
    main(sys.argv)
