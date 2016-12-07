#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from shell_game.msg import Treasure
from cv_bridge import CvBridge, CvBridgeError

class treasure_finder:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
        self.pub = rospy.Publisher("treasure_location",Treasure, queue_size=10)
        self.treasurePoint = Treasure()

    def callback(self,data):
        try:
            imgOriginal = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("==[CAMERA MANAGER]==", e)

        blurred = cv2.GaussianBlur(imgOriginal,(11,11),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower = np.array([60,90,70])    # hsv range for green
        upper = np.array([90,175,255])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=7)
        mask = cv2.dilate(mask, None, iterations=7)
        output = cv2.bitwise_and(imgOriginal, imgOriginal, mask = mask)
        outputGrayscale = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        if major_ver == '3':
            contours = cv2.findContours(outputGrayscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]
        elif major_ver == '2':
            contours = cv2.findContours(outputGrayscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]

        if len(contours) > 0:
            c = max(contours,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            treasureCenter = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.treasurePoint.x = treasureCenter[0]
            self.treasurePoint.y = treasureCenter[1]
            self.treasurePoint.flag = 1
            self.pub.publish(self.treasurePoint)
        else:
            self.treasurePoint.flag = 0
            self.pub.publish(self.treasurePoint)


def main(args):
    rospy.init_node('find_treasure')
    ic = treasure_finder()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    print "OpenCV Major Version:",major_ver
    main(sys.argv)
