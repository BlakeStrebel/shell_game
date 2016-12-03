#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.movement_data = rospy.Publisher("movement_data",Point,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.pts = deque(maxlen=32)
        self.deltaValues = Point()

    def callback(self,data):
        try:
            imgOriginal = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("==[CAMERA MANAGER]==", e)

        blurred = cv2.GaussianBlur(imgOriginal,(11,11),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower = np.array([0,100,100])
        upper = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=7)
        mask = cv2.dilate(mask, None, iterations=7)
        output = cv2.bitwise_and(imgOriginal, imgOriginal, mask = mask)
        outputGrayscale = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        contours = cv2.findContours(outputGrayscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]

        if len(contours) > 0:
            c = max(contours,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            ballCenter = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv2.circle(imgOriginal,(int(x),int(y)),int(radius),(0,255,0),2)
                cv2.circle(imgOriginal,ballCenter,5,(0,0,255),-1)
                self.pts.appendleft(ballCenter)
                # print "Ball Center:",ballCenter,
                print "Ball Center x:",ballCenter[0],
                print "Ball Center y:",ballCenter[1],
                print "Image Center x:",imgOriginal.shape[1]/2,
                print "Image Center y:",imgOriginal.shape[0]/2
                self.deltaValues.x = ballCenter[0] - imgOriginal.shape[1]/2 # if this is positive, we want to increase the value of panAngle
                self.deltaValues.y = imgOriginal.shape[0]/2 - ballCenter[1] # if this is positive, we want to increase the value of tiltAngle
                if self.deltaValues.x > 640 or self.deltaValues.x < -640:
                    self.deltaValues.x = 0
                if self.deltaValues.y > 480 or self.deltaValues.y < -480:
                    self.deltaValues.y = 0
                self.deltaValues.x *= (1008/640)
                self.deltaValues.y *= (1008/480)
                rospy.loginfo(self.deltaValues)
                self.movement_data.publish(self.deltaValues)
                for i in np.arange(1,len(self.pts)):
                    thickness = int(np.sqrt(32/float(i+1))*2.5)
                    cv2.line(imgOriginal,self.pts[i-1],self.pts[i],(0,0,255),thickness)

        flipped = cv2.flip(imgOriginal, 1)
        output = cv2.flip(output, 1)
        cv2.imshow("Image", flipped)
        cv2.imshow("MyImage", output)
        cv2.waitKey(3)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#
# import numpy as np
# import cv2
#
# # Load an color image in grayscale
# img = cv2.imread('feature_screen.png',0)
#
# cv2.imshow('image',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
