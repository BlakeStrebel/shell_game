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
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.pts1 = deque(maxlen=32)
        self.pts2 = deque(maxlen=32)
        self.pts3 = deque(maxlen=32)
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

        if len(contours) > 2:
            contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
            c1 = contours[0]
            c2 = contours[1]
            c3 = contours[2]
            ((x1,y1),radius1) = cv2.minEnclosingCircle(c1)
            ((x2,y2),radius2) = cv2.minEnclosingCircle(c2)
            ((x3,y3),radius3) = cv2.minEnclosingCircle(c3)
            M1 = cv2.moments(c1)
            M2 = cv2.moments(c2)
            M3 = cv2.moments(c3)
            ballCenter1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
            ballCenter2 = (int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))
            ballCenter3 = (int(M3["m10"] / M3["m00"]), int(M3["m01"] / M3["m00"]))

            if radius1 > 10:
                cv2.circle(imgOriginal,(int(x1),int(y1)),int(radius1),(0,255,0),2)
                cv2.circle(imgOriginal,ballCenter1,5,(0,0,255),-1)
                self.pts1.appendleft(ballCenter1)
                for i in np.arange(1,len(self.pts1)):
                    thickness1 = int(np.sqrt(32/float(i+1))*2.5)
                    cv2.line(imgOriginal,self.pts1[i-1],self.pts1[i],(0,0,255),thickness1)

            if radius2 > 10:
                cv2.circle(imgOriginal,(int(x2),int(y2)),int(radius2),(0,255,0),2)
                cv2.circle(imgOriginal,ballCenter2,5,(0,0,255),-1)
                self.pts2.appendleft(ballCenter2)
                for i in np.arange(1,len(self.pts2)):
                    thickness2 = int(np.sqrt(32/float(i+1))*2.5)
                    cv2.line(imgOriginal,self.pts2[i-1],self.pts2[i],(0,0,255),thickness2)

            if radius3 > 10:
                cv2.circle(imgOriginal,(int(x3),int(y3)),int(radius3),(0,255,0),2)
                cv2.circle(imgOriginal,ballCenter3,5,(0,0,255),-1)
                self.pts3.appendleft(ballCenter3)
                for i in np.arange(1,len(self.pts3)):
                    thickness3 = int(np.sqrt(32/float(i+1))*2.5)
                    cv2.line(imgOriginal,self.pts3[i-1],self.pts3[i],(0,0,255),thickness3)

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
