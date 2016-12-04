#!/usr/bin/env python

import rospy
import sys

timeout = 0

if __name__ == '__main__':
    rospy.init_node('testmodulesnode', log_level=rospy.INFO)
    while():
    	raw_input("Press ENTER to continue.")
    	print "Gotcha!"
    	timeout += 1
    timeout = 0
    rospy.spin()