#!/usr/bin/env python

import rospy
import planningnode as pnode

if __name__=='__main__':
    
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('plannode', log_level=rospy.INFO)

    des_pose = [0.52, -0.3, -0.32+0.13+0.05, 0.99, 0.01, 0.01, 0.01]

    print "\r\nTesting position 1"
    pnode.initplannode(des_pose)
    rospy.sleep(2)