#!/usr/bin/env python

import rospy
import planningnode as pnode
from gripnode import GripperClient

zsafe = -0.32+0.23+0.05
zdrop = -0.32+0.18+0.05
zpick = -0.32+0.13+0.05

if __name__=='__main__':
    
    rospy.init_node('plannode', log_level=rospy.INFO)
    gc = GripperClient()

    dsafe = [0.52, -0.3, zsafe, 0.99, 0.01, 0.01, 0.01]
    dpick = [0.52, -0.3, zpick, 0.99, 0.01, 0.01, 0.01]
    ddrop = [0.75, -0.56, zdrop, 0.99, 0.01, 0.01, 0.01]

    print "\r\nTesting position 1"
    gc.command(position=100.0, effort=50.0)
    gc.wait()
    
    pnode.initplannode(dsafe)
    rospy.sleep(2)
    pnode.initplannode(dpick)
    rospy.sleep(2)

    gc.command(position=4.0, effort=50.0)
    gc.wait()

    pnode.initplannode(dsafe)
    rospy.sleep(2)
    pnode.initplannode(ddrop)
    rospy.sleep(2)

    gc.command(position=100.0, effort=50.0)
    gc.wait()