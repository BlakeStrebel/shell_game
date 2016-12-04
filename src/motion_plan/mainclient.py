#!/usr/bin/env python

import sys
import rospy
from shell_game.srv import *

def mainclient(x, y):
    rospy.wait_for_service('pix_to_loc')
    try:
        add_two_ints = rospy.ServiceProxy('pix_to_loc', pixToLoc)
        resp1 = pix_to_loc(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, mainclient(x, y))