#!/usr/bin/env python

import rospy
import planningnode as pnode
from gripnode import GripperClient

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState
import planningnode as pnode

cameraStateInfo = None
pixelInfo = None
cameraInfo = None

def convertTo3D(pixelInfo, camera_model, camera_x, camera_y):
    ray = camera_model.projectPixelTo3dRay(pixelInfo)
    x = camera_x + ray[0]
    y = camera_y + ray[1]
    return (x, y)


def initCamera(data):
    global cameraInfo
    cameraInfo = data

def getPixel(data):
    global pixelInfo
    pixelInfo = data

def getCameraState(data):
    global cameraStateInfo
    cameraStateInfo = data


zsafe = -0.32+0.23+0.05
zdrop = -0.32+0.18+0.05
zpick = -0.32+0.13+0.05

def testnode(data):

    gc = GripperClient()

    print data
    print ((data.x-640)*0.0023*0.24 + 0.72 + 0.2)
    print ((data.y-400)*0.0023*0.24 + 0.17 + 0.2)
    coords = [ ((data.x-640)*0.0015*0.24 + 0.769 + 0.02), ((data.y-400)*0.0015*0.24 + 0.03 + 0.02)]
    print coords
    rospy.sleep(2)

    des_pose = [coords[0], coords[1], -0.32+0.19+0.05, 0.99, 0.01, 0.01, 0.01]

    dsafe = [coords[0], coords[1], zsafe, 0.99, 0.01, 0.01, 0.01]
    dpick = [coords[0], coords[1], zpick, 0.99, 0.01, 0.01, 0.01]
    ddrop = [0.75, -0.56, zdrop, 0.99, 0.01, 0.01, 0.01]
    rospy.sleep(2)

    print "Lets pick up the cup"

    

    gc.command(position=100.0, effort=50.0)
    gc.wait()
    rospy.sleep(2)
    
    pnode.initplannode(dsafe)
    rospy.sleep(2)
    pnode.initplannode(dpick)
    rospy.sleep(2)

    gc.command(position=70.0, effort=50.0)
    gc.wait()

    pnode.initplannode(dsafe)
    rospy.sleep(2)
    pnode.initplannode(ddrop)
    rospy.sleep(2)

    print "Lets drop the cup"

    gc.command(position=100.0, effort=50.0)
    gc.wait()

    

    return

if __name__ == '__main__':
    rospy.init_node('plannode', log_level=rospy.INFO)
    rospy.Subscriber("/treasure_point", Point, testnode)
    rospy.spin()