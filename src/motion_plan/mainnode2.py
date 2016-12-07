#!/usr/bin/env python

from gripnode import GripperClient

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
from baxter_core_msgs.msg import EndpointState
import planningnode as pnode

cameraStateInfo = None
pixelInfo = None
cameraInfo = None

# Left Arm Camera Position:
# Position: x = .745, y = .150, z = .499
# Orientation: x = -.003, y = .996, z = 0.011, w = .082


def convertTo3D(pixelInfo, camera_model, camera_x, camera_y, camera_z):
    val = camera_model.rectifyPoint(pixelInfo)
    print "-----------------------------PIXEl--------------------------------"
    print pixelInfo[0]
    print pixelInfo[1]
    print "-----------------------------VALUE--------------------------------"
    print val
    ray = camera_model.projectPixelTo3dRay(val)
    # ray = camera_model.projectPixelTo3dRay(pixelInfo)
    print "-----------------------------RAY--------------------------------"
    print ray
    print "-----------------------------CAMERA--------------------------------"
    print camera_x
    print camera_y
    print camera_z

    x = camera_x - ray[1]
    y = camera_y - ray[0]

    test = camera_model.project3dToPixel((y,x,camera_z+.33))
    print "-----------------------------NEW PIXEL--------------------------------"
    print test[0]
    print test[1]

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

if __name__=='__main__':
    
    rospy.init_node('plannode', log_level=rospy.INFO)
    gc = GripperClient()

    camera_model = image_geometry.PinholeCameraModel()

    cameraTopic = "/cameras/left_hand_camera/camera_info"
    pixelTopic = "/shell_game/pixel_point"
    cameraStateTopic = "/robot/limb/left/endpoint_state"

    rospy.Subscriber(cameraTopic, CameraInfo, initCamera)
    #rospy.Subscriber(pixelTopic, Point, getPixel)
    rospy.Subscriber(cameraStateTopic, EndpointState, getCameraState)

    rate = rospy.Rate(50)
    while(cameraInfo is None) or (cameraStateInfo is None):
        rate.sleep()

    camera_x = cameraStateInfo.pose.position.x
    camera_y = cameraStateInfo.pose.position.y
    camera_z = cameraStateInfo.pose.position.y

    # pixeldummy = Point()
    # pixeldummy.x = 27.0
    # pixeldummy.y = 52.05
    #pixeldummy.z = 89

    camera_model.fromCameraInfo(cameraInfo)
    coords = convertTo3D([370,250], camera_model, camera_x, camera_y, camera_z) #[0.48, -0.3]#
    print "-----------------COORDS----------------------"
    print coords
    rospy.sleep(2)

    # des_pose = [coords[0], coords[1], -0.32+0.19+0.05, 0.99, 0.01, 0.01, 0.01]

    # dsafe = [.17, 0.44, zsafe, 0.99, 0.01, 0.01, 0.01]

    dsafe = [coords[0], coords[1], zsafe, 0.99, 0.01, 0.01, 0.01]
    dpick = [coords[0], coords[1], zpick, 0.99, 0.01, 0.01, 0.01]
    ddrop = [0.75, -0.56, zdrop, 0.99, 0.01, 0.01, 0.01]
    rospy.sleep(2)

    print "\r\nTesting position 1"

    print "\r\nTesting position 1"
    gc.command(position=100.0, effort=50.0)
    gc.wait()
    rospy.sleep(5)
    
    pnode.initplannode(dsafe)
    rospy.sleep(5)
    pnode.initplannode(dpick)
    rospy.sleep(5)

    gc.command(position=4.0, effort=50.0)
    gc.wait()

    pnode.initplannode(dsafe)
    rospy.sleep(5)
    pnode.initplannode(ddrop)
    rospy.sleep(5)

    gc.command(position=100.0, effort=50.0)
    gc.wait()
    rospy.spin()