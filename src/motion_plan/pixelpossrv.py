#!/usr/bin/env python

from shell_game.srv import pixToLoc
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


def handle_pix(req):
    print "Got pixels %f and %f" % (req.a, req.b)

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

    camera_model.fromCameraInfo(cameraInfo)
    coords = convertTo3D([req.a, req.b], camera_model, camera_x, camera_y)

    return pixToLocResponse(coords)

def pix_to_loc_server():
    rospy.init_node('pix_to_loc_server')
    s = rospy.Service('pix_to_loc', pixToLoc, handle_pix)
    print "Ready to show locations"
    rospy.spin()

if __name__ == "__main__":
    pix_to_loc_server()