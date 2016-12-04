#!/usr/bin/env python

import rospy
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState
import planningnode as pnode


cameraStateInfo = None
pixelInfo = None
cameraInfo = None

def convertTo3D(pixelInfo, camera_model, camera_x, camera_y):
    ray = camera_model.projectPixelTo3dRay(pixelInfo)
    print ray
    x = camera_x + (ray[0])
    y = camera_y + (ray[1])
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

def main():
    global cameraInfo
    global cameraStateInfo
    global pixelInfo
    rospy.init_node("convertpixeltocoord")
    camera_model = PinholeCameraModel()

    # cameraTopic = "/cameras/left_hand_camera/camera_info"
    # pixelTopic = "/shell_game/pixel_point"
    # cameraStateTopic = "/robot/limb/left/endpoint_state"
    rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, initCamera)
    #rospy.Subscriber(pixelTopic, Point, getPixel)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, getCameraState)

    rate = rospy.Rate(50)
    while (cameraInfo is None) or (cameraStateInfo is None):
        rate.sleep()

    camera_x = cameraStateInfo.pose.position.x
    camera_y = cameraStateInfo.pose.position.y

    pixeldummy = Point()
    pixeldummy.x = 27
    pixeldummy.y = 52
    #pixeldummy.z = 89

    camera_model.fromCameraInfo(cameraInfo)
    coords = convertTo3D([500,200], camera_model, camera_x, camera_y)
    print coords

    des_pose = [coords[0], coords[1], -0.32+0.19+0.05, 0.99, 0.01, 0.01, 0.01]

    print "\r\nTesting position 1"
    pnode.move_pos(des_pose)



if __name__=="__main__":
    main()