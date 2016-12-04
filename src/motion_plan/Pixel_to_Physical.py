#!/usr/bin/env python

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState

cameraInfo = None
pixelInfo = None
cameraStateInfo = None

def convertTo3D(pixelInfo, camera_model, camera_x, camera_y):
    ray = camera_model.projectPixelTo3dRay(pixelInfo)
    x = camera_x + (ray.x/1000)
    y = camera_y + (ray.y/1000)
    return (x, y)


def initCamera(data):
    global cameraInfo
    cameraInfo = data.data

def getPixel(data):
    global pixelInfo
    pixelInfo = data.data

def getCameraState(data):
    global cameraStateInfo
    cameraStateInfo = data.data


def main():
    rospy.init_node("convertpixeltocoord", anonymous=True)
    camera_model = image_geometry.PinholeCameraModel()

    cameraTopic = "/cameras/left_hand_camera/camera_info"
    pixelTopic = "/shell_game/pixel_point"
    cameraStateTopic = "/robot/limb/left/endpoint_state"

    rospy.Subscriber(cameraTopic, CameraInfo, initCamera)
    rospy.Subscriber(pixelTopic, Point, getPixel)
    rospy.Subscriber(cameraStateTopic, EndpointState, getCameraState)

    camera_x = cameraStateInfo.pose.position.x
    camera_y = cameraStateInfo.pose.position.y

    camera_model.fromCameraInfo(cameraInfo)
    coords = convertTo3D(pixelInfo, camera_model, camera_x, camera_y)





if __name__=="__main__":
    main()