#!/usr/bin/env python

from gripnode import GripperClient

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState, DigitalIOState
import planningnode as pnode
import baxter_interface

cameraStateInfo = None
pixelInfo = None
cameraInfo = None
global buttonPressed
treasurePoint = None

def convertTo3D(pixelInfo, camera_model, camera_x, camera_y, camera_z):

    print "-----------------------------PIXEl--------------------------------"
    print pixelInfo.x
    print pixelInfo.y
    val = camera_model.rectifyPoint([pixelInfo.x, pixelInfo.y])
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
    xray_norm = ray[0]/ray[2] * (camera_z)*.9170
    yray_norm = ray[1]/ray[2] * (camera_z)*1.536
    x = (camera_x - yray_norm)
    y = (camera_y - xray_norm)
    print "-------------------------X Y -------------------------"
    print (x,y)

    return (x, y)


def initCamera(data):
    global cameraInfo
    cameraInfo = data

def getCameraState(data):
    global cameraStateInfo
    cameraStateInfo = data

def getTreasurePoint(data):
    global treasurePoint
    treasurePoint = data


def testnode():
    print "----------------BUTTON VALUE----------"
    global buttonPressed
    print buttonPressed
    buttonPressed = False
    if not buttonPressed:

        print "Pass"
        zsafe = -0.32 + 0.33 + 0.05
        zdrop = -0.32 + 0.18 + 0.05
        zpick = -0.32 + 0.13 + 0.05

        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(cameraInfo)
        # print "--------------CAMERA_MODEL--------------"
        # print camera_model
        gc = GripperClient()

        # x: 0.683145146599
        # y: 0.0731765874398
        # z: 0.433126480405

        camera_x = cameraStateInfo.pose.position.x
        camera_y = cameraStateInfo.pose.position.y
        camera_z = cameraStateInfo.pose.position.z
        # print data
        #coords = convertTo3D(treasurePoint, camera_model, camera_x, camera_y, camera_z)  # [0.48, -0.3]#

        zoffset = -0.287811174717 # table height in baxter's frame
        pixel_size = .0023 # camera calibration (meter/pixels)
        h = camera_z-zoffset # height from table to camera
        x0 = camera_x # x camera position
        y0 = camera_y # y camera position
        x_offset = 0 # offsets
        y_offset = -.02
        height = 400 # image frame dimensions
        width = 640
        cx = treasurePoint.x
        cy = treasurePoint.y
        # Convert pixel coordinates to baxter coordinates
        xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
        yb = (cx - (width/2))*pixel_size*h + y0  + y_offset

        coords = [xb,yb]
        rospy.sleep(2)

        des_pose = [coords[0], coords[1], -0.32+0.19+0.05, 0.99, 0.01, 0.01, 0.01]

        dsafe = [coords[0], coords[1], zsafe, 0.99, 0.01, 0.01, 0.01]
        dpick = [coords[0], coords[1], zpick, 0.99, 0.01, 0.01, 0.01]
        ddropsafe = [0.75, -0.56, zsafe, 0.99, 0.01, 0.01, 0.01]
        ddrop = [0.75, -0.56, zdrop, 0.99, 0.01, 0.01, 0.01]
        leftsafe = [0.5, .5, zsafe, 0.99, 0.01, 0.01, 0.01]
        rospy.sleep(2)

        print "Lets pick up the cup"

        gc.command(position=100.0, effort=50.0)
        gc.wait()
        rospy.sleep(2)

        arm_goaway()
        rospy.sleep(0.1)
        pnode.initplannode(dsafe, "right")
        rospy.sleep(0.1)
        pnode.initplannode(dpick, "right")
        rospy.sleep(0.1)

        gc.command(position=50.0, effort=50.0)
        gc.wait()

        pnode.initplannode(dsafe, "right")
        rospy.sleep(0.1)
        pnode.initplannode(dpick, "right")
        rospy.sleep(0.1)
        # pnode.initplannode(ddropsafe, "right")
        # rospy.sleep(2)
        # pnode.initplannode(ddrop, "right")
        # rospy.sleep(0.1)
        # arm_setup()
        # rospy.sleep(0.1)

        print "Lets drop the cup"

        gc.command(position=100.0, effort=50.0)
        gc.wait()

        pnode.initplannode(dsafe, "right")
        rospy.sleep(0.1)
        pnode.initplannode(ddropsafe, "right")
        rospy.sleep(0.1)

        return

def button_press(data):
    global buttonPressed
    print data.state
    if data.state == 1:
        buttonPressed = True
        testnode()


def arm_goaway():
    left2_w0 = rospy.get_param('left2_w0', default=0)
    left2_w1 = rospy.get_param('left2_w1', default=0)
    left2_w2 = rospy.get_param('left2_w2', default=0)
    left2_e0 = rospy.get_param('left2_e0', default=0)
    left2_e1 = rospy.get_param('left2_e1', default=0)
    left2_s0 = rospy.get_param('left2_s0', default=0)
    left2_s1 = rospy.get_param('left2_s1', default=0)

    # Send the left arm to the desired position
    away = {'left_w0': left2_w0, 'left_w1': left2_w1, 'left_w2': left2_w2, 'left_e0': left2_e0, 'left_e1': left2_e1,
            'left_s0': left2_s0, 'left_s1': left2_s1}
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(away)


def arm_setup():
    # Get desired joint values from parameter server
    left_w0 = rospy.get_param('left_w0',default =0)
    left_w1 = rospy.get_param('left_w1',default =0)
    left_w2 = rospy.get_param('left_w2',default =0)
    left_e0 = rospy.get_param('left_e0',default =0)
    left_e1 = rospy.get_param('left_e1',default =0)
    left_s0 = rospy.get_param('left_s0',default =0)
    left_s1 = rospy.get_param('left_s1',default =0)

    # Send the left arm to the desired position
    home = {'left_w0': left_w0, 'left_w1': left_w1, 'left_w2': left_w2, 'left_e0': left_e0, 'left_e1': left_e1, 'left_s0': left_s0, 'left_s1': left_s1}
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(home)




if __name__ == '__main__':
    print "plan node is running"
    rospy.init_node('plannode', log_level=rospy.INFO)
    cameraTopic = "/cameras/left_hand_camera/camera_info"
    pixelTopic = "/shell_game/pixel_point"
    cameraStateTopic = "/robot/limb/left/endpoint_state"
    rightButtonTopic = "/robot/digital_io/right_button_ok/state"

    arm_setup()
    print "test1"
    rospy.Subscriber(cameraTopic, CameraInfo, initCamera)
    print "test2"
    rospy.Subscriber(cameraStateTopic, EndpointState, getCameraState)
    print "test3"
    rate = rospy.Rate(50)
    print "test4"
    while (cameraInfo is None) or (cameraStateInfo is None):
        rate.sleep()
    print "test5"
    rospy.Subscriber(rightButtonTopic, DigitalIOState, button_press)
    rospy.Subscriber("/treasure_cup_location", Point, getTreasurePoint)
    print "test6"
    rospy.spin()
