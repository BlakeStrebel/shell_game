#!/usr/bin/env python

import struct
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg

from std_msgs.msg import Header

import time

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest)
import baxter_interface

from sensor_msgs.msg import JointState

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion)

from tf.transformations import quaternion_from_euler

def initplannode(des_pose):
    print "\r\nTesting position 1"
    print des_pose
    move_pos(des_pose)
    return

def ik_timeout(req, timeout=3.0):
    limb = "right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    base = rospy.Time.now()
    # set seed:
    req.seed_mode = req.SEED_USER
    right = baxter_interface.Limb('right')
    while (rospy.Time.now()-base).to_sec() <= timeout:
        # generate a random q:
        q = right.joint_angles()
        js = JointState(name=q.keys(), position=q.values() + np.pi/2*np.random.randn(7))
        req.seed_angles = np.array([js])
        try:
            rospy.wait_for_service(ns, 0.5)
            resp = iksvc(req)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.loginfo("Service exception")
            return None
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            break
    return resp

def move_pos(des_pose, timeout=3.0):
    #right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    limb = "right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    #quat1 = quaternion_from_euler(des_pose[3],des_pose[4],des_pose[5])
    pose = Pose()
    quat = Quaternion()
    quat.x = des_pose[3]
    quat.y = des_pose[4]
    quat.z = des_pose[5]
    quat.w = des_pose[6]
    pose.orientation = quat
    pose.position.x = des_pose[0]
    pose.position.y = des_pose[1]
    pose.position.z = des_pose[2]
    ikreq.pose_stamp = [PoseStamped(header=hdr, pose=pose)]
    # print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.loginfo("Service exception")

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp

        des_joints = [0]*7
        for i in range(7):
            des_joints[i] = resp.joints[0].position[i]
        ra = moveit_commander.MoveGroupCommander("right_arm")
        ra.set_joint_value_target(des_joints)
        ra.plan()
        ra.go()
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        print("Trying random seeds until timeout is reached")
        ikt = ik_timeout(ikreq,timeout=timeout)
        print "\r\n","ikt = "
        print ikt
        print ""
        if ikt is not None:
            des_joints = [0]*7
            for i in range(7):
                des_joints[i] = ikt.joints[0].position[i]
            ra = moveit_commander.MoveGroupCommander("right_arm")
            ra.set_joint_value_target(des_joints)
            ra.plan()
            ra.go()
    return

