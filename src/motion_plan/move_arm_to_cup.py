#!/usr/bin/env python

import struct
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg

from std_msgs.msg import Header

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

def ik_timeout(req, timeout=3.0):
    limb = "right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    base = rospy.Time.now()
    # set seed:
    req.seed_mode = req.SEED_USER
    right = baxter_interface.Limb('right')
    while (rospy.Time.now() - base).to_sec() <= timeout:
        # generate a random q:
        q = right.joint_angles()
        js = JointState(name=q.keys(), position=q.values() + np.pi / 2 * np.random.randn(7))
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
    limb = "right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # quat = quaternion_from_euler(des_pose[3],des_pose[4],des_pose[5])
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
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp

        des_joints = [0] * 7
        for i in range(7):
            des_joints[i] = resp.joints[0].position[i]
        right_arm_group.set_joint_value_target(des_joints)
        right_arm_group.plan()
        right_arm_group.go()
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        print("Trying random seeds until timeout is reached")
        ikt = ik_timeout(ikreq, timeout=timeout)
        print "\r\n", "ikt = "
        print ikt
        print ""
        if ikt is not None:
            des_joints = [0] * 7
            for i in range(7):
                des_joints[i] = ikt.joints[0].position[i]
            right_arm_group.set_joint_value_target(des_joints)
            right_arm_group.plan()
            right_arm_group.go()
    return

if __name__ == '__main__':
    rospy.init_node('motion_planning', log_level=rospy.INFO)
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # add scene:
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(3.0)
    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.75
    p.pose.position.y = -0.25
    p.pose.position.z = 0.1

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.8
    p.pose.position.y = 0.025
    p.pose.position.z = -0.6
    scene.add_box("table", p, (0.75, 1.25, 0.68))

    rospy.spin()
