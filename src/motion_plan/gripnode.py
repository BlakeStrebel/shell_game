import sys
import argparse

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

class GripperClient(object):
    def __init__(self):
        ns = 'robot/end_effector/right_gripper/'
        self._client = actionlib.SimpleActionClient(
            ns + "gripper_action",
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
                         (gripper.capitalize(),))
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()