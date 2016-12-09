import sys
import argparse

import rospy

# Specifically used to provide gripper effort
# ... and open/ close sliding positions
import actionlib

# Write to these topics to open or close gripper
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

# Send commands to Baxter
import baxter_interface

class GripperClient(object):
    def __init__(self):
        # Set to default right gripper
        # ... and subscribe to it's topic
        ns = 'robot/end_effector/right_gripper/'
        
        # Call action service
        self._client = actionlib.SimpleActionClient(
            ns + "gripper_action",
            GripperCommandAction,
        )
        
        # Set message of type GripperCommandGoal
        self._goal = GripperCommandGoal()
        self.clear()

    def command(self, position, effort):
        # How wide the grippers are from each other
        self._goal.command.position = position
        # How fast or slow they would move
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal)
        
    ### These functions are taken from
    # ... baxter_examples --> gripper_action_client.py
    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

# REFERENCES:
# gripper_action_client.py for package "BaxterExamples"
# Grippers were caliberated using "BaxterInterface" and "BaxterExamples" 
# ... and node gripper_keyboard.py
# Commands were taken from [http://sdk.rethinkrobotics.com/wiki/Grippers]
