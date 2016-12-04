#!/usr/bin/env python
import rospy
import baxter_interface

def camera_setup():
    limb = baxter_interface.Limb('left')
    home = {'left_w0': 1.3679273675968178, 'left_w1': 1.8338740319170121, 'left_w2': -0.1234854534247758, 'left_e0': -1.3591069780664766, 'left_e1': -0.04410194765170564, 'left_s0': -0.8881748761856546, 'left_s1': -0.27074760906177553}
    limb.move_to_joint_positions(home)

if __name__ == '__main__':
    rospy.init_node('leftarm_setup')
    try:
        camera_setup()
    except rospy.ROSInterruptException:
        pass

rospy.spin()
