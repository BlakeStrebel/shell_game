#!/usr/bin/env python
import rospy
import baxter_interface

def camera_setup():
    limb = baxter_interface.Limb('left')
    home = {'left_w0': 1.3081021168692866, 'left_w1': 2.091966299478733, 'left_w2': 1.1140535472017816, 'left_e0': -1.3410827038088229, 'left_e1': 0.09740778003072377, 'left_s0': -1.1136700520048104, 'left_s1': -0.5184855063052698}
    limb.move_to_joint_positions(home)

if __name__ == '__main__':
    rospy.init_node('leftarm_setup')
    try:
        camera_setup()
    except rospy.ROSInterruptException:
        pass

rospy.spin()
