#!/usr/bin/env python
import rospy
import baxter_interface

def dab():
    limb1 = baxter_interface.Limb('left')
    limb2 = baxter_interface.Limb('right')

    left1 = {'left_w0': -1.3437671701876224, 'left_w1': 1.57539826915832, 'left_w2': 1.8400099550685538, 'left_e0': -1.2689856067782086, 'left_e1': 1.4718545659760545, 'left_s0': -0.9782962474739226, 'left_s1': -0.7217379607000872}
    right1 = {'right_s0': -0.6611457195786133, 'right_s1': -0.9069661408372509, 'right_w0': -0.41800976469877527, 'right_w1': 0.24428644047075213, 'right_w2': 1.744903146219658, 'right_e0': -0.03144660615165098, 'right_e1': -0.05062136600021865}
    left2 = {'left_w0': -3.0311460368615775, 'left_w1': 0.007669903939427069, 'left_w2': 0.4007524808350643, 'left_e0': -2.6027819018445757, 'left_e1': 0.12885438618237474, 'left_s0': 0.6473398924876446, 'left_s1': -0.7363107781849986}
    right2 = {'right_s0': 1.022014699928657, 'right_s1': -0.9215389583221623, 'right_w0': 1.1861506442323961, 'right_w1': 1.435422522263776, 'right_w2': 1.4434759214001744, 'right_e0': 1.2233496783386175, 'right_e1': 1.3541215405058489}

    while 1:
        limb1.move_to_joint_positions(left1)
        limb2.move_to_joint_positions(right1)
        limb1.move_to_joint_positions(left2)
        limb2.move_to_joint_positions(right2)

if __name__ == '__main__':
    rospy.init_node('baxter_dab')
    try:
        dab()
    except rospy.ROSInterruptException:
        pass

rospy.spin()
