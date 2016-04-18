from __future__ import division, print_function
from cStringIO import StringIO
import sys
from baxter_pykdl import baxter_kinematics
import numpy as np
import rospy
import baxter_interface
from std_msgs.msg import Int32MultiArray, Float64
from sensor import sensor_node
from time import time


class ForceController(object):
    def __init__(self):
        self.limb_name = 'left'
        self.other_limb_name = 'right'
        self.limb = baxter_interface.Limb(self.limb_name)
        self.kinematics = baxter_kinematics(self.limb_name)
        self.jtp = self.kinematics.jacobian_transpose()
        self.endpteff = self.limb.endpoint_effort()

    def compute_joint_torques(self, jacobian_tranpose, endpoint_effort):
        joint_t = np.squeeze(np.asarray(self.jacobian_tranpose.dot(endpoint_effort)))
    	joint_t_dict = {'{}_{}'.format(self.limb_name, joint_name): val for
            joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'],
                                    joint_v)}
    	joint_t_dict.update({'{}_{}'.format(self.other_limb_name, joint_name): 0.0 for
            joint_name in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']})
    	joint_t_dict.update({'head_nod':0.0 , 'head_pan':0.0, 'torso_t0':0.0})
        return joint_t_dict


def main():
    rospy.init_node('baxter_forcecontroller')
    fc = FroceController()
    trqs = compute_joint_torques(self.jtp, self.endpteff)
    fc.limb.move_to_joint_position(trqs)

if __name__ == '__main__':
    main()
    #rospy.spin()
